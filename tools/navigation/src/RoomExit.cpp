//
// Created by tzuk on 6/25/23.
//

#include <iostream>
#include "navigation/include/RoomExit.h"

RoomExit::RoomExit(std::vector<Eigen::Vector3d> &data) : points(data) {
    auto center = findCenter(data);
    for (auto point: points) {
        point -= center;
    }
}

Eigen::Vector3d RoomExit::findCenter(std::vector<Eigen::Vector3d> &data) {
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
    for (auto &point: data) {
        center.x() += point.x();
        center.y() += point.y();
        center.z() += point.z();
    }
    center /= double(points.size());
    return center;
}

void RoomExit::getPolygonVertices() {
    findBestLines();
    polygonVertices = std::map<int, Eigen::Vector2d>{};

    for (auto &[slope, slicePoints]: slices) {
        if (slicePoints.size() > 5) {
            Eigen::Vector3d vertex = slicePoints[slicePoints.size() / 2];
            polygonVertices[slope] = Eigen::Vector2d(vertex.x(), vertex.z());
        }
    }
}

void RoomExit::createLines() {
    Eigen::Vector2d center(0, 0);
    for (int i = 0; i < 180; ++i) {
        double slope = tan((double(i) * M_PI) / double(180));
        lines.emplace_back(center, slope);
    }
}

void RoomExit::findBestLines() {
    createLines();
    auto to90 = std::vector<Line>(lines.begin(), lines.begin() + 90);
    auto to180 = std::vector<Line>(lines.begin() + 90, lines.end());
    auto sectors = dividePointsToAxesSectors();
    for (int i = 0; i < sectors.size(); i++) {
        auto goodLines = findBestLinesInSector(sectors[i], i % 2 == 0 ? to90 : to180);
        for (auto &line: goodLines) {
            int lineAngle = int((double(180) / M_PI) * std::atan(line.getSlope()) + 180 * (i / 2) + 360) % 360;
            if (!slices.count(lineAngle)) {
                slices[lineAngle] = std::vector<Eigen::Vector3d>{};
            }
        }
    }
    for (auto &sector: sectors) {
        for (auto &point: sector) {
            int pointAngle =
                    static_cast <int> (std::floor(atan2(point.z(), point.x()) * (double(180) / M_PI)) + 360) % 360;
            auto it = slices.lower_bound(pointAngle);
            if (it != slices.begin()) {
                it--;
                it->second.emplace_back(point);
            } else {
                slices.end()->second.emplace_back(point);
            }

        }
    }
}


std::vector<std::vector<Eigen::Vector3d>> RoomExit::dividePointsToAxesSectors() {
    std::vector<std::vector<Eigen::Vector3d>> sectors(4);
    for (auto &point: points) {
        if (point.x() >= 0 && point.z() >= 0) {
            sectors[0].emplace_back(point);
        } else if (point.x() >= 0 && point.z() < 0) {
            sectors[3].emplace_back(point);
        } else if (point.x() < 0 && point.z() >= 0) {
            sectors[1].emplace_back(point);
        } else if (point.x() < 0 && point.z() < 0) {
            sectors[2].emplace_back(point);
        }
    }
    return sectors;
}

std::vector<Line>
RoomExit::findBestLinesInSector(std::vector<Eigen::Vector3d> &sector, std::vector<Line> &sectorLines) {
    std::vector<Line> goodLines;
    double prevDistance = sectorLines.front().getSumOfDistanceToCloud(sector);
    double prevFirstDerivative = prevDistance;
    bool acc = true;
    bool positiveDirection = true;
    for (int i = 1; i < sectorLines.size(); i++) {
        double distance = sectorLines[i].getSumOfDistanceToCloud(sector);
        double firstDerivative = distance - prevDistance;
        if (firstDerivative >= 0) {
            positiveDirection = true;
        } else if (positiveDirection) {
            goodLines.emplace_back(sectorLines[i]);
            positiveDirection = false;
        }
        double secondDerivative = firstDerivative - prevFirstDerivative;
        if (secondDerivative > 0 and !acc) {
            acc = true;
            goodLines.emplace_back(sectorLines[i]);
        } else if (secondDerivative <= 0 and acc) {
            acc = false;
            goodLines.emplace_back(sectorLines[i]);
        }
        prevDistance = distance;
        prevFirstDerivative = firstDerivative;
    }
    return goodLines;
}

void RoomExit::calculateVariances() {
    for (auto &[angle, slice]: slices) {
        double avg = 0.0;
        for (auto &i: slice) {
            avg += i.norm();
        }
        avg /= double(slice.size());
        double variance = 0.0;
        for (auto &i: slice) {
            variance += std::pow(i.norm() - avg, 2);
        }
        variance /= double(slice.size() - 1);
        slicesVariances[angle] = variance;
    }
}

void RoomExit::filterByVariance() {
    double minVariance = std::numeric_limits<double>::max();
    double maxVariance = std::numeric_limits<double>::min();
    std::map<int, std::vector<Eigen::Vector3d>> filterSlices;
    for (auto &var: slicesVariances) {
        if (minVariance > var.second) {
            minVariance = var.second;
        } else if (maxVariance < var.second) {
            maxVariance = var.second;
        }
    }
    for (auto &[angle, slice]: slices) {
        double ratio = (slicesVariances[angle] - minVariance) / (maxVariance - minVariance);
        Line side(polygonVertices[angle], polygonVertices.upper_bound(angle)->second);

        for (auto &point: slice) {
            Eigen::Vector2d point2d(point.x(), point.z());
            double distance = side.getDistanceToPoint(point2d);
            if (distance > (1 - ratio) * 0.1 && !isInsidePolygon(point2d)) {
                if (not filterSlices.count(angle)){
                    filterSlices[angle] = std::vector<Eigen::Vector3d>{};
                }
                filterSlices[angle].emplace_back(point);
            }
        }
    }
    slices = filterSlices;
}

bool RoomExit::isInsidePolygon(Eigen::Vector2d &point) {
    int amountOfCrossing = 0;
    for (auto &[slope, vertex]: polygonVertices) {
        auto nextVertex = polygonVertices.upper_bound(slope + 1)->second;
        if ((vertex.x() < point.x() && point.x() < nextVertex.x()) or
            (vertex.x() > point.x() && point.x() > nextVertex.x())) {
            double ratio = (point.x() - nextVertex.x()) / (vertex.x() - nextVertex.x());
            amountOfCrossing += (ratio * vertex.y() + ((1 - ratio) * nextVertex.y())) >= point.y() ? 1 : 0;
        }
    }
    return amountOfCrossing % 2 != 0;
}

std::vector<std::pair<double, Eigen::Vector3d>> RoomExit::getExitPointsByVariance() {
    std::vector<std::pair<double, Eigen::Vector3d>> exitPoints;
    std::vector<int> angles;
    for (auto &[angle, slice]: slices) {
        angles.emplace_back(angle);
    }
    std::unordered_map<int, Line> biSectors;
    Eigen::Vector2d zero(0, 0);
    for (int i = 0; i < angles.size() - 1; ++i) {
        biSectors[angles[i]] = Line(zero, (double(angles[i] + angles[i + 1]) / 2) * (M_PI / double(180)));
    }
    biSectors[angles.back()] = Line(zero, (double(angles.back() + angles.front()) / 2) * (M_PI / double(180)));

    for (auto &[angle, slicePoints]: slices) {
        if (slicePoints.size() <5){
            continue;
        }
        auto biSector = biSectors[angle];
        double minDistance = std::numeric_limits<double>::max();
        Eigen::MatrixXd covarianceMatrix(slicePoints.size(), 2);
        int i = 0;
        Eigen::Vector3d exitPoint;
        Eigen::Vector2d means(0, 0);
        double avgHeight = 0;
        for (auto &point: slicePoints) {

            Eigen::Vector2d point2d(point.x(), point.z());
            double distance = biSector.getDistanceToPoint(point2d);
            means.x() += point2d.x();
            means.y() += point2d.y();
            avgHeight += point.y();
            i += 1;
            if (distance < minDistance) {
                exitPoint = point;
            }
        }
        means /= double(slicePoints.size());
        avgHeight /= double(slicePoints.size());
        i = 0;
        double varianceHeight = 0;
        for (auto &point: slicePoints) {
            Eigen::Vector2d point2d(point.x() - means.x(), point.z() - means.y());
            covarianceMatrix(i, 0) = point2d.x();
            covarianceMatrix(i, 1) = point2d.y();
            varianceHeight += std::pow(point.y() - avgHeight, 2);
            i+=1;
        }
        varianceHeight /= double(slicePoints.size() - 1);
        covarianceMatrix = covarianceMatrix.transpose() * covarianceMatrix;
        auto trace = covarianceMatrix.trace();
        auto det = covarianceMatrix.determinant();
        double grade = (det / trace) / varianceHeight;
        exitPoints.emplace_back(grade, exitPoint);
    }
    return exitPoints;
}

std::vector<std::pair<double, Eigen::Vector3d>> RoomExit::getExitPoints() {
    getPolygonVertices();
    calculateVariances();
    filterByVariance();
    return getExitPointsByVariance();
}

Line::Line(Eigen::Vector2d &point, double slope) : slope(slope), origin(point) {
    yIntercept = point.y() - point.x() * slope;
    dest = Eigen::Vector2d(1, slope * 1 + yIntercept);
}

double Line::getSumOfDistanceToCloud(std::vector<Eigen::Vector3d> &cloud) {
    double distance = 0.0;
    for (auto &point: cloud) {
        Eigen::Vector2d point2d(point.x(), point.z());
        distance += getDistanceToPoint(point2d);
    }
    return distance;
}

double Line::getDistanceToPoint(Eigen::Vector2d &point) {
    return std::abs(slope * point.x() - point.y() + yIntercept) / std::sqrt(std::pow(slope, 2) + 1);
}

Line::Line(Eigen::Vector2d &point1, Eigen::Vector2d &point2) : origin(point1), dest(point2) {
    slope = (point2.y() - point1.y()) / (point2.x() - point1.x());
    yIntercept = point1.y() - slope * point1.x();
}

Line::Line() : slope(0), origin(), dest(), yIntercept(0) {

}
