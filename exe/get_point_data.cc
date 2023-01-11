#include <string>
#include <iostream>
#include <nlohmann/json.hpp>

#include "include/Point.h"
#include "include/Auxiliary.h"

#define X1 (0.0649042)
#define Y1 (-0.180186)
#define Z1 (0.51533)

#define X2 (0.118669)
#define Y2 (-0.158835)
#define Z2 (0.513098)

void getFramesWithPoints(std::vector<std::string> row, std::map<int, Point2D>* framesWithPoint) {
    for(int i=3; i<row.size(); i+=3) {
        Point2D currentPoint(std::stod(row[i+1]), std::stod(row[i+2]));
        (*framesWithPoint).insert(std::make_pair(std::stoi(row[i]), currentPoint))  ;
    }
}

bool searchPoint(std::string csvPath, Point point, std::map<int, Point2D>* framesWithPoint) {
    std::fstream pointData;
    pointData.open(csvPath, std::ios::in);

    std::vector<std::string> row;
    std::string line, word, temp;

    while (!pointData.eof()) {
        Point pointToCompare;

        row.clear();
        
        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            row.push_back(word);
        }
        
        pointToCompare = Point(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
        if (point.compare(pointToCompare)) {
            getFramesWithPoints(row, framesWithPoint);
            pointData.close();
            return true;
        }
    }
    pointData.close();
    return false;
}

void searchAllPoints(std::string csvPath, std::vector<Point> points, std::vector<std::map<int, Point2D>>* framesWithPoints) {
    for (auto point : points) {
        std::map<int, Point2D> framesWithPoint;
        if (!searchPoint(csvPath, point, &framesWithPoint)) {
            std::cout << "Point (" << point.x << ", " << point.y << ", " << point.z << ") is not in the cloud points!" << std::endl;
        }
        (*framesWithPoints).push_back(framesWithPoint);
    }
}

void printFrames(std::vector<Point> points, std::vector<std::map<int, Point2D>> framesWithPoints) {
    for (int i=0; i < framesWithPoints.size(); i++) {
        std::cout << "Point (" << points[i].x << ", " << points[i].y << ", " << points[i].z << ")" << std::endl;
        for(const auto &myPair : framesWithPoints[i])
        {
            std::cout << "\t" << myPair.first << ": (" << myPair.second.x << ", " << myPair.second.y << ")" << std::endl;
        }
    }
}

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string csvPath = data["getPointDataCsv"];
    std::vector<std::map<int, Point2D>> framesWithPoints;
    std::vector<Point> points;

    points.push_back(Point(X1, Y1, Z1));
    points.push_back(Point(X2, Y2, Z2));

    searchAllPoints(csvPath, points, &framesWithPoints);

    printFrames(points, framesWithPoints);

    return 0;
}

