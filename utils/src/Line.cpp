//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Line.h"
#include "include/Auxiliary.h"

Line::Line(const Point& point1, const Point& point2) {
    this->point1 = point1;
    this->point2 = point2;
    slope = (point2.y - point1.y) / (point2.x - point1.x);
    yIntercept = point1.y - slope * point1.x;
}

Line::Line(const Point& point, double slope) {
    point1 = point;
    yIntercept = point.y - point.x * slope;
    point2 = Point(point.x + 100,(point.x +100 )*slope + yIntercept,point.z);
    this->slope = slope;

}

double Line::getDistanceToSegment(const Point& point) const {
    Point sideDifference(point.x - point1.x, point.y - point1.y, 0);
    Point segmentDifference(point2.x - point1.x, point2.y - point1.y, 0);
    double segmentLength = pow(segmentDifference.x, 2) + pow(segmentDifference.y, 2);
    double param =
            segmentLength != 0 ? sideDifference.x * segmentDifference.x + sideDifference.y * segmentDifference.y : -1;
    double distanceX;
    double distanceY;
    if (param < 0) {
        distanceX = point1.x;
        distanceY = point1.y;
    } else if (param > 1) {
        distanceX = point2.x;
        distanceY = point2.y;
    } else {
        distanceX = point1.x + param * segmentDifference.x;
        distanceY = point1.y + param * segmentDifference.y;
    }
    return sqrt(pow(distanceX, 2) + pow(distanceY, 2));
}

double Line::getDistanceToPoint(const Point& point) const {
    return std::abs(slope * point.x - point.y + yIntercept) / std::sqrt(std::pow(slope, 2) + 1);
}

Point Line::getLineIntersection(const Line& line) {
    Point xDifference(point1.x - point2.x, line.point1.x - line.point2.x, 0);
    Point yDifference(point1.y - point2.y, line.point1.y - line.point2.y, 0);
    double div = Auxiliary::det(xDifference, yDifference);
    if (div == 0.0) {
        return {};
    }
    Point detPoint = Point(Auxiliary::det(point1, point2), Auxiliary::det(line.point1, line.point2), 0);
    return {Auxiliary::det(detPoint, xDifference) / div, Auxiliary::det(detPoint, yDifference) / div, point1.z};
}