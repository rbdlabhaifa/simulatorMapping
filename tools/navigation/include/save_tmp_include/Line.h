//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_LINE_H
#define ORB_SLAM2_LINE_H

#include "Point.h"
#include <cmath>

class Line {
public:
    Line(const Point& point1, const Point& point2);

    Line(const Point& point, double slope);
    double getSlope() const{
        return slope;
    }
    Point getPoint2(){
        return point2;
    }
    Point getPoint1(){
        return point1;
    }
    double getDistanceToPoint(const Point& point) const;
    double getDistanceToSegment(const Point& point) const;

    Point getLineIntersection(const Line& line);

    double det(const Point &point1, const Point &point2);

private:
    Point point1;
    Point point2;
    double slope;
    double yIntercept;
};


#endif //ORB_SLAM2_LINE_H