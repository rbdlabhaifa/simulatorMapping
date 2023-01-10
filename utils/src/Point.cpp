//
// Created by rbdstudent on 15/06/2021.
//

#include "include/Point.h"

#include <utility>


Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    rotationMatrix = cv::Mat::zeros(3,3,CV_64F);
    this->label = -1;
    this->frameId = 0;
}

Point::Point(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
    this->label = -1;
    this->frameId = 0;
}

Point::Point(const Point &point) {
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;
    this->rotationMatrix = point.rotationMatrix;
    this->label = point.label;
    this->frameId = point.frameId;
}

Point::Point(double x, double y, double z,const cv::Mat &rotationMatrix, int frameId, int label) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->rotationMatrix =rotationMatrix;
    this->label = label;
    this->frameId = frameId;
}

bool Point::compare(Point point) {
    if (this->x == point.x &&
        this->y == point.y &&
        this->z == point.z)
        return true;
    return false;
}

Point2D::Point2D(double x, double y) {
    this->x = x;
    this->y = y;
}
