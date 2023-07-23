#include "include/OfflineMapPoint.h"

#include <utility>

OfflineMapPoint::OfflineMapPoint(const OfflineMapPoint &offlineMapPoint) {
    this->point = offlineMapPoint.point;
    this->minDistanceInvariance = offlineMapPoint.minDistanceInvariance;
    this->maxDistanceInvariance = offlineMapPoint.maxDistanceInvariance;
    this->normal = offlineMapPoint.normal;
    this->keyPoints = offlineMapPoint.keyPoints;
    this->descriptors = std::vector<cv::Mat>();
    for (auto desc : offlineMapPoint.descriptors)
        this->descriptors.push_back(desc.clone());
}

OfflineMapPoint::OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors) {
    this->point = point;
    this->minDistanceInvariance = minDistanceInvariance;
    this->maxDistanceInvariance = maxDistanceInvariance;
    this->normal = normal;
    this->keyPoints = keyPoints;
    this->descriptors = std::vector<cv::Mat>();
    for (auto desc : descriptors)
        this->descriptors.push_back(desc.clone());
}

bool OfflineMapPoint::operator==(const cv::Point3d& anotherPoint) { //returns if the points are equal. the argument is Point3D variable
    return this->point == anotherPoint;
}

//why not const reference
bool OfflineMapPoint::compare(OfflineMapPoint offlineMapPoint) { //returns if the points are equal. the argument is OfflineMapPoint variable.
    if (this->point == offlineMapPoint.point)
        return true;
    return false;
}