#include "MapControl/OfflineMapPoint.h"

#include <utility>

OfflineMapPoint::OfflineMapPoint(const OfflineMapPoint &offlineMapPoint) {
    this->point = offlineMapPoint.point;
    this->minDistanceInvariance = offlineMapPoint.minDistanceInvariance;
    this->maxDistanceInvariance = offlineMapPoint.maxDistanceInvariance;
    this->normal = offlineMapPoint.normal;
    this->bestKeyPoint = offlineMapPoint.bestKeyPoint;
    this->bestDescriptor = offlineMapPoint.bestDescriptor;
    this->keyPoints = offlineMapPoint.keyPoints;
    this->descriptors = std::vector<cv::Mat>();
    for (auto desc : offlineMapPoint.descriptors)
        this->descriptors.push_back(desc.clone());
}

OfflineMapPoint::OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, cv::KeyPoint bestKeyPoint, cv::Mat bestDescriptor, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors) {
    this->point = point;
    this->minDistanceInvariance = minDistanceInvariance;
    this->maxDistanceInvariance = maxDistanceInvariance;
    this->normal = normal;
    this->bestKeyPoint = bestKeyPoint;
    this->bestDescriptor = bestDescriptor;
    this->keyPoints = keyPoints;
    this->descriptors = std::vector<cv::Mat>();
    for (auto desc : descriptors)
        this->descriptors.push_back(desc.clone());
}

bool OfflineMapPoint::operator==(const cv::Point3d& anotherPoint) {
    return this->point == anotherPoint;
}

bool OfflineMapPoint::compare(OfflineMapPoint offlineMapPoint) {
    if (this->point == offlineMapPoint.point)
        return true;
    return false;
}