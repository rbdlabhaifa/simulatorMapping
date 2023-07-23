#include "include/OfflineMapPoint.h"

#include <utility>

OfflineMapPoint::OfflineMapPoint(const OfflineMapPoint &offlineMapPoint) {
    // setting the values for the OfflineMapPoint constructed
    this->point = offlineMapPoint.point;
    this->minDistanceInvariance = offlineMapPoint.minDistanceInvariance;
    this->maxDistanceInvariance = offlineMapPoint.maxDistanceInvariance;
    this->normal = offlineMapPoint.normal;
    this->keyPoints = offlineMapPoint.keyPoints;
    this->descriptors = std::vector<cv::Mat>();

    // cloning the descriptors given and putting them in this->descriptors
    for (auto desc : offlineMapPoint.descriptors)
        this->descriptors.push_back(desc.clone());
}

OfflineMapPoint::OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, 
                                cv::Point3d normal, std::vector<std::pair<long unsigned int,
                                cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors) {
    // setting the values for the OfflineMapPoint constructed
    this->point = point;
    this->minDistanceInvariance = minDistanceInvariance;
    this->maxDistanceInvariance = maxDistanceInvariance;
    this->normal = normal;
    this->keyPoints = keyPoints;
    this->descriptors = std::vector<cv::Mat>();

    // cloning the descriptors given and putting them in this->descriptors
    for (auto desc : descriptors)
        this->descriptors.push_back(desc.clone());
}

bool OfflineMapPoint::operator==(const cv::Point3d& anotherPoint) {
    // comparing between OfflineMapPoint and cv::Point3d object using this->point field
    return this->point == anotherPoint;
}

bool OfflineMapPoint::compare(OfflineMapPoint offlineMapPoint) {
    // comparing between 2 OfflineMapPoint instances using this->point field
    if (this->point == offlineMapPoint.point)
        return true;
    return false;
}