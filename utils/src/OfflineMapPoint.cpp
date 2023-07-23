/*********** add-comments , task1 ***********/
#include "include/OfflineMapPoint.h"

#include <utility>

//initialize all the fields of Offline MapPoint
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

//initialize all the fields of Offline MapPoint
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

//check if this 3D point and the other 3D point are similar
bool OfflineMapPoint::operator==(const cv::Point3d& anotherPoint) {
    return this->point == anotherPoint;
}

//check if this 3D point and the other offlineMapPoint's 3D point are similar
bool OfflineMapPoint::compare(OfflineMapPoint offlineMapPoint) {
    if (this->point == offlineMapPoint.point)
        return true;
    return false;

    //we can write it in one line
    //return (this->point == offlineMapPoint.point);
}