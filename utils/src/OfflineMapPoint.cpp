#include "include/OfflineMapPoint.h"

#include <utility>

OfflineMapPoint::OfflineMapPoint(const OfflineMapPoint &offlineMapPoint) {
    this->point = offlineMapPoint.point;
    this->minDistanceInvariance = offlineMapPoint.minDistanceInvariance;
    this->maxDistanceInvariance = offlineMapPoint.maxDistanceInvariance;
    this->normal = offlineMapPoint.normal;
    this->descriptor = offlineMapPoint.descriptor;
}

OfflineMapPoint::OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, cv::Mat descriptor) {
    this->point = point;
    this->minDistanceInvariance = minDistanceInvariance;
    this->maxDistanceInvariance = maxDistanceInvariance;
    this->normal = normal;
    this->descriptor = descriptor;
}

bool matCompare(const cv::Mat& a, const cv::Mat& b) {
        if (a.rows != b.rows || a.cols != b.cols || a.type() != b.type()) {
            return false;
        }
        for (int i = 0; i < a.rows; i++) {
            const void* a_row = a.ptr(i);
            const void* b_row = b.ptr(i);
            if (memcmp(a_row, b_row, a.cols*a.elemSize()) != 0) {
                return false;
            }
        }
        return true;
    }

bool OfflineMapPoint::operator==(const cv::Point3d& anotherPoint) {
    return this->point == anotherPoint;
}

bool OfflineMapPoint::compare(OfflineMapPoint offlineMapPoint) {
    if (this->point == offlineMapPoint.point && matCompare(this->descriptor, offlineMapPoint.descriptor))
        return true;
    return false;
}