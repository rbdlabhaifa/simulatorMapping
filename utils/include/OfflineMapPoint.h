#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

class OfflineMapPoint {
public:
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint);

    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, cv::Mat descriptor);

    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal << ", " << this->descriptor;
        return ss.str();
    }

    bool compare(OfflineMapPoint offlineMapPoint);

    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default;

    cv::Point3d point;
    double minDistanceInvariance;
    double maxDistanceInvariance;
    cv::Point3d normal;
    cv::Mat descriptor;
};

#endif // OFFLINE_MAP_POINT_H
