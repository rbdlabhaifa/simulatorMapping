#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

class OfflineMapPoint {
public:
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint);

    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors);

    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal;
        for (auto desc : this->descriptors)
            ss << ", " << desc;
        return ss.str();
    }

    bool compare(OfflineMapPoint offlineMapPoint);

    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default;

    bool operator==(const cv::Point3d& anotherPoint);

    cv::Point3d point;
    double minDistanceInvariance;
    double maxDistanceInvariance;
    cv::Point3d normal;
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints;
    std::vector<cv::Mat> descriptors;
};

#endif // OFFLINE_MAP_POINT_H
