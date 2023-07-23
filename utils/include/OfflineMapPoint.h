#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

class OfflineMapPoint {
public:
    // this function receives another OfflineMapPoint instances and construces a copy of it
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint);

    // this function receives values for the fields of an offlineMapPoint instances and construces one with those values
    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors);

    // sets a conversion to string format for OfflineMapPoint
    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal;
        for (auto desc : this->descriptors)
            ss << ", " << desc;
        return ss.str();
    }

    // comapres between 2 offlineMapPoint by comparing their this->point fields
    bool compare(OfflineMapPoint offlineMapPoint);

    // defining a behavior the = operator using the default method
    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default;

    // defining a behavior the == operator using this->point field
    bool operator==(const cv::Point3d& anotherPoint);

    cv::Point3d point; // the coordinates of the mappoint (x, y, z)
    double minDistanceInvariance;
    double maxDistanceInvariance;
    cv::Point3d normal; // the normal of this point (average of normalized distances between the camera and the keypoints at each frame)
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints; // all keypoints correlating to this mappoint
    std::vector<cv::Mat> descriptors; // all descriptors of the keypoints correlating to this mappoint
};

#endif // OFFLINE_MAP_POINT_H
