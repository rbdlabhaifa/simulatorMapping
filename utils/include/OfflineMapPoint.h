#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

class OfflineMapPoint {
public:
    //recieves a different instance of offlineMapPoint
    //create a new instance with the same member values as the one it got
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint);

    //recieves a value for each memeber
    //create a new instance with the arguments values
    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors);

    //recieves an ostringstream instance
    //returns a string of information about this Map Point
    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal;
        for (auto desc : this->descriptors)
            ss << ", " << desc;
        return ss.str();
    }

    //recieves an offlineMapPoint
    //returns wether they represents the same Map Point or not
    bool compare(OfflineMapPoint offlineMapPoint); // can be const + get a reference

    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default;

    //receive a 3d Point
    //returns wether the given Point is the Map Point of this instance
    bool operator==(const cv::Point3d& anotherPoint); // can be const

    cv::Point3d point;
    double minDistanceInvariance;
    double maxDistanceInvariance;
    cv::Point3d normal;
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints;
    std::vector<cv::Mat> descriptors;
};

#endif // OFFLINE_MAP_POINT_H
