#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

class OfflineMapPoint {
public:
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint); //copy constructor

    //constructor
    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors);

    std::string to_string() const { //converts all the data to string, and returns it.
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal;
        for (auto desc : this->descriptors)
            ss << ", " << desc;
        return ss.str();
    }

    bool compare(OfflineMapPoint offlineMapPoint);  //returns if the points are equal. the argument is OfflineMapPoint variable

    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default; //default = operator overloading. 

    bool operator==(const cv::Point3d& anotherPoint); //returns if the points are equal. the argument is Point3D variable

    cv::Point3d point; //3D point
    double minDistanceInvariance; 
    double maxDistanceInvariance; 
    cv::Point3d normal; //the normal of the point
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints; //vector of the keypoints and their frameId
    std::vector<cv::Mat> descriptors; //vector that contains the descriptors
};

#endif // OFFLINE_MAP_POINT_H
