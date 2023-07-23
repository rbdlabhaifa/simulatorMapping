#ifndef OFFLINE_MAP_POINT_H
#define OFFLINE_MAP_POINT_H

#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

//This is the class of OfflineMapPoint. It stores information about a 3D point in world coordinates along with its associated keypoint indices, 2D keypoints, and descriptors.
class OfflineMapPoint {
public:
    // Constructor, Creates an OfflineMapPoint object by copying another OfflineMapPoint
    OfflineMapPoint(const OfflineMapPoint &offlineMapPoint);

    // Constructor, Creates an OfflineMapPoint object with specified parameters
    OfflineMapPoint(cv::Point3d point, double minDistanceInvariance, double maxDistanceInvariance, cv::Point3d normal, std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints, std::vector<cv::Mat> descriptors);

    // Convert the OfflineMapPoint to a string representation
    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << this->point << ", " << this->minDistanceInvariance << ", " << this->maxDistanceInvariance << ", " << this->normal;
        for (auto desc : this->descriptors)
            ss << ", " << desc;
        return ss.str();
    }

    // Compare two OfflineMapPoint objects for equality
    bool compare(OfflineMapPoint offlineMapPoint);

    // Overload the assignment operator for the OfflineMapPoint class.
    OfflineMapPoint &operator=(const OfflineMapPoint &offlineMapPoint) = default;

    // Overload the equality operator to compare an OfflineMapPoint with a Point3d object.
    bool operator==(const cv::Point3d& anotherPoint);

    // Public data members of the OfflineMapPoint class
    cv::Point3d point;                            
    double minDistanceInvariance;                
    double maxDistanceInvariance;                
    cv::Point3d normal;                          
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints;  // List of keypoint indices and their corresponding 2D keypoints
    std::vector<cv::Mat> descriptors;             // List of descriptors associated with the 2D keypoints
};

#endif // OFFLINE_MAP_POINT_H
