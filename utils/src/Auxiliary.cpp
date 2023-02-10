//
// Created by rbdstudent on 17/06/2021.
//

#include <fstream>
#include "include/Auxiliary.h"

double Auxiliary::det(const Point &point1, const Point &point2) {
    return point1.x * point2.y - point1.y * point2.x;
}

std::string Auxiliary::GetGeneralSettingsPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../generalSettings.json";
    return settingPath;
}


cv::Mat createIntrinsicMatrix(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2, int width, int height)
{
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                 fx,  k1*fy, cx,
                 0.0,      fy, cy,
                 0.0,     0.0,  1.0
               );

    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);

    cv::Mat r = cv::Mat::eye(3,3,CV_64F);

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, distCoeffs, r, K, cv::Size(width, height), CV_32FC1, map1, map2);

    return K;
}

static bool Auxiliary::isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree)
{
    image_geometry::PinholeCameraModel cam_model;

    // Set the camera intrinsic parameters
    cam_model.fx = fx;
    cam_model.fy = fy;
    cam_model.cx = cx;
    cam_model.cy = cy;
    cam_model.k1 = k1;
    cam_model.k2 = k2;
    cam_model.k3 = k3;
    cam_model.p1 = p1;
    cam_model.p2 = p2;

    // Set the camera image size
    cam_model.width = width;
    cam_model.height = height;

    // Define the 3D point expressed in the world frame
    cv::Point3d world_point(point.x, point.y, point.z);


    // Define the position and orientation of the camera
    double roll = roll_degree * CV_PI / 180.0;
    double pitch = pitch_degree * CV_PI / 180.0;
    double yaw = yaw_degree * CV_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << roll, pitch, yaw);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << cameraPos.x, cameraPos.y, cameraPos.z);

    cv::Mat K = createIntrinsicMatrix(fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height);
    cv::Mat 

    // Project the 3D point onto the image plane
    cv::Point2d image_point;
    image_point = cv::projectPoint(world_point, rvec, tvec, cam_model.intrinsicMatrix(), cam_model.distortionCoeffs());

    // The image_point variable now contains the 2D projection of the 3D point on the image plane
    if (image_point.x >= 0 && image_point.x < width && image_point.y >= 0 && image_point.y < height)
    {
        return true;
    }

    return false;
}


// bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float yaw, float pitch,
//                     int imageWidth, int imageHeight, float fov) {
//     // Convert yaw and pitch to rotation matrix
//     cv::Mat rot = cv::Mat::zeros(3, 3, CV_32F);
//     cv::Rodrigues(cv::Vec3f(pitch, yaw, 0), rot);

//     // Calculate camera matrix
//     float f = imageWidth / (2 * tan(fov / 2));
//     cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 
//         f, 0, imageWidth / 2, 
//         0, f, imageHeight / 2, 
//         0, 0, 1);

//     // Transform point to camera space
//     cv::Mat pointCameraSpace = rot * (cv::Mat)point + cameraPos;

//     // Check if point is in front of camera
//     if (pointCameraSpace.at<float>(2, 0) <= 0) {
//         return false;
//     }

//     // Project point to image space
//     cv::Mat imagePoint;
//     cv::projectPoints(cv::Mat(point), cv::Mat::zeros(3, 1, CV_32F), cv::Mat::zeros(3, 1, CV_32F), cameraMatrix, cv::Mat::zeros(4, 1, CV_32F), imagePoint);

//     // Check if point is within image bounds
//     int x = imagePoint.at<float>(0, 0);
//     int y = imagePoint.at<float>(1, 0);
//     if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight) {
//         return true;
//     } else {
//         return false;
//     }
// }


// bool Auxiliary::isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float yaw, float pitch,
//                     int imageWidth, int imageHeight) {

//   // Calculate the rotation matrix based on yaw and pitch
//   cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
//   cv::Rodrigues(cv::Vec3f(0, pitch, yaw), rot);

//   // Transform the point from world space to camera space
//   cv::Mat pointCameraSpace = rot * cv::Mat(point - cameraPos);

//   // Check if the point is in front of the camera
//   if (pointCameraSpace.at<float>(2, 0) <= 0) {
//     return false;
//   }

//   // Project the point from camera space to image space
//   cv::Mat pointImageSpace = cv::Mat::zeros(3, 1, CV_32FC1);
//   pointImageSpace.at<float>(0, 0) = pointCameraSpace.at<float>(0, 0) / pointCameraSpace.at<float>(2, 0);
//   pointImageSpace.at<float>(1, 0) = pointCameraSpace.at<float>(1, 0) / pointCameraSpace.at<float>(2, 0);

//   // Check if the point is within the image bounds
//   if (pointImageSpace.at<float>(0, 0) < 0 || pointImageSpace.at<float>(0, 0) >= imageWidth ||
//       pointImageSpace.at<float>(1, 0) < 0 || pointImageSpace.at<float>(1, 0) >= imageHeight) {
//     return false;
//   }

//   return true;
// }


// bool Auxiliary::isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float yaw, float pitch,
//                     int imageWidth, int imageHeight)
// {
//     cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
//         1, 0, 0,
//         0, 1, 0,
//         0, 0, 1
//     );
//     cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F);

//     // Calculate the rotation matrix from yaw and pitch angles
//     cv::Mat rot = cv::Mat::eye(3, 3, CV_32F);
//     rot.at<float>(0, 0) = cos(yaw) * cos(pitch);
//     rot.at<float>(0, 1) = sin(yaw) * cos(pitch);
//     rot.at<float>(0, 2) = -sin(pitch);
//     rot.at<float>(1, 0) = -sin(yaw);
//     rot.at<float>(1, 1) = cos(yaw);
//     rot.at<float>(2, 0) = cos(yaw) * sin(pitch);
//     rot.at<float>(2, 1) = sin(yaw) * sin(pitch);
//     rot.at<float>(2, 2) = cos(pitch);

//     // Convert camera position to a matrix
//     cv::Mat cameraPosMat = cv::Mat(cameraPos);

//     // Transform the point into the camera coordinate system
//     cv::Mat pointCam = rot * (cv::Mat(point) - cameraPosMat);

//     // Project the point into the image plane
//     float x = cameraMatrix.at<float>(0, 0) * pointCam.at<float>(0, 0) / pointCam.at<float>(2, 0) +
//               cameraMatrix.at<float>(0, 2);
//     float y = cameraMatrix.at<float>(1, 1) * pointCam.at<float>(1, 0) / pointCam.at<float>(2, 0) +
//               cameraMatrix.at<float>(1, 2);

//     // Check if the point is within the bounds of the image
//     return x >= 0 && x < imageWidth && y >= 0 && y < imageHeight;
// }
