//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include <cmath>
#include <vector>
#include <limits>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <filesystem>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <pangolin/pangolin.h>
#include <eigen3/Eigen/Geometry>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

class Auxiliary {
public:
    static std::string GetGeneralSettingsPath();

    static bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

    static void getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

    static std::vector<cv::Point3f> FilterPointsInView(std::vector<cv::Point3f> points, cv::Point3f cam_pos, cv::Vec3f cam_angle, cv::Vec3f focal);

    static std::vector<cv::Point3d> getPointsFromPos(const std::string cloud_points, const cv::Point3d camera_position, double yaw, double pitch, double roll, cv::Mat &Twc);

    static std::vector<cv::Point3d> getPointsFromTcw(const std::string cloud_points, const pangolin::OpenGlMatrix &Tcw, pangolin::OpenGlMatrix &Twc);

    static std::vector<std::string> GetAllFrameDatas();

    static std::vector<std::string> GetFrameDatas(double amount=1); // Between 0 to 1

    static void add_unique_points(std::vector<cv::Point3d>& target, const std::vector<cv::Point3d>& source);
};
#endif // ORB_SLAM2_AUXILIARY_H
