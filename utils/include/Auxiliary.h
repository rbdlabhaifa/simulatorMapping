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
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <pangolin/pangolin.h>
#include <Eigen/Geometry>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

//#include "Point.h"
//#include "Line.h"
#include <matplotlibcpp.h>

class Auxiliary {
public:
    static std::string GetGeneralSettingsPath();

    static std::string GetDemoSettingsPath();

    static bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

    static void getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

    static std::vector<cv::Point3f> FilterPointsInView(std::vector<cv::Point3f> points, cv::Point3f cam_pos, cv::Vec3f cam_angle, cv::Vec3f focal);

    static std::vector<cv::Point3d> getPointsFromPos(const std::string cloud_points, const cv::Point3d camera_position, double yaw, double pitch, double roll, cv::Mat &Twc);

    static std::vector<cv::Point3d> getPointsFromTcw(const std::string cloud_points, const pangolin::OpenGlMatrix &Tcw, pangolin::OpenGlMatrix &Twc);

    static std::vector<std::string> GetAllFrameDatas();

    static std::vector<std::string> GetFrameDatas(double amount=1); // Between 0 to 1

    static void add_unique_points(std::vector<cv::Point3d>& target, const std::vector<cv::Point3d>& source);

//    static void showCloudPoint(const std::vector<Point> &redPoints, const std::vector<Point> &cloud);
//
//    static double calculateDistanceXY(const Point &point1, const Point &point2);
//
//    static double getDistanceToClosestSegment(const Point &point, const std::vector<Line> &segments);
//
//    static double getAngleFromSlope(double slope);
//
//    static double distanceBetweenPointAndSegment(const Point &point, Line segment);
//
//    static double getAngleBySlopes(Line &line1, Line &line2);
//
//    static double radiansToAngle(double radian);
//
//    static std::vector<double> getXValues(const std::vector<Point> &points);
//    static std::vector<double> getYValues(const std::vector<Point> &points);
//    static std::vector<double> getZValues(const std::vector<Point> &points);
//
//    //static double det(const Point &point1, const Point &point2);
//
//    static double angleToRadians(int angle);
};
#endif // ORB_SLAM2_AUXILIARY_H
