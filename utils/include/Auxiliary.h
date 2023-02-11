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
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "Point.h"
#include "Line.h"

class Auxiliary {
public:
    static std::string GetGeneralSettingsPath();

    static double det(const Point &point1, const Point &point2);
    
    static bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

    static void getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree);

};
#endif //ORB_SLAM2_AUXILIARY_H
