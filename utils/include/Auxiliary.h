//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include <cmath>
#include <vector>
#include <limits>
#include <matplotlibcpp.h>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <pangolin/pangolin.h>
#include <opencv2/calib3d.hpp>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <image_geometry/pinhole_camera_model.h>

#include "Point.h"
#include "Line.h"

class Auxiliary {
public:
    static std::string GetGeneralSettingsPath();

    static double det(const Point &point1, const Point &point2);
    
    static bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree)
};
#endif //ORB_SLAM2_AUXILIARY_H
