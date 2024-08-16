//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include <cmath>
#include <vector>
#include <limits>
#include <fstream>
#include <csignal>
#include <Eigen/Core>
#include <filesystem>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>


class Auxiliary
{
public:
    static std::string GetGeneralSettingsPath();

    static void drawPoints(std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>> &pointsToDraw);
};
#endif // ORB_SLAM2_AUXILIARY_H
