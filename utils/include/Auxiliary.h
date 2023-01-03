//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include "Point.h"
#include "Line.h"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <matplotlibcpp.h>
#include <limits>
#include <pangolin/pangolin.h>

#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

class Auxiliary {
public:
    static std::string GetGeneralSettingsPath();

    static double det(const Point &point1, const Point &point2);

};
#endif //ORB_SLAM2_AUXILIARY_H
