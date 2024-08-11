#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ostream>
#include <fstream>
#include <iostream>
#include <matplotlibcpp.h>
#include "RoomExit/RoomExit.h"

int main(int argc, char **argv) {
    std::string fileName = argv[1];
    std::string s;
    std::ifstream data;
    data.open(fileName);
    std::vector<Eigen::Vector3d> points;
    std::vector<double> xs;
    std::vector<double> ys;
    while (std::getline(data, s)) {
        std::stringstream lineStream(s);
        double x;
        double y;
        double z;
        lineStream >> x;
        if (lineStream.peek() == ',')
            lineStream.ignore();
        lineStream >> y;
        if (lineStream.peek() == ',')
            lineStream.ignore();
        lineStream >> z;
        if (lineStream.peek() == ',')
            lineStream.ignore();
        points.emplace_back(x, y, z);
        xs.emplace_back(x);
        ys.emplace_back(z);
    }
    RoomExit roomExit(points);
    auto exitPoints = roomExit.getExitPoints();
    std::vector<double> xExit;
    std::vector<double> yExit;
    for (auto &point:exitPoints) {
        xExit.emplace_back(point.second.x());
        yExit.emplace_back(point.second.z());
    }
    matplotlibcpp::scatter(xs, ys, 2.0);
    matplotlibcpp::plot(xExit, yExit, "ro");
    matplotlibcpp::show();

    return 0;
}