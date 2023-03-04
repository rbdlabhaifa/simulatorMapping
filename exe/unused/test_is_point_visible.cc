#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sstream>

#include "Python.h"

#include "include/Auxiliary.h"

int main(void)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Check settings file
    cv::FileStorage fsSettings(data["DroneYamlPathSlam"], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << data["DroneYamlPathSlam"] << std::endl;
       exit(-1);
    }

    const cv::Point3f camera_position(data["cameraPosX"], data["cameraPosY"], data["cameraPosZ"]);

    // Between -180 to 180, yaw
    double left_to_right_degree = data["leftToRightDegree"];
    // Between -180 to 180, pitch
    double bottom_to_up_degree = data["bottomToUpDegree"];
    // between -180 to 180, roll
    double roll_degree = data["rollDegree"];

    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];
    float k1 = fsSettings["Camera.k1"];
    float k2 = fsSettings["Camera.k2"];
    float k3 = fsSettings["Camera.k3"];
    float p1 = fsSettings["Camera.p1"];
    float p2 = fsSettings["Camera.p2"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    std::vector<cv::Point3f> points;
    Auxiliary::getPoints(data["getPointDataCsv"], &points, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, left_to_right_degree, bottom_to_up_degree);

    for(cv::Point3f  point : points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    std::stringstream command, command2;
    Py_Initialize();
    command << "ax.scatter([";
    for(cv::Point3f  point : points)
    {
        command << point.x << ", ";
    }
    command << "0.0], [";
    for(cv::Point3f  point : points)
    {
        command << point.y << ", ";
    }
    command << "0.0], [";
    for(cv::Point3f  point : points)
    {
        command << point.z << ", ";
    }
    command << "0.0], c='r', marker='o')";

    command2 << "ax.scatter(" << camera_position.x << ", " << camera_position.y << ", " << camera_position.z << ", c='b', marker='o')";
    PyRun_SimpleString("import matplotlib.pyplot as plt");
    PyRun_SimpleString("fig = plt.figure()");
    PyRun_SimpleString("ax = fig.add_subplot(111, projection='3d')");
    PyRun_SimpleString(command.str().c_str());
    PyRun_SimpleString("ax.scatter(0, 0, 0, c='g', marker='o')");
    PyRun_SimpleString(command2.str().c_str());
    PyRun_SimpleString("ax.set_xlabel('X Label')");
    PyRun_SimpleString("ax.set_xlabel('Y Label')");
    PyRun_SimpleString("ax.set_xlabel('Z Label')");
    PyRun_SimpleString("plt.show()");
    Py_Exit(0);
    return 0;}
