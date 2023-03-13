#include <math.h>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "include/Auxiliary.h"

int main()
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

    const cv::Point3f camera_position(data["startingCameraPosX"], data["startingCameraPosY"], data["startingCameraPosZ"]);

    // Between -180 to 180, yaw
    double yaw = data["yawDegree"];
    // Between -180 to 180, pitch
    double pitch = data["pitchDegree"];
    // between -180 to 180, roll
    double roll = data["rollDegree"];

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
    Auxiliary::getPoints(data["getPointDataCsv"], &points, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll, yaw, pitch);

    for(cv::Point3f  point : points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
