#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "include/Auxiliary.h"

// Default camera field of view of tello drone
const float DEFAULT_TELLO_FOV = 82.6;

void checkVisiblePoints(cv::Point3f cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree) {
    // Generate many 3D points
    std::vector<cv::Point3f> points;
    for (int x = -100; x <= 100; x++) {
        for (int y = -100; y <= 100; y++) {
            for (int z = 0; z <= 100; z++) {
                points.push_back(cv::Point3f(x, y, z));
            }
        }
    }

    // Plot visible points
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    for (cv::Point3f point : points) {
        if (Auxiliary::isPointVisible(point, cameraPos, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, yaw_degree, pitch_degree)) {
            cv::circle(image, cv::Point2f(point.x + 10, -point.y + 10), 2, cv::Scalar(0, 255, 0), -1);
        }
    }
    cv::imshow("Visible Points", image);
    cv::waitKey(0);
}

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
    // Between -90 to 90, pitch
    double bottom_to_up_degree = data["bottomToUpDegree"];
    // between -180 to 180, roll
    double roll_degree = data["rollDegree"]

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

    checkVisiblePoints(camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, left_to_right_degree, bottom_to_up_degree);
}
