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
    double yaw_degree = data["yawDegree"];
    // Between -180 to 180, pitch
    double pitch_degree = data["pitchDegree"];
    // between -180 to 180, roll
    double roll_degree = data["rollDegree"];

    double yaw = yaw_degree * CV_PI / 180.0;
    double pitch = pitch_degree * CV_PI / 180.0;
    double roll = roll_degree * CV_PI / 180.0;

    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];

    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1.0;


    cv::Mat R = (cv::Mat_<float>(3, 3) <<
             cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch),
             sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw), 
             sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), 
             sin(roll)*cos(pitch), 
             cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw), 
             cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw), 
             cos(roll)*cos(pitch));

    // Create translation vector from camera position
    cv::Mat t = (cv::Mat_<float>(3, 1) << camera_position.x, camera_position.y, camera_position.z);

    // Create camera matrix
    cv::Mat P = K * (cv::Mat_<float>(3, 4) << R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), t.at<float>(0,0),
                                            R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), t.at<float>(1,0),
                                            R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), t.at<float>(2,0));



    std::vector<cv::Point3f> seen_points;

    seen_points = Auxiliary::FilterPointsInView(points, camera_position, cv::Vec3f(yaw_rad, pitch_rad, roll_rad), cv::Vec3f(fx, cy*2, cx*2));

    for(cv::Point3f  point : seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
