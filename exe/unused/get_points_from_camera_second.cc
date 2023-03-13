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
    double left_to_right_degree = data["yawDegree"];
    // Between -180 to 180, pitch
    double bottom_to_up_degree = data["pitchDegree"];
    // between -180 to 180, roll
    double roll_degree = data["rollDegree"];

    double roll_rad = roll_degree * CV_PI / 180.0;
    double pitch_rad = bottom_to_up_degree * CV_PI / 180.0;
    double yaw_rad = left_to_right_degree * CV_PI / 180.0;

    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];

    std::vector<cv::Point3f> points;

    std::fstream pointData;
    pointData.open(data["getPointDataCsv"], std::ios::in);

    std::vector<std::string> row;
    std::string line, word, temp;

    while (!pointData.eof()) {
        row.clear();
        
        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            row.push_back(word);
        }
        
        points.push_back(cv::Point3f(std::stod(row[0]), std::stod(row[1]), std::stod(row[2])));
    }
    pointData.close();

    std::vector<cv::Point3f> seen_points;

    seen_points = Auxiliary::FilterPointsInView(points, camera_position, cv::Vec3f(yaw_rad, pitch_rad, roll_rad), cv::Vec3f(fx, cy*2, cx*2));

    for(cv::Point3f  point : seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
