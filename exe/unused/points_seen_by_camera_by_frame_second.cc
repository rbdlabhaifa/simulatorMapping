#include <math.h>
#include <string>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

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

    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];

    std::vector<cv::Point3f> points;

    int frame_to_check = data["frameToCheck"];
    std::string map_inpur_dir = data["mapInputDir"];
    std::ifstream pointData;
    pointData.open(map_inpur_dir + "frameData" + std::to_string(frame_to_check) + ".csv");

    std::vector<std::string> row;
    std::string line, word, temp;
    
    std::getline(pointData, line);

    std::stringstream words(line);

    while (std::getline(words, word, ',')) {
        row.push_back(word);
    }

    pointData.close();
    
    // Extract the camera position
    double x = stod(row[1]);
    double y = stod(row[2]);
    double z = stod(row[3]);

    double yaw_rad = stod(row[4]);
    double pitch_rad = stod(row[5]);
    double roll_rad = stod(row[6]);
    
    // Convert the Euler angles to degrees
    double yaw_degree = yaw_rad * 180.0 / CV_PI;
    double pitch_degree = pitch_rad * 180.0 / CV_PI;
    double roll_degree = roll_rad * 180.0 / CV_PI;

    pointData.open(map_inpur_dir + "cloud1.csv");

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

    seen_points = Auxiliary::FilterPointsInView(points, cv::Point3f(x, y, z), cv::Vec3f(yaw_rad, pitch_rad, roll_rad), cv::Vec3f(fx, cy*2, cx*2));

    for(cv::Point3f  point : seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    std::cout << seen_points.size() << std::endl;

    return 0;
}
