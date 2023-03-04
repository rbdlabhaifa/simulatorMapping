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
    float k1 = fsSettings["Camera.k1"];
    float k2 = fsSettings["Camera.k2"];
    float k3 = fsSettings["Camera.k3"];
    float p1 = fsSettings["Camera.p1"];
    float p2 = fsSettings["Camera.p2"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

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
    float x = stof(row[1]);
    float y = stof(row[2]);
    float z = stof(row[3]);

    cv::Point3f camera_position(x, y, z);

    float yaw_rad = stof(row[4]);
    float pitch_rad = stof(row[5]);
    float roll_rad = stof(row[6]);
    
    // Convert the Euler angles to degrees
    float yaw_degree = yaw_rad * 180.0 / CV_PI;
    float pitch_degree = pitch_rad * 180.0 / CV_PI;
    float roll_degree = roll_rad * 180.0 / CV_PI;

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

    Auxiliary::getPoints(data["getPointDataCsv"], &seen_points, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, yaw_degree, pitch_degree);

    for(cv::Point3f  point : seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    std::cout << seen_points.size() << std::endl;

    return 0;
}
