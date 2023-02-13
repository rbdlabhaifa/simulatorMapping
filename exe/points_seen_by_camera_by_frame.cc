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
    
    cv::Mat twc = (cv::Mat_<double>(1, 3) << stod(row[1]), stod(row[3]), stod(row[2]));
    cv::Mat rwc = (cv::Mat_<double>(3, 3) << stod(row[4]), stod(row[5]), stod(row[6]),
                                            stod(row[7]), stod(row[8]), stod(row[9]),
                                            stod(row[10]), stod(row[11]), stod(row[12]));

    cv::Matx33d r;
    cv::Matx33d eulerAngles;

    cv::RQDecomp3x3(rwc, r, eulerAngles);

    // Extract the camera position
    double x = twc.at<double>(0);
    double y = twc.at<double>(1);
    double z = twc.at<double>(2);

    double roll_rad = std::atan2(eulerAngles(2, 1), eulerAngles(2, 2));
    double pitch_rad = std::asin(-eulerAngles(2, 0));
    double yaw_rad = std::atan2(eulerAngles(1, 0), eulerAngles(0, 0));
    
    // Convert the Euler angles to degrees
    double roll_degree = roll_rad * 180.0 / CV_PI;
    double pitch_degree = pitch_rad * 180.0 / CV_PI;
    double yaw_degree = yaw_rad * 180.0 / CV_PI;

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

    return 0;
}
