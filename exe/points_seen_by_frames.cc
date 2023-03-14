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

    double amount = data["amount"];
    std::string map_input_dir = data["mapInputDir"];
    const std::string cloud_points = map_input_dir + "cloud1.csv";

    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;

    std::vector<cv::Point3d> seen_points;

    std::vector<std::string> frames_datas = Auxiliary::GetFrameDatas(amount);

    for(std::string frame : frames_datas)
    {
        row.clear();

        pointData.open(frame);
    
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

        cv::Point3d camera_position(x, y, z);

        double yaw = stod(row[4]);
        double pitch = stod(row[5]);
        double roll = stod(row[6]);

        cv::Mat Twc;

        std::vector<cv::Point3d> new_points = Auxiliary::getPointsFromPos(cloud_points, camera_position, yaw, pitch, roll, Twc);

        Auxiliary::add_unique_points(seen_points, new_points);
    }
    
    for(cv::Point3d point: seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }
    std::cout << "total: " << seen_points.size() << std::endl;

    return 0;
}
