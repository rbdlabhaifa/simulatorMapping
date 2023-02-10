#include <math.h>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "include/Auxiliary.h"

// Default camera field of view of tello drone
const float DEFAULT_TELLO_FOV = 82.6;

void getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree) {
    std::fstream pointData;
    pointData.open(csvPath, std::ios::in);

    std::vector<std::string> row;
    std::string line, word, temp;

    while (!pointData.eof()) {
        cv::Point3f pointToCompare;

        row.clear();
        
        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            row.push_back(word);
        }
        
        pointToCompare = cv::Point3f(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
        if (Auxiliary::isPointVisible(pointToCompare, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, yaw_degree, pitch_degree)) {
            (*points).push_back(pointToCompare);
        }
    }
    pointData.close();
}

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

    const cv::Point3f camera_position(data["cameraPosX"], data["cameraPosY"], data["cameraPosZ"]);

    // Between -180 to 180, yaw
    double left_to_right_degree = data["leftToRightDegree"];
    // Between -180 to 180, pitch
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

    std::vector<cv::Point3f> points;
    getPoints(data["getPointDataCsv"], &points, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, left_to_right_degree, bottom_to_up_degree);

    for(cv::Point3f  point : points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
