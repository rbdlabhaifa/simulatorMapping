#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "include/Auxiliary.h"

// Default camera field of view of tello drone
const float DEFAULT_TELLO_FOV = 82.6;

bool isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float yaw, float pitch,
                    int imageWidth, int imageHeight)
{
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    );
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F);

    // Calculate the rotation matrix from yaw and pitch angles
    cv::Mat rot = cv::Mat::eye(3, 3, CV_32F);
    rot.at<float>(0, 0) = cos(yaw) * cos(pitch);
    rot.at<float>(0, 1) = sin(yaw) * cos(pitch);
    rot.at<float>(0, 2) = -sin(pitch);
    rot.at<float>(1, 0) = -sin(yaw);
    rot.at<float>(1, 1) = cos(yaw);
    rot.at<float>(2, 0) = cos(yaw) * sin(pitch);
    rot.at<float>(2, 1) = sin(yaw) * sin(pitch);
    rot.at<float>(2, 2) = cos(pitch);

    // Convert camera position to a matrix
    cv::Mat cameraPosMat = cv::Mat(cameraPos);

    // Transform the point into the camera coordinate system
    cv::Mat pointCam = rot * (cv::Mat(point) - cameraPosMat);

    // Project the point into the image plane
    float x = cameraMatrix.at<float>(0, 0) * pointCam.at<float>(0, 0) / pointCam.at<float>(2, 0) +
              cameraMatrix.at<float>(0, 2);
    float y = cameraMatrix.at<float>(1, 1) * pointCam.at<float>(1, 0) / pointCam.at<float>(2, 0) +
              cameraMatrix.at<float>(1, 2);

    // Check if the point is within the bounds of the image
    return x >= 0 && x < imageWidth && y >= 0 && y < imageHeight;
}

void getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float cameraYaw, float cameraPitch, int width, int height) {
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
        if (isPointVisible(pointToCompare, camera_position, cameraYaw, cameraPitch, width, height)) {
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

    // Between -90 to 90, yaw
    double left_to_right_degree = data["leftToRightDegree"];
    // Between -90 to 90, pitch
    double bottom_to_up_degree = data["bottomToUpDegree"];

    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    std::vector<cv::Point3f> points;
    getPoints(data["getPointDataCsv"], &points, camera_position, left_to_right_degree, bottom_to_up_degree, width, height);

    for(cv::Point3f  point : points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
