//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Auxiliary.h"

double Auxiliary::det(const Point &point1, const Point &point2) {
    return point1.x * point2.y - point1.y * point2.x;
}

std::string Auxiliary::GetGeneralSettingsPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../generalSettings.json";
    return settingPath;
}

bool Auxiliary::isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree)
{
    // Define the position and orientation of the camera
    double roll_rad = roll_degree * CV_PI / 180.0;
    double pitch_rad = pitch_degree * CV_PI / 180.0;
    double yaw_rad = yaw_degree * CV_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << roll_rad, pitch_rad, yaw_rad);
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << cameraPos.x, cameraPos.y, cameraPos.z);

    cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) << fx, 0, cx,
                                                      0, fy, cy,
                                                      0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);

    // Project the 3D point onto the image plane
    std::vector<cv::Point3f> points = {point};
    std::vector<cv::Point2f> image_points;

    cv::projectPoints(points, R, tvec, camera_matrix, distCoeffs, image_points);

    // The image_point variable now contains the 2D projection of the 3D point on the image plane
    if (image_points[0].x >= 0 && image_points[0].x < width && image_points[0].y >= 0 && image_points[0].y < height)
    {
        return true;
    }

    return false;
}

void Auxiliary::getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree) {
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
