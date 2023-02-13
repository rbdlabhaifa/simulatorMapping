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

std::vector<cv::Point3f> Auxiliary::FilterPointsInView(std::vector<cv::Point3f> points, cv::Point3f cam_pos, cv::Vec3f cam_angle, cv::Vec3f focal)
{
    // Extract the rotations from the rotation matrix
    float Yaw = cam_angle[0];
    float Pitch = cam_angle[1];
    float Roll = cam_angle[2];

    // Calculate the rotation matrices

    // The rotation on the YZ-plane is the pitch
    float cp = cos(Pitch);
    float sp = sin(Pitch);
    cv::Mat Rx = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0,
                   0, cp, -sp, 0,
                   0, sp, cp, 0,
                   0, 0, 0, 1);
    
    // The rotation on the XZ-plane is the yaw
    float cy = cos(-Yaw);
    float sy = sin(-Yaw);
    cv::Mat Ry = (cv::Mat_<float>(4, 4) << cy, 0, sy, 0,
                   0, 1, 0, 0,
                   -sy, 0, cy, 0,
                   0, 0, 0, 1);
    
    // The rotation on the XY-plane is the roll
    float cr = cos(Roll);
    float sr = sin(Roll);
    cv::Mat Rz = (cv::Mat_<float>(4, 4) << cr, -sr, 0, 0,
                   sr, cr, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1);

    // Matrix to represent the change to cameras axises
    float Cx = cam_pos.x;
    float Cy = cam_pos.y;
    float Cz = cam_pos.z;
    cv::Mat Tc = (cv::Mat_<float>(4, 4) << 1, 0, 0, -Cx,
                   0, 1, 0, -Cy,
                   0, 0, 1, -Cz,
                   0, 0, 0, 1);
    
    // Calculate the extrinsic transformation
    cv::Mat Rt = Rz * Rx * Ry * Tc;

    // Empty set to hold the points that in the view range
    std::vector<cv::Point3f> SeenPoints;

    // Calculate the lengths seen on the picture frame
    float f_depth = focal[0];
    float f_height = focal[1];
    float f_width = focal[2];
    float vt = f_height / f_depth / 2;
    float ht = f_width  / f_depth / 2;

    // Iterate on the points, and deside which point is in the field of view
    for (cv::Point3f point : points)
    {
        // Extract point coordinates
        float Pwx = point.x;
        float Pwy = point.y;
        float Pwz = point.z;

        // Create homogeneous vector for the point 
        cv::Mat Pw = (cv::Mat_<float>(4, 1) << Pwx, Pwy, Pwz, 1);

        // Calculate the position relative to the camera
        cv::Mat Pc = Rt * Pw;

        // If the point have negative side of z-axis, its behind the camera
        if (Pc.at<float>(2, 0) <= 0) {
            continue;
        }

        // Check if horizantlly, relative to the camera, the point inside the FOV
        if (abs(Pc.at<float>(0, 0) / Pc.at<float>(2, 0)) > vt) {
            continue;
        }

        // Check if vertically, relative to the camera, the point inside the FOV
        if (abs(Pc.at<float>(1, 0) / Pc.at<float>(2, 0)) > ht) {
            continue;
        }

        // All the checks passed, the point is seen by the camera.
        // Append the point to the seen points
        SeenPoints.push_back(cv::Point3f(Pwx, Pwy, Pwz));
    }
    
    return SeenPoints;
}