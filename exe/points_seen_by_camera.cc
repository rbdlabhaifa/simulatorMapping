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

    int frame_to_check = data["frameToCheck"];
    std::string map_input_dir = data["mapInputDir"];
    std::ifstream pointData;
    pointData.open(map_input_dir + "frameData" + std::to_string(frame_to_check) + ".csv");

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

    cv::Point3d camera_position(x, y, z);

    double yaw_rad = stod(row[4]);
    double pitch_rad = stod(row[5]);
    double roll_rad = stod(row[6]);
    
    // Convert the Euler angles to degrees
    double yaw_degree = yaw_rad * 180.0 / CV_PI;
    double pitch_degree = pitch_rad * 180.0 / CV_PI;
    double roll_degree = roll_rad * 180.0 / CV_PI;

    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    double k1 = fsSettings["Camera.k1"];
    double k2 = fsSettings["Camera.k2"];
    double k3 = fsSettings["Camera.k3"];
    double p1 = fsSettings["Camera.p1"];
    double p2 = fsSettings["Camera.p2"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    fsSettings["Camera.fx"] >> K.at<double>(0, 0);
    fsSettings["Camera.fy"] >> K.at<double>(1, 1);
    fsSettings["Camera.cx"] >> K.at<double>(0, 2);
    fsSettings["Camera.cy"] >> K.at<double>(1, 2);
    K.at<double>(2, 2) = 1;

    double FOV = 2 * atan(width / (2 * fx));

    // Calculate the horizontal and vertical angles of the field of view
    double HFOV = 2 * atan(tan(FOV / 2) * (width / height));
    double VFOV = FOV;

    // Calculate the pixel coordinates of the four corners of the image in the camera frame
    cv::Mat C1 = K.inv() * (cv::Mat_<double>(3, 1) << 0, 0, 1);
    cv::Mat C2 = K.inv() * (cv::Mat_<double>(3, 1) << (width-1), 0, 1);
    cv::Mat C3 = K.inv() * (cv::Mat_<double>(3, 1) << (width-1), (height-1), 1);
    cv::Mat C4 = K.inv() * (cv::Mat_<double>(3, 1) << 0, (height-1), 1);

    // Calculate the range of pixel coordinates that are likely to contain features
    double minX = std::min({C1.at<double>(0), C4.at<double>(0)});
    double maxX = std::max({C2.at<double>(0), C3.at<double>(0)});
    double minY = std::min({C1.at<double>(1), C2.at<double>(1)});
    double maxY = std::max({C3.at<double>(1), C4.at<double>(1)});

    // Construct the camera distortion parameters
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0) = k1;
    distCoeffs.at<double>(1) = k2;
    distCoeffs.at<double>(2) = p1;
    distCoeffs.at<double>(3) = p2;
    distCoeffs.at<double>(4) = k3;

    // Construct the rotation and translation matrices
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = cv::Mat(cv::Vec3d(camera_position.x, camera_position.y, camera_position.z));

    // Combine R and t into a single matrix
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);

    // Construct the full projection matrix P = K * [R|t]
    cv::Mat P = K * Rt;

    // Construct the 4x4 transformation matrix Tcw = [R|t;0 0 0 1]
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64F);
    P.copyTo(Tcw(cv::Rect(0, 0, 4, 3)));

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat mOw = -Rcw.t()*tcw;

    std::vector<cv::Vec<double, 8>> points;

    pointData.open(map_input_dir + "cloud1.csv", std::ios::in);

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
        
        points.push_back(cv::Vec<double, 8>(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7])));
    }
    pointData.close();

    std::vector<cv::Point3d> seen_points;

    for(cv::Vec<double, 8>  point : points)
    {
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point[0];
        worldPos.at<double>(1) = point[1];
        worldPos.at<double>(2) = point[2];

        const cv::Mat Pc = Rcw*worldPos+tcw;
        const double &PcX = Pc.at<double>(0);
        const double &PcY= Pc.at<double>(1);
        const double &PcZ = Pc.at<double>(2);

        // Check positive depth
        if(PcZ<0.0f)
            continue;

        std::cout << "A" << std::endl;
        // Project in image and check it is not outside
        const double invz = 1.0f/PcZ;
        const double u=fx*PcX*invz+cx;
        const double v=fy*PcY*invz+cy;

        if(u<minX || u>maxX)
            continue;
        std::cout << "B" << std::endl;
        if(v<minY || v>maxY)
            continue;
        std::cout << "C" << std::endl;

        // Check distance is in the scale invariance region of the MapPoint
        const double minDistance = point[3];
        const double maxDistance = point[4];
        const cv::Mat PO = P-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;
        std::cout << "D" << std::endl;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point[5];
        Pn.at<double>(1) = point[6];
        Pn.at<double>(2) = point[7];

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5)
            continue;
        std::cout << "E" << std::endl;

        seen_points.push_back(cv::Point3d(point[0], point[1], point[2]));
    }
    
    for(cv::Point3d point: seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }

    return 0;
}
