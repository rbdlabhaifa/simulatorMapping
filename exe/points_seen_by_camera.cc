#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
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

    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    double minX = 0;
    double maxX = width;
    double minY = 0;
    double maxY = height;

    Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
    Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Tcw_eigen.block<3, 1>(0, 3) << camera_position.x, camera_position.y, camera_position.z;

    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw.at<double>(i,j) = Tcw_eigen(i,j);
        }
    }

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
            try 
            {
                std::stod(word);
            } 
            catch(std::out_of_range)
            {
                word = "0";
            }
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

        // Project in image and check it is not outside
        const double invz = 1.0f/PcZ;
        const double u=fx*PcX*invz+cx;
        const double v=fy*PcY*invz+cy;

        if(u<minX || u>maxX)
            continue;
        if(v<minY || v>maxY)
            continue;

        // Check distance is in the scale invariance region of the MapPoint
        const double minDistance = point[3];
        const double maxDistance = point[4];
        const cv::Mat PO = worldPos-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point[5];
        Pn.at<double>(1) = point[6];
        Pn.at<double>(2) = point[7];

        std::cout << "P: " << worldPos << ", Pn: " << Pn << std::endl;

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5)
            continue;

        seen_points.push_back(cv::Point3d(point[0], point[1], point[2]));
    }
    
    for(cv::Point3d point: seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }
    std::cout << "total: " << seen_points.size() << std::endl;

    return 0;
}
