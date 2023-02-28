#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "System.h"

#include "include/Auxiliary.h"

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];

    // Check settings file
    cv::FileStorage fsSettings(droneYamlPathSlam, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << droneYamlPathSlam << std::endl;
       exit(-1);
    }

    std::string map_input_dir = data["mapInputDir"];
    std::string vocPath = data["VocabularyPath"];
    
    // Extract the camera position
    double x = data["cameraPosX"];
    double y = data["cameraPosY"];
    double z = data["cameraPosZ"];

    cv::Point3d camera_position(x, y, z);

    double yaw_rad = data["leftToRightDegree"];
    double pitch_rad = data["bottomToUpDegree"];
    double roll_rad = data["rollDegree"];

    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    double minX = 3.7;
    double maxX = width;
    double minY = 3.7;
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

    ORB_SLAM2::System system(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, map_input_dir + "simulatorMap.bin", true, false);

    std::vector<cv::Point3d> seen_points;

    for(auto point : system.GetMap()->GetAllMapPoints())
    {
        if(point)
        {
            auto point_pos = point->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point_pos);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();


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
            const double minDistance = point->GetMinDistanceInvariance();
            const double maxDistance = point->GetMaxDistanceInvariance();
            const cv::Mat PO = worldPos-mOw;
            const double dist = cv::norm(PO);

            if(dist<minDistance || dist>maxDistance)
                continue;

            // Check viewing angle
            cv::Mat Pn = point->GetNormal();
            Pn.convertTo(Pn, CV_64F);

            const float viewCos = PO.dot(Pn)/dist;

            if(viewCos<0.5)
                continue;

            seen_points.push_back(cv::Point3d(worldPos.at<double>(0), worldPos.at<double>(1), worldPos.at<double>(2)));
        }
    }
    
    for(cv::Point3d point: seen_points)
    {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }
    std::cout << "total: " << seen_points.size() << std::endl;

    return 0;
}
