#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

#include "include/Auxiliary.h"

#include "ORBextractor.h"

int main(void)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Check settings file
    cv::FileStorage fSettings(data["DroneYamlPathSlam"], cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << data["DroneYamlPathSlam"] << std::endl;
        exit(-1);
    }

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    cv::Mat img = cv::imread("/home/liam/dev/rbd/slamMaps/example_mapping11/frame_45.png");
    cv::cvtColor(img, img,  cv::COLOR_RGBA2GRAY);
    img.convertTo(img, CV_8UC1);
    ORB_SLAM2::ORBextractor* orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);

    // Draw keypoints on the image
    cv::Mat image_keypoints;
    cv::drawKeypoints(img, keypoints, image_keypoints);

    cv::imwrite("/home/liam/image_keypoints.png", image_keypoints);
    cv::waitKey(2);
    return 0;}
