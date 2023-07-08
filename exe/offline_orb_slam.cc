#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "System.h"
#include "Converter.h"
#include "include/Auxiliary.h"

/************* SIGNAL *************/
std::unique_ptr<ORB_SLAM2::System> SLAM;
std::string simulatorOutputDir;


void saveFrame(cv::Mat &img, cv::Mat pose, int currentFrameId, int number_of_points) {
    if (img.empty()) 
    {
        std::cout << "Image is empty!!!" << std::endl;
        return;
    }
    std::ofstream frameData;
    frameData.open(simulatorOutputDir + "frameData" +
                   std::to_string(currentFrameId) + ".csv");

    std::ofstream framePointsCount;
    framePointsCount.open(simulatorOutputDir + "framePointsCount" +
                   std::to_string(currentFrameId) + ".txt");
    framePointsCount << number_of_points;
    framePointsCount.close();

    // Extract position from pose matrix
    double x = pose.at<float>(0,3);
    double y = pose.at<float>(1,3);
    double z = pose.at<float>(2,3);

    cv::Point3d camera_position(x, y, z);

    // Extract orientation from pose matrix
    double yaw, pitch, roll;
    yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0));
    pitch = atan2(-pose.at<float>(2,0), sqrt(pose.at<float>(2,1)*pose.at<float>(2,1) + pose.at<float>(2,2)*pose.at<float>(2,2)));
    roll = atan2(pose.at<float>(2,1), pose.at<float>(2,2));

    frameData << currentFrameId << ',' << camera_position.x << ',' << camera_position.y << ',' << camera_position.z << ','
              << yaw << ',' << pitch << ',' << roll << std::endl;
    cv::imwrite(simulatorOutputDir + "frame_" + std::to_string(currentFrameId) + ".png", img);
    frameData.close();
}

void saveMap(int mapNumber) {
    std::ofstream pointData;
    std::unordered_set<int> seen_frames;
    int i = 0;

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        if (p != nullptr && !p->isBad()) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth();
            cv::Mat Pn = p->GetNormal();
            Pn.convertTo(Pn, CV_64F);
            pointData << i << ",";
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);

            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations();
            std::ofstream keyPointsData;
            std::ofstream descriptorData;
            keyPointsData.open(simulatorOutputDir + "point" + std::to_string(i) + "_keypoints.csv");
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_descriptors.csv");
            for (auto obs : observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;

                // Save keyPoints
                cv::KeyPoint currentKeyPoint = currentFrame->mvKeys[obs.second];
                keyPointsData << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y << "," << currentKeyPoint.size <<
                              "," << currentKeyPoint.angle << "," << currentKeyPoint.response << "," <<
                              currentKeyPoint.octave << "," << currentKeyPoint.class_id << std::endl;

                // Save Descriptors
                cv::Mat current_descriptor = currentFrame->mDescriptors.row(obs.second);
                for (int j=0; j < current_descriptor.rows; j++) {
                    descriptorData << static_cast<int>(current_descriptor.at<uchar>(j, 0));
                    for (int k=1; k < current_descriptor.cols; k++) {
                        descriptorData << "," << static_cast<int>(current_descriptor.at<uchar>(j, k));
                    }
                    descriptorData << std::endl;
                }
            }
            keyPointsData.close();
            descriptorData.close();

            pointData << std::endl;
            i++;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count());
    SLAM->Shutdown();
    cvDestroyAllWindows();
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string simulatorOutputDirPath = data["simulatorOutputDir"];
    simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true);
    int amountOfAttepmpts = 0;
    while (amountOfAttepmpts++ < 1) {
        cv::VideoCapture capture(videoPath);
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 170; ++i) {
            capture >> frame;
        }
        int amount_of_frames = 1;

        for (;;) {
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));

            capture >> frame;

            if (frame.empty()) {
                break;
            }
        }
        saveMap(amountOfAttepmpts);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    if (isSavingMap) {
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}

