#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "System.h"
#include "Converter.h"
#include "include/Point.h"
#include "include/Auxiliary.h"
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
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations();
            for (auto &obs : observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;
                if (!currentFrame->image.empty())
                {
                    size_t pointIndex = obs.second;
                    cv::KeyPoint keyPoint = currentFrame->mvKeysUn[pointIndex];
                    cv::Point2f featurePoint = keyPoint.pt;
                    pointData << "," << currentFrame->mnId << "," << featurePoint.x << "," << featurePoint.y;
                    if (seen_frames.count(currentFrame->mnId) <= 0)
                    {
                        saveFrame(currentFrame->image, currentFrame->GetPose(), currentFrame->mnId, currentFrame->GetMapPoints().size());
                        seen_frames.insert(currentFrame->mnId);
                    }
                    // cv::Mat image = cv::imread(simulatorOutputDir + "frame_" + std::to_string(currentFrame->mnId) + ".png");
                    // cv::arrowedLine(image, featurePoint, cv::Point2f(featurePoint.x, featurePoint.y - 100), cv::Scalar(0, 0, 255), 2, 8, 0, 0.1);
                    // cv::imshow("image", image);
                    // cv::waitKey(0);
                }
            }
            pointData << std::endl;
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

int main(int argc, char** argv) {
    // signal(SIGINT, stopProgramHandler);
    // signal(SIGTERM, stopProgramHandler);
    // signal(SIGABRT, stopProgramHandler);
    // signal(SIGSEGV, stopProgramHandler);
    std::ifstream programData("C:/Users/tzuk9/Documents/simulatorMapping/generalSettings.json");
    nlohmann::json data;
    programData >> data;
    programData.close();

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
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true);
        std::cout << "here4" <<std::endl;
        cv::VideoCapture capture(videoPath);
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        int amount_of_frames = 1;
        capture >> frame;
        for (;;) {
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));

            capture >> frame;

            if (frame.empty()) {
                break;
            }
        }
        saveMap(0);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();
    

    if (isSavingMap) {
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}
