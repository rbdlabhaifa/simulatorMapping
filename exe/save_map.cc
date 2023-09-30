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

bool matCompare(cv::Mat& a, cv::Mat& b) {
    if (a.rows != b.rows || a.cols != b.cols || a.type() != b.type()) {
        return false;
    }
    for (int i = 0; i < a.rows; i++) {
        const void* a_row = a.ptr(i);
        const void* b_row = b.ptr(i);
        if (memcmp(a_row, b_row, a.cols*a.elemSize()) != 0) {
            return false;
        }
    }
    return true;
};

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
                keyPointsData << currentFrame->mnId << "," << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y <<
                              "," << currentKeyPoint.size << "," << currentKeyPoint.angle << "," <<
                              currentKeyPoint.response << "," << currentKeyPoint.octave << "," <<
                              currentKeyPoint.class_id << std::endl;

                // Save Descriptors
                cv::Mat current_descriptor = currentFrame->mDescriptors.row(obs.second);
                for (int j = 0; j < current_descriptor.rows; j++) {
                    descriptorData << static_cast<int>(current_descriptor.at<uchar>(j, 0));
                    for (int k = 1; k < current_descriptor.cols; k++) {
                        descriptorData << "," << static_cast<int>(current_descriptor.at<uchar>(j, k));
                    }
                    descriptorData << std::endl;
                }
            }
            keyPointsData.close();
            descriptorData.close();

            std::ofstream bestDescriptorData;
            bestDescriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_bestDescriptor.csv");
            // Save Descriptor
            cv::Mat best_descriptor = p->GetDescriptor();
            for (int j = 0; j < best_descriptor.rows; j++) {
                bestDescriptorData << static_cast<int>(best_descriptor.at<uchar>(j, 0));
                for (int k = 1; k < best_descriptor.cols; k++) {
                    bestDescriptorData << "," << static_cast<int>(best_descriptor.at<uchar>(j, k));
                }
                bestDescriptorData << std::endl;
            }
            bestDescriptorData.close();

            std::ofstream bestKeyPointData;
            bestKeyPointData.open(simulatorOutputDir + "point" + std::to_string(i) + "_bestKeyPoint.csv");
            for (auto obs : observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;
                cv::Mat current_descriptor = currentFrame->mDescriptors.row(obs.second);
                if (matCompare(best_descriptor, current_descriptor)) {
                    // Save Related Keypoint
                    cv::KeyPoint currentKeyPoint = currentFrame->mvKeys[obs.second];
                    bestKeyPointData << currentFrame->mnId << "," << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y <<
                                  "," << currentKeyPoint.size << "," << currentKeyPoint.angle << "," <<
                                  currentKeyPoint.response << "," << currentKeyPoint.octave << "," <<
                                  currentKeyPoint.class_id << std::endl;
                }
            }
            bestKeyPointData.close();

            pointData << std::endl;
            i++;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

int main() {
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
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true,
                                               simulatorOutputDirPath + "simulatorMap.bin",
                                               false);

    saveMap(0);

    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}
