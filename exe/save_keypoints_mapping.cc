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

void saveKeyPointsMapping() {
    std::ofstream pointData;
    int i = 0;

    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        if (p != nullptr && !p->isBad()) {
            pointData.open(simulatorOutputDir + "point" + std::to_string(i) + ".csv");

            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations();
            for (auto obs : observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;

                // Save Pose
                cv::Mat pose = currentFrame->GetPose();
                pointData << pose.at<float>(0, 0);
                for (int j = 0; j < pose.rows; j++) {
                    for (int k = 0; k < pose.cols; k++) {
                        if (j != 0  || k != 0)
                            pointData << "," << pose.at<float>(j, k);
                    }
                }
                pointData << std::endl;

                // Save keyPoints
                cv::KeyPoint currentKeyPoint = currentFrame->mvKeys[obs.second];
                pointData << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y <<
                              "," << currentKeyPoint.size << "," << currentKeyPoint.angle << "," <<
                              currentKeyPoint.response << "," << currentKeyPoint.octave << "," <<
                              currentKeyPoint.class_id << std::endl;

                // Save Descriptors
                cv::Mat current_descriptor = currentFrame->mDescriptors.row(obs.second);
                pointData << static_cast<int>(current_descriptor.at<uchar>(0, 0));
                for (int j = 0; j < current_descriptor.rows; j++) {
                    for (int k = 1; k < current_descriptor.cols; k++) {
                        if (j != 0  || k != 0)
                            pointData << "," << static_cast<int>(current_descriptor.at<uchar>(j, k));
                    }
                }
                pointData << std::endl;
            }

            pointData.close();
            i++;
        }
    }
    std::cout << "Saved all data" << std::endl;

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
    simulatorOutputDir = simulatorOutputDirPath + "keypointsMapping-" + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true,
                                               loadMapPath, false);

    saveKeyPointsMapping();

    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}
