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
#include "include/Point.h"
#include "include/Auxiliary.h"

/************* SIGNAL *************/
std::unique_ptr<ORB_SLAM2::System> SLAM;
std::string simulatorOutputDir;


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
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << std::endl;

            // Save Descriptor to another file
            cv::Mat current_descriptor = p->GetDescriptor();
            std::ofstream descriptorData;
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + ".csv");
            for (int j=0; j < current_descriptor.rows; j++) {
                descriptorData << std::to_string(current_descriptor.at<uchar>(j, 0));
                for (int k=1; k < current_descriptor.cols; k++) {
                    descriptorData << "," << std::to_string(current_descriptor.at<uchar>(j, k));
                }
                descriptorData << std::endl;
            }
            descriptorData.close();

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
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, true,
                                               simulatorOutputDirPath + "simulatorMap.bin",
                                               false);

    saveMap(0);

    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}
