//
// Created by tzuk on 6/4/23.
//
#include "include/Auxiliary.h"
#include "simulator/simulator.h"
#include "navigation/RoomExit.h"

int main(int argc, char **argv)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    std::string vocabulary_path = data["VocabularyPath"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string simulatorOutputDir = data["simulatorOutputDir"];
    std::string mapInputDir = data["mapInputDir"];
    bool trackImages = data["trackImages"];
    double movementFactor = data["movementFactor"];
    double simulatorStartingSpeed = data["simulatorStartingSpeed"];

    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, simulatorOutputDir, false,
                        mapInputDir, movementFactor, simulatorStartingSpeed, vocabulary_path);

    auto simulatorThread = simulator.run();

    while (!simulator.isReady())
    { // wait for the 3D model to load
        ORB_SLAM2::System::systemUsleep(1000);
    }

    std::cout << "to stop press k" << std::endl;
    std::cout << "to stop tracking press t" << std::endl;
    std::cout << "to save map point press m" << std::endl;
    std::cout << "to start scanning press tab" << std::endl;

    while (!simulator.startScanning())
    { // wait for the 3D model to load
        ORB_SLAM2::System::systemUsleep(10);
    }

    simulator.setTrack(true);
    int currentYaw = 0;
    int angle = 10;
    std::string commands[] = {"forward 0.5", "right 0.7", "left 0.7", "back 0.5", "cw " + std::to_string(angle)};
    for (int i = 0; i < std::ceil(360 / angle); i++)
    {
        for(const auto &command : commands) {
            simulator.command(command);
        }
        sleep(1);
    }
    sleep(2);
    auto scanMap = simulator.getCurrentMap();

    std::vector<Eigen::Vector3d> eigenData;
    for (auto &mp: scanMap) {
        if (mp != nullptr && !mp->isBad()) {
            auto vector = ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());
            eigenData.emplace_back(vector);
        }
    }

    RoomExit roomExit(eigenData);
    auto exitPoints = roomExit.getExitPoints();
    std::sort(exitPoints.begin(), exitPoints.end(), [&](auto &p1, auto &p2) {
        return p1.first < p2.first;
    });
    cv::Mat cvCurrentLocation = simulator.getCurrentLocation().rowRange(0, 2).col(3);
    Eigen::Vector3d currentLocation = ORB_SLAM2::Converter::toVector3d(cvCurrentLocation);

    double currentAngle = std::atan2(currentLocation.z(),currentLocation.x());
    double targetAngle = std::atan2(exitPoints.front().second.z(),exitPoints.front().second.x());
    int angle_difference = targetAngle;

    std::string rotCommand;
    if (angle_difference<0) {
        rotCommand = "ccw " + std::to_string(std::abs(angle_difference));

    } else {
        rotCommand = "cw "+std::to_string(angle_difference);
    }

    std::cout << rotCommand << std::endl;
    simulator.command(rotCommand);
    double distanceToTarget = (currentLocation - exitPoints.front().second).norm();
    std::string forwardCommand = "forward " + std::to_string(3 * int(distanceToTarget));
    std::cout << forwardCommand << std::endl;
    simulator.command(forwardCommand);
    simulatorThread.join();
}
