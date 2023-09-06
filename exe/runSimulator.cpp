//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
#include "simulator/simulator.h"
#include "navigation/include/RoomExit.h"
#include "include/Auxiliary.h"


int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    std::string VocabularyPath = data["VocabularyPath"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string map_input_dir = data["mapInputDir"];
    bool trackImages = data["trackImages"];
    double movementFactor = data["movementFactor"];
    double simulatorStartingSpeed = data["simulatorStartingSpeed"];
    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, true, map_input_dir, false,
                        "", movementFactor,VocabularyPath, simulatorStartingSpeed);
    auto simulatorThread = simulator.run();
    while (!simulator.isReady()) { // wait for the 3D model to load
        usleep(1000);
    }



    std::cout << "to stop press k" << std::endl;
    std::cout << "to stop tracking press t" << std::endl;
    std::cout << "to save map point press m" << std::endl;
    std::cout << "waiting for key press to start scanning " << std::endl << std::endl;
    std::cin.get();
    simulator.setTrack(true);
    int currentYaw = 0;
    int angle = 7;

    cv::Mat runTimeCurrentLocation;
    for (int i = 0; i < std::ceil(360 / angle); i++) {
        std::string c = "left 0.3";
        simulator.command(c);

        //runTimeCurrentLocation = simulator.getCurrentLocation();
        c = "right 0.3";
        simulator.command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        c = "cw " + std::to_string(angle);
        simulator.command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        auto currentLocation = simulator.getCurrentLocation();
        if (!currentLocation.empty()) {
            auto currentLoc = ORB_SLAM2::Converter::toVector3d(currentLocation.rowRange(0, 2).col(3));
            double currentAngle = std::atan2(currentLoc.z(), currentLoc.x());
            std::cout << "Current Angle " << currentAngle << std::endl;
        }
        sleep(1);
        std::cout << "Finished " << 100 * angle * (i+1) / 360 << "% of scan" << std::endl;
    }
    std::cout << "Finished scan" << std::endl;
    //simulator.setTrack(false);
    sleep(2);

    std::cout << "Saving map after scan" << std::endl;
    simulator.SaveMap();

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


    for (auto point: exitPoints){
        cv::Point3d cvPoint = cv::Point3d (point.second[0], point.second[1], point.second[2]);
        std::cout << cvPoint << std::endl;
        simulator.getSLAM()->GetMapDrawer()->navigationPoints.emplace_back(cvPoint);
        std::cout << simulator.getSLAM()->GetMapDrawer()->navigationPoints.size() << std::endl;
    }

    simulator.navigateToPoint(exitPoints.front().second);
    simulatorThread.join();
}
