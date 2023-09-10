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
    int angle = 5;

    cv::Mat runTimeCurrentLocation;
    simulator.Initialization();
    //auto [R_align, mu_align] = simulator.align_to_xy_axis();

    for (int i = 0; i < std::ceil(45 / angle); i++) {
        std::string c = "left 0.3";
        simulator.command(c);
        c = "right 0.3";
        simulator.command(c);
        c = "cw " + std::to_string(angle);
        simulator.command(c);

        auto currentLocation = simulator.getCurrentLocation();
        if (!currentLocation.empty()){
            auto align_pose = simulator.align_pose(currentLocation.clone());
            //TODO: use align_pose
            auto R = align_pose.rowRange(0, 3).colRange(0, 3).clone();
            R = R.t();
            //auto R = simulator.extract_rotation_matrix_from_pose(currentLocation);
            auto angles = simulator.rotation_matrix_to_euler_angles(R);
            std::cout << "yaw = " << angles.z << std::endl;
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
        cv::Point3f cvPoint = cv::Point3d (point.second[0], point.second[1], point.second[2]);
        simulator.getSLAM()->GetMapDrawer()->navigationPoints.emplace_back();
        std::cout << simulator.getSLAM()->GetMapDrawer()->navigationPoints.size() << std::endl;
    }

    auto nav_point = simulator.getSLAM()->GetMapDrawer()->navigationPoints[0];


    auto navigation_point = exitPoints.front().second;

//    cv::Point3f tmp(navigation_point.coeff(0, 0), navigation_point.coeff(1, 0), navigation_point.coeff(2, 0));
//    cv::Mat nav_point(tmp);

//    cv::Mat aligned_navigation_point = simulator.R_align * (nav_point - simulator.mu_align);

    std::cout << "We will navigate to " << navigation_point << std::endl;

    Eigen::Vector3f aligned_navigation_point_eigen(navigation_point.x(), navigation_point.y(), navigation_point.z());
    simulator.navigateToPoint(aligned_navigation_point_eigen);
    simulatorThread.join();
}
