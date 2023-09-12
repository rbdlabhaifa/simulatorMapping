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
    std::string loadMapPath = data["loadMapPath"];
    bool loadMap = data["loadMap"];
    bool trackImages = data["trackImages"];
    double movementFactor = data["movementFactor"];
    double simulatorStartingSpeed = data["simulatorStartingSpeed"];
    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, true, map_input_dir, loadMap,
                        loadMapPath, movementFactor,VocabularyPath, simulatorStartingSpeed);
    auto simulatorThread = simulator.run();
    while (!simulator.isReady()) { // wait for the 3D model to load
        usleep(1000);
    }

    std::cout << "to stop press k" << std::endl;
    std::cout << "to stop tracking press t" << std::endl;
    std::cout << "to save map point press m" << std::endl;
    std::cout << "waiting for key press to start scanning " << std::endl << std::endl;
//    std::cin.get();
    simulator.setTrack(true);

    simulator.Initialization();

    int currentYaw = 0;
    int angle = 5;

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
        auto current_pose = simulator.getCurrentLocation();
        if (!current_pose.empty()) {
            auto aligned_pose = simulator.align(current_pose);
//            std::cout << "---------------------" << std::endl;
//            std::cout << "aligned pose " << aligned_pose << std::endl;
//            std::cout << "---------------------" << std::endl;
            auto Rwc = simulator.rotation_matrix_from_pose(aligned_pose);
//            std::cout << "---------------------" << std::endl;
//            std::cout << "Rwc " << Rwc << std::endl;
//            std::cout << "---------------------" << std::endl;
            auto angles = simulator.rotation_matrix_to_euler_angles(Rwc);
//            std::cout << "---------------------" << std::endl;
//            std::cout << "angles " << angles << std::endl;
//            std::cout << "---------------------" << std::endl;
            auto currentAngle = angles.z + 90;
//            auto currentLoc = ORB_SLAM2::Converter::toVector3d(currentLocation.rowRange(0, 2).col(3));
//            double currentAngle = std::atan2(currentLoc.z(), currentLoc.x());
            std::cout << "Current Angle " << currentAngle << std::endl;
            std::cout << "Current Angle function " << simulator.ExtractYaw() << std::endl;
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

    std::vector<Eigen::Vector3f> exitPointsFloat;
    for (auto point: exitPoints){
        cv::Point3f cvPoint = cv::Point3f (float(point.second[0]), float(point.second[1]), float(point.second[2]));
        Eigen::Vector3f eigenPoint(float(point.second[0]), float(point.second[1]), float(point.second[2]));
        std::cout << cvPoint << std::endl;
        simulator.getSLAM()->GetMapDrawer()->navigationPoints.emplace_back(cvPoint);
        exitPointsFloat.emplace_back(eigenPoint);
        std::cout << simulator.getSLAM()->GetMapDrawer()->navigationPoints.size() << std::endl;
    }

    auto nav_point = exitPointsFloat.front();
//    cv::Mat cv_nav_point = (cv::Mat_<float>(3,1) << nav_point.x(), nav_point.y(), nav_point.z()) ;
    cv::Mat cv_nav_point = (cv::Mat_<float>(3,1) << eigenData[300].x(), eigenData[300].y(), eigenData[300].z());

    cv::Mat aligned_nav_point = simulator.R_align * (cv_nav_point - simulator.mu_align);
    std::cout << "Aligned point " << aligned_nav_point << std::endl;
    Eigen::Vector3f eigen_nav_point(aligned_nav_point.at<float>(0, 0), aligned_nav_point.at<float>(1, 0), aligned_nav_point.at<float>(2, 0));

//    simulator.getSLAM()->GetMapDrawer()->current_navigtion_point = cv::Point3f (nav_point.x(), nav_point.y(), nav_point.z());
    simulator.getSLAM()->GetMapDrawer()->current_navigtion_point = cv::Point3f (eigenData[300].x(), eigenData[300].y(), eigenData[300].z());

    simulator.navigateToPoint(eigen_nav_point);
    simulatorThread.join();
}
