

#include <iostream>
#include "AutonomousDrone.h"

AutonomousDrone::AutonomousDrone(Simulator *sim) {
    this->simulator = sim;
}

void AutonomousDrone::run(){
    //Doing scan
    simulator->setTrack(true);
    int currentYaw = 0;
    int angle = 5;
    cv::Mat runTimeCurrentLocation;

    //Initialize map
    std::string command = "left " + std::to_string(simulator->amountLeftRight);
    simulator->command(command);
    command = "right " + std::to_string(simulator->amountLeftRight);
    simulator->command(command);
//    command  = "up " + std::to_string(simulator->amountUpDown);
//    simulator->command(command);
//    command = "down " + std::to_string(simulator->amountUpDown);
//    simulator->command(command);
    sleep(1);

    for (int i = 0; i < std::ceil(360 / angle); i++) {
        std::string c = "cw " + std::to_string(simulator->amountYaw);
        simulator->command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        c = "right 0.3";
        simulator->command(c);
        c = "left 0.3";
        simulator->command(c);
//        //runTimeCurrentLocation = simulator.getCurrentLocation();
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        sleep(1);
    }

    //Loading current map

    //simulator.setTrack(false);
    sleep(2);
    auto scanMap = simulator->getCurrentMap();
    std::vector<Eigen::Vector3d> eigenData;
    for (auto &mp: scanMap) {
        if (mp != nullptr && !mp->isBad()) {
            auto vector = ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());
            eigenData.emplace_back(vector);
        }
    }
/*
    //Algorithm for moving to next point to scan
    RoomExit roomExit(eigenData);
    auto exitPoints = roomExit.getExitPoints();
    // sorting interest points
    std::sort(exitPoints.begin(), exitPoints.end(), [&](auto &p1, auto &p2) {
        return p1.first < p2.first;
    });

    //Preprocessing for navigation to interest point
    auto currentLocation = ORB_SLAM2::Converter::toVector3d(simulator->getCurrentLocation().rowRange(0, 2).col(3));

    double currentAngle = std::atan2(currentLocation.z(),currentLocation.x());
    double targetAngle = std::atan2(exitPoints.front().second.z(),exitPoints.front().second.x());
    int angle_difference = targetAngle;

    std::string rotCommand;
    if (angle_difference<0){
        rotCommand = "ccw " + std::to_string(std::abs(angle_difference));

    }else{
        rotCommand = "cw "+std::to_string(angle_difference);
    }
    std::cout << rotCommand << std::endl;
    simulator->command(rotCommand);

    double distanceToTarget = (currentLocation-exitPoints.front().second).norm();
    std::string forwardCommand = "forward " + std::to_string( 3*int(distanceToTarget));
    std::cout << forwardCommand << std::endl;
    simulator->command(forwardCommand);*/
}