//
// Created by Dean on 9/30/23.
//

#include <matplotlibcpp.h>
#include "simulator/polySimulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"

#include <iostream>

#include "navigation/ExitRoomFiles/exit_room_algo.cpp"
#include "../tools/simulator/simulatorManager.h"



using std::vector;

int main(int argc, char **argv) {
    bool loadCustomMap = std::stoi(argv[1]);
    SimulatorManager &simManager = SimulatorManager::getInstance(loadCustomMap);

    PolySimulator* simPtr = simManager.getSimulatorPointer();
    std::cout << std::endl << "running " << argv[0] << " file" << std::endl;

    //This loop makes sure you want to activate this series of actions by letting you change the isSetPlace flag of the simulator to 'true' whenever you want to execute the code.
    while (!simManager.getSimulatorPointer()->isSetInPlace()){/*wait for key to enable*/}

    //The drone will follow the saved path.
    simManager.moveAccordingToSavedPath();
    simManager.getSimulatorPointer()->printCurrentLocation();

    vector<vector<double>> listOfPoints;
    listOfPoints.push_back({4, -8, 0.17});
    listOfPoints.push_back({6, -3, 0.17});
    listOfPoints.push_back({2.5, -2.2, 0.17});
    listOfPoints.push_back({2.5, -13.7, 0.17});
    listOfPoints.push_back({4.2, -12.2, 0.17});

    for (const vector<double>& p : listOfPoints){
        simManager.moveDroneToXYZ(p);
    }

    simManager.alignDroneWithAxis(true, PolySimulator::posZ);

    cv::Mat latestView = simManager.captureLatestViewBW();

    std::string windowName = "Current Drone View Sampled";
    cv::namedWindow(windowName);
    cv::imshow(windowName, latestView);
    cv::waitKey(1000 * 5);

    cv::destroyWindow(windowName);

    simManager.moveDroneToXYZ({4, -8, -0.17});
    simManager.alignDroneWithAxis(true, PolySimulator::posZ);

    simManager.rotateAndSlam(360*9, 360);
    std::cout << "Simulator finished making a loop" << std::endl;
    auto mapPoints = simPtr->getCurrentMapPoint();

    while(!simManager.getSimulatorPointer()->slamFinishMapReleasing()){
        //wait for map to release
    }
    simPtr->shouldSaveDroneLocations();
    simPtr->getSLAM()->SaveMap("Current Map");

//    simManager.activateExitRoomAlgorithm(mapPoints);

    simManager.simulatorThread.join();

    std::cout << "All done" << std::endl;
    return 0;
}


