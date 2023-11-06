//
// Created by Dean on 9/30/23.
//

#include <matplotlibcpp.h>
#include "simulator/polySimulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"

#include <iostream>

#include "navigation/ExitRoomFiles/exit_room_algo.cpp"
#include "../tools/simulator/polySimulator.h"
#include "../tools/simulator/simulatorManager.h"

using std::vector;



int main(int argc, char **argv) {
    bool loadCustomMap = std::stoi(argv[1]);
    SimulatorManager &simManager = SimulatorManager::getInstance(loadCustomMap);

    std::cout << std::endl << "running " << argv[0] << " file" << std::endl;

    //This loop makes sure you want to activate this series of actions by letting you change the isSetPlace flag of the simulator to 'true' whenever you want to execute the code.
    while (!simManager.getSimulatorPointer()->isSetInPlace()){
//        simManager.getSimulatorPointer()->printCurrentLocation();
        usleep(1000);
    }

    simManager.moveDroneToXYZ({4, -8, 0.17}, 30000);
    simManager.rotateAndSlam(2, 360);

    simManager.moveDroneToXYZ({6, -3, 0.17}, 30000);
    simManager.rotateAndSlam(2, 360);

    simManager.moveDroneToXYZ({2.5, -2.2, 0.17}, 30000);
    simManager.rotateAndSlam(2, 360);

    simManager.moveDroneToXYZ({2.5, -13.7, 0.17}, 30000);
    simManager.rotateAndSlam(2, 360);

    simManager.moveDroneToXYZ({4.2, -12.2, 0.17}, 30000);
    simManager.rotateAndSlam(2, 360);

    //simManager.reflectDroneView();


    simManager.simulatorThread.join();

    std::cout << "All done" << std::endl;
    return 0;
}


