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

    while (!simManager.getSimulatorPointer()->isSetInPlace()){
        usleep(1000);
    }

    //simManager.rotateAndSlam(45, 360);
    //simManager.reflectDroneView();


    simManager.simulatorThread.join();

    std::cout << "All done" << std::endl;
    return 0;
}


