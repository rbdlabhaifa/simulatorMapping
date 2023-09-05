//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
//#include "simulator.h"
#include "navigation/include/RoomExit.h"
#include "include/Auxiliary.h"
//#include "AutonomousDrone.h"

int main(int argc, char **argv) {
    //Loading generalSettings.json
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    //Defining constant variables
    std::string configPath = data["DroneYamlPathSlam"];
    std::string VocabularyPath = data["VocabularyPath"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string map_input_dir = data["mapInputDir"];
    bool trackImages = data["trackImages"];
    double movementFactor = data["movementFactor"];

    //Creating the simulator
//    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, map_input_dir, false,
//                        "", movementFactor,VocabularyPath);
//    auto simulatorThread = simulator.run();
//    while (!simulator.isReady()) { // wait for the 3D model to load
//        usleep(1000);
//    }

    //Need to take care of the case of loss of localization
//    auto localizationThread = simulator.localization();


//    AutonomousDrone autonomousDrone(&simulator);
//    std::thread thread(&AutonomousDrone::run, autonomousDrone);


//    std::cout << "to stop press k" << std::endl;
//    std::cout << "to stop tracking press t" << std::endl;
//    std::cout << "to save map point press m" << std::endl;
//    std::cout << "waiting for key press to start scanning " << std::endl << std::endl;
//    std::cin.get();


//    simulatorThread.join();
}
