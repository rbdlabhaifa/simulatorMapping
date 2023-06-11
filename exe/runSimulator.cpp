//
// Created by tzuk on 6/4/23.
//
#include "simulator.h"
#include "include/Auxiliary.h"

int main(int argc, char **argv) {
    std::ifstream programData(argv[1]);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];

    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo);
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
    int angle = 10;
    cv::Mat currentLocation;
    for (int i = 0; i < std::ceil(360 / angle); i++) {
        std::string c = "forward 0.5";
        simulator.command(c);
        usleep(500000);
        currentLocation = simulator.getCurrentLocation();
        c = "back 0.5";
        simulator.command(c);
        usleep(500000);
        currentLocation = simulator.getCurrentLocation();
        c = "cw " + std::to_string(angle);
        simulator.command(c);
        sleep(1);
        currentLocation = simulator.getCurrentLocation();
    }
    auto scanMap = simulator.getCurrentMap();

    simulatorThread.join();
}