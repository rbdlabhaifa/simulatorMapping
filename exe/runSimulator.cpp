//
// Created by tzuk on 6/4/23.
//
#include <matplotlibcpp.h>
#include "simulator/simulator.h"
#include "navigation/RoomExit.h"
#include "include/Auxiliary.h"

int main(int argc, char **argv) {
    std::ifstream programData(argv[1]);//This line creates an input file stream programData and opens the file whose path is provided as the first command-line argument (argv[1]).
    nlohmann::json data;//creates a JSON object data
    programData >> data;//This line reads the JSON data from the file and stores it in the data JSON object. 
    programData.close();// closes the input file stream after reading the JSON data.

    //retrieves the value associated with the key "DroneYamlPathSlam" from the JSON object data and assigns it to the configPath
    std::string configPath = data["DroneYamlPathSlam"];
    //retrieves the value associated with the key "VocabularyPath" from the JSON object data and assigns it to  VocabularyPath
    std::string VocabularyPath = data["VocabularyPath"];
    //retrieves the value associated with the key "modelTextureNameToAlignTo" from the JSON object data and assigns it to  modelTextureNameToAlignTo
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];

    //retrieves the value associated with the key "modelPath" from the JSON object data and assigns it to   model_path.
    std::string model_path = data["modelPath"];

    //retrieves the value associated with the key "trackImages" from the JSON object data and assigns it to  trackImages
    bool trackImages = data["trackImages"];

    //retrieves the value associated with the key "movementFactor" from the JSON object data and assign it  to movementFactor. I
    double movementFactor = data["movementFactor"];

    //creates a Simulator object named simulator
    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, "../../slamMaps/", false,
                        "", movementFactor,VocabularyPath);

    // starts the simulator by calling the run()
    auto simulatorThread = simulator.run();
    while (!simulator.isReady()) { // wait for the 3D model to load
        usleep(1000);
    }


    std::cout << "to stop press k" << std::endl;
    std::cout << "to stop tracking press t" << std::endl;
    std::cout << "to save map point press m" << std::endl;
    std::cout << "waiting for key press to start scanning " << std::endl << std::endl;
    std::cin.get(); //waits for user input from the standard input (keyboard) and pauses the program until the user presses the Enter key
    simulator.setTrack(true); //This line calls the setTrack() method of the simulator object and sets the tracking mode to true. 
    int currentYaw = 0;
    int angle = 5;
    cv::Mat runTimeCurrentLocation;
    for (int i = 0; i < std::ceil(360 / angle); i++) {
        std::string c = "left 0.7";
        simulator.command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        c = "right 0.7";
        simulator.command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        c = "cw " + std::to_string(angle);
        simulator.command(c);
        //runTimeCurrentLocation = simulator.getCurrentLocation();
        sleep(1);
    }
    //simulator.setTrack(false);
    sleep(2);

    auto scanMap = simulator.getCurrentMap(); //This line calls the getCurrentMap() method of the simulator object and stores the result in the scanMap variable
    std::vector<Eigen::Vector3d> eigenData;//declare an empty std::vector named eigenData
    for (auto &mp: scanMap) {
        if (mp != nullptr && !mp->isBad()) {//checks if the current map element mp is not a null pointer (nullptr) 
            auto vector = ORB_SLAM2::Converter::toVector3d(mp->GetWorldPos());//: This line uses the ORB_SLAM2::Converter class to convert the 3D world position of the current map point (mp->GetWorldPos()) into an Eigen vector (Eigen::Vector3d). 
            eigenData.emplace_back(vector);//adds the vector to the end of the eigenData
        }
    }
    RoomExit roomExit(eigenData);// creates a RoomExit object called roomExit and passes the eigenData
    auto exitPoints = roomExit.getExitPoints(); //calls the getExitPoints() method of the roomExit object and stores the result in the exitPoints
    std::sort(exitPoints.begin(), exitPoints.end(), [&](auto &p1, auto &p2) {//sorts the exitPoints vector in ascending order based on the first element of each pair in the vector.
        return p1.first < p2.first;
    });
    auto currentLocation = ORB_SLAM2::Converter::toVector3d(simulator.getCurrentLocation().rowRange(0, 2).col(3));//converts the current location of the simulator object to a 3D vector 

    double currentAngle = std::atan2(currentLocation.z(),currentLocation.x()); // calculates the angle (in radians) between the z and x components of the currentLocation vector
    double targetAngle = std::atan2(exitPoints.front().second.z(),exitPoints.front().second.x());//calculates the angle (in radians) between the z and x components of the first element in the exitPoints
    int angle_difference = targetAngle;

    std::string rotCommand;   
    if (angle_difference<0){
        rotCommand = "ccw " + std::to_string(std::abs(angle_difference));// creates a command to rotate counterclockwise (ccw) 

    }else{
        rotCommand = "cw "+std::to_string(angle_difference); //creates a command to rotate clockwise.
    }
    std::cout << rotCommand << std::endl;
    simulator.command(rotCommand); //ends the rotation command to the simulator
    double distanceToTarget = (currentLocation-exitPoints.front().second).norm(); //alculates the Euclidean distance between the currentLocation (camera position) and the first element in the exitPoints vector (target exit point)
        std::string forwardCommand = "forward " + std::to_string( 3*int(distanceToTarget));// creates a command to move the camera forward by a distance equal to three times 
        std::cout << forwardCommand << std::endl;// prints the forward movement command 
        simulator.command(forwardCommand);//sends the forward movement command to the simulator object, 
    simulatorThread.join();//waits for the simulatorThread to finish,
}
