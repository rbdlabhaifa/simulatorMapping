#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "Auxiliary.h"
#include "simulator/simulator.h"
#include "RunModel/TextureShader.h"


int main(int argc, char **argv) {
    // Load the generalSettings file
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Load Config parameters of the simulator
    std::string configPath = data["slam_configuration"]["drone_yaml_path"];
    std::string vocabulary_path = data["slam_configuration"]["vocabulary_path"];
    std::string modelTextureNameToAlignTo = data["simulator_configuration"]["align_model_to_texture"]["texture"];
    bool alignModelToTexture = data["simulator_configuration"]["align_model_to_texture"]["align"];
    std::string model_path = data["simulator_configuration"]["model_path"];
    std::string simulatorOutputDir = data["simulator_configuration"]["simulator_output_dir"];
    std::string loadMapPath = data["slam_configuration"]["load_map_path"];
    double movementFactor = data["simulator_configuration"]["movement_factor"];
    double simulatorStartingSpeed = data["simulator_configuration"]["simulator_starting_speed"];
    bool runWithSlam = data["simulator_configuration"]["run_with_slam"];
    Eigen::Vector3f startingPoint((float)data["simulator_configuration"]["starting_pose"]["x"], (float)data["simulator_configuration"]["starting_pose"]["y"],
                                (float)data["simulator_configuration"]["starting_pose"]["z"]);

    Simulator simulator(configPath, model_path, alignModelToTexture, modelTextureNameToAlignTo, startingPoint, false, simulatorOutputDir, false,
                        loadMapPath, movementFactor, simulatorStartingSpeed, vocabulary_path);
    auto simulatorThread = simulator.run();

    // wait for the 3D model to load
    while (!simulator.isReady()) {
        usleep(10);
    }

    // If we want slam, wait until its loaded
    if (runWithSlam) {
        simulator.setTrack(true);
        while (!simulator.startScanning()) {
            usleep(10);
        }
        // Waits here until Tab is clicked!
    }

    // Implement here
    
    // Example
    // Forward 1
    simulator.command("forward 1");

    // Draw point, pay attention to reversed y
    Eigen::Matrix4d pose = simulator.getCurrentLocation();
    float pointSize = 10.5;
    Eigen::Vector3d color(0, 1, 0);
    simulator.drawPoint(cv::Point3d(pose(0, 3), -pose(1, 3), pose(2, 3)), pointSize, color);

    // Back 1 and clockwise 10 degrees
    simulator.command("back 1");
    simulator.command("cw 10");

    // Draw another point
    pose = simulator.getCurrentLocation();
    pointSize = 20.5;
    color = Eigen::Vector3d(1, 0, 0);
    simulator.drawPoint(cv::Point3d(pose(0, 3), -pose(1, 3), pose(2, 3)), pointSize, color);

    // Back 0.7 and clockwise 10 degrees
    simulator.command("back 0.7");
    simulator.command("cw 10");

    // Clear all points
    simulator.cleanPoints();

    // Back 0.5
    simulator.command("back 0.5");

    simulatorThread.join();

    return 0;
}
