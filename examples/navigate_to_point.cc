#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "Auxiliary.h"
#include "simulator/simulator.h"
#include "RunModel/TextureShader.h"


int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

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
        usleep(1000);
    }
    if (runWithSlam) {
        simulator.setTrack(true);
        // wait for the 3D model to load
        while (!simulator.startScanning()) {
            usleep(10);
        }
    }

    // Options
    double targetPointX = data["target_pose"]["x"];
    double targetPointY = data["target_pose"]["y"];
    double targetPointZ = data["target_pose"]["z"];
    Eigen::Vector3d target_point(targetPointX, targetPointY, targetPointZ);

    // Implement moving to target

    simulatorThread.join();

    return 0;
}
