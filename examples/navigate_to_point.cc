#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "Auxiliary.h"
#include "simulator/simulator.h"
#include "RunModel/TextureShader.h"

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

#define ANIMATION_DURATION 5.0
#define ANIMATION_STEPS 100.0

void applyYawRotationToModelCam(pangolin::OpenGlRenderState *s_cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    s_cam->SetModelViewMatrix(newModelView);
}

int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["slam_configuration"]["DroneYamlPathSlam"];
    std::string vocabulary_path = data["slam_configuration"]["VocabularyPath"];
    std::string modelTextureNameToAlignTo = data["simulator_configuration"]["align_model_to_texture"]["texture"];
    bool alignModelToTexture = data["simulator_configuration"]["align_model_to_texture"]["align"];
    std::string model_path = data["simulator_configuration"]["modelPath"];
    std::string simulatorOutputDir = data["simulator_configuration"]["simulatorOutputDir"];
    std::string loadMapPath = data["slam_configuration"]["loadMapPath"];
    double movementFactor = data["simulator_configuration"]["movementFactor"];
    double simulatorStartingSpeed = data["simulator_configuration"]["simulatorStartingSpeed"];
    bool runWithSlam = data["simulator_configuration"]["run_with_slam"];

    Simulator simulator(configPath, model_path, alignModelToTexture, modelTextureNameToAlignTo, false, simulatorOutputDir, false,
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

    simulatorThread.join();

    return 0;
}
