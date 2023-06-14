#include <string>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "include/Auxiliary.h"

using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string map_input_dir = data["mapInputDir"];
    // Load the ORB-SLAM2 system
    System system(vocPath, droneYamlPathSlam, System::MONOCULAR, true, true, true, map_input_dir + "simulatorMap.bin", true, false);

    // Track the monocular camera and get the current camera pose
    int frame_to_check = data["frameToCheck"];
    cv::Mat image = cv::imread(map_input_dir + "frame_" + std::to_string(frame_to_check) + ".png");
    cv::Mat pose = system.TrackMonocular(image, 0);

    std::cout << "Pose: " << pose << std::endl;

    if (!pose.empty()) {
        // Extract position from pose matrix
        double x = pose.at<float>(0,3);
        double y = pose.at<float>(1,3);
        double z = pose.at<float>(2,3);

        cv::Point3d camera_position(x, y, z);

        // Extract orientation from pose matrix
        double yaw, pitch, roll;
        yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0));
        pitch = atan2(-pose.at<float>(2,0), sqrt(pose.at<float>(2,1)*pose.at<float>(2,1) + pose.at<float>(2,2)*pose.at<float>(2,2)));
        roll = atan2(pose.at<float>(2,1), pose.at<float>(2,2));

        std::cout << "camera position: " << camera_position << std::endl << "yaw: " << yaw << ", pitch: " << pitch << ", roll: " << roll << std::endl;
    }

    return 0;
}
