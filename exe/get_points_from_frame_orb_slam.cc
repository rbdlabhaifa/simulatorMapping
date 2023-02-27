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
    System system(vocPath, droneYamlPathSlam, System::MONOCULAR, true, true, map_input_dir + "simulatorMap.bin", true, false);

    // Track the monocular camera and get the current camera pose
    int frame_to_check = data["frameToCheck"];
    cv::Mat image = cv::imread(map_input_dir + "frame_" + std::to_string(frame_to_check) + ".png");
    cv::Mat pose = system.TrackMonocular(image, 0);

     // Get the current Frame
    Frame currentFrame = system.GetTracker()->mCurrentFrame;

    std::vector<MapPoint*> frame_points = currentFrame.mvpMapPoints;

    int counter = 0;

    for (auto point : frame_points)
    {
        if (point)
        {
            cv::Mat currPoint = point->GetWorldPos();
            std::cout << "P: " << currPoint << ", Pn: " << point->GetNormal() << std::endl;
            //std::cout << "(" << currPoint.at<float>(0) << ", " << currPoint.at<float>(1) << ", " << currPoint.at<float>(2) << ")" << std::endl;
            counter++;
        }
    }

    std::cout << "total: " << frame_points.size() << std::endl;
    std::cout << "Valid: " << counter << std::endl;

    return 0;
}
