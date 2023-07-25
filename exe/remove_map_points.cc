#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "System.h"
#include "Converter.h"
#include "include/Auxiliary.h"

std::vector<cv::Point3d> readPoints(std::string filename) {
    std::vector<cv::Point3d> points;

    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;
    pointData.open(filename, std::ios::in);

    while (!pointData.eof()) {
        row.clear();

        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            try
            {
                std::stod(word);
            }
            catch(std::out_of_range)
            {
                word = "0";
            }
            row.push_back(word);
        }
        points.push_back(cv::Point3d(std::stod(row[0]), std::stod(row[1]), std::stod(row[2])));
    }
    pointData.close();

    return points;
}

bool areAlmostEqualPoints(cv::Point3d a, cv::Point3d b) {
    double tolerance = 0.0001;
    if (std::abs(a.x - b.x) < tolerance && std::abs(a.y - b.y) < tolerance && std::abs(a.z - b.z) < tolerance)
        return true;
    return false;
}

bool pointInVec(cv::Point3d point, std::vector<cv::Point3d> points) {
    for (auto &curr_point : points) {
        if (areAlmostEqualPoints(curr_point, point))
            return true;
    }
    return false;
}

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Run System
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    ORB_SLAM2::System SLAM = ORB_SLAM2::System( vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, true,
                                               "/home/liam/simulatorMap.bin", false);

    // Read all first frame points
    std::vector<cv::Point3d> points = readPoints("/home/liam/firstFramePoints.csv");

    // Erase all points that don't in first frame
    std::vector<ORB_SLAM2::MapPoint*> points_to_erase;
    for (size_t i=0; i < SLAM.GetMap()->GetAllMapPoints().size(); i ++) {
        ORB_SLAM2::MapPoint* point = SLAM.GetMap()->GetAllMapPoints()[i];
        auto pointValue = point->GetWorldPos();
        Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(pointValue);
        cv::Point3d point3D = cv::Point3d(vector.x(), vector.y(), vector.z());
        if (!pointInVec(point3D, points))
            points_to_erase.push_back(point);
    }

    for (auto& point : points_to_erase) {
        SLAM.GetMap()->EraseMapPoint(point);
    }

    // Erase all frames that doesn't contains any point
    std::vector<ORB_SLAM2::KeyFrame*> keyframes_to_erase;
    for (size_t i=0; i < SLAM.GetMap()->GetAllKeyFrames().size(); i++) {
        ORB_SLAM2::KeyFrame* keyframe = SLAM.GetMap()->GetAllKeyFrames()[i];
        bool to_delete = true;
        for (auto& point : keyframe->GetMapPoints()) {
            auto pointValue = point->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(pointValue);
            cv::Point3d point3D = cv::Point3d(vector.x(), vector.y(), vector.z());
            if (pointInVec(point3D, points)) {
                to_delete = false;
                break;
            }
        }
        if (to_delete) {
            keyframes_to_erase.push_back(keyframe);
        }
    }

    for (auto& keyframe : keyframes_to_erase) {
        SLAM.GetMap()->EraseKeyFrame(keyframe);
    }

    std::cout << "Points left in map: " << SLAM.GetMap()->GetAllMapPoints().size() << std::endl;
    SLAM.SaveMap("/home/liam/a.bin");

    sleep(4);

    SLAM.Shutdown();
    cvDestroyAllWindows();

    return 0;
}