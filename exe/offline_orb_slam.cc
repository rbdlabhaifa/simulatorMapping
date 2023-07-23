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

/************* SIGNAL *************/
std::unique_ptr<ORB_SLAM2::System> SLAM; // an instance which helps us use the interface of ORB_SLAM
std::string simulatorOutputDir;  // stores the path of the output directory


// this function saves the information about a frame
void saveFrame(cv::Mat &img, cv::Mat pose, int currentFrameId, int number_of_points) {
    if (img.empty())  
    {
        std::cout << "Image is empty!!!" << std::endl;  // there is no information to save on empty frame
        return;
    }
    std::ofstream frameData;
    frameData.open(simulatorOutputDir + "frameData" +
                   std::to_string(currentFrameId) + ".csv");  // we open a new file to store the information

    std::ofstream framePointsCount;
    framePointsCount.open(simulatorOutputDir + "framePointsCount" +
                   std::to_string(currentFrameId) + ".txt");
    framePointsCount << number_of_points;   // saving the amount of key points in this frame
    framePointsCount.close();

    // Extract position from pose matrix
    double x = pose.at<float>(0,3);
    double y = pose.at<float>(1,3);
    double z = pose.at<float>(2,3);

    cv::Point3d camera_position(x, y, z); // we save the location of the camera

    // Extract orientation from pose matrix
    double yaw, pitch, roll;
    yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0)); // spin on x axis
    pitch = atan2(-pose.at<float>(2,0), sqrt(pose.at<float>(2,1)*pose.at<float>(2,1) + pose.at<float>(2,2)*pose.at<float>(2,2))); // spin on y axis
    roll = atan2(pose.at<float>(2,1), pose.at<float>(2,2)); // spin on z axis

    frameData << currentFrameId << ',' << camera_position.x << ',' << camera_position.y << ',' << camera_position.z << ','
              << yaw << ',' << pitch << ',' << roll << std::endl; // we store the camera details
    cv::imwrite(simulatorOutputDir + "frame_" + std::to_string(currentFrameId) + ".png", img);  // we store the frame
    frameData.close();
}

// this function saves the descriptors, keyPoints and the Map Points
void saveMap(int mapNumber) {
    std::ofstream pointData;
    std::unordered_set<int> seen_frames; // we dont use this
    int i = 0;

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {     // we go over all of our Map Points which we extract from the video
        if (p != nullptr && !p->isBad()) {
            auto point = p->GetWorldPos();  // the cordinates of our 3d Map Point as a Matrix
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth(); // update the average devitaion of the camera from the point(normal) and update possible depth
            cv::Mat Pn = p->GetNormal(); // gets the updated normal
            Pn.convertTo(Pn, CV_64F);

            //saves the position of each Map Point and it's normal(average devitaion of the camera from the point)
            pointData << i << ",";
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);

            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations(); // gets all of the frames which have p
            std::ofstream keyPointsData;
            std::ofstream descriptorData;
            keyPointsData.open(simulatorOutputDir + "point" + std::to_string(i) + "_keypoints.csv");
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_descriptors.csv");
            for (auto obs : observations) { // go over all of the key Points related to this Map Point
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;

                // Save keyPoints information
                cv::KeyPoint currentKeyPoint = currentFrame->mvKeys[obs.second];
                keyPointsData << currentFrame->mnId << "," << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y <<
                              "," << currentKeyPoint.size << "," << currentKeyPoint.angle << "," <<
                              currentKeyPoint.response << "," << currentKeyPoint.octave << "," <<
                              currentKeyPoint.class_id << std::endl;

                // Save Descriptors information
                cv::Mat current_descriptor = currentFrame->mDescriptors.row(obs.second);
                for (int j=0; j < current_descriptor.rows; j++) {
                    descriptorData << static_cast<int>(current_descriptor.at<uchar>(j, 0));
                    for (int k=1; k < current_descriptor.cols; k++) {
                        descriptorData << "," << static_cast<int>(current_descriptor.at<uchar>(j, k));
                    }
                    descriptorData << std::endl;
                }
            }
            keyPointsData.close();
            descriptorData.close();

            pointData << std::endl;
            i++;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

// this function sets the behavior of the program when we get a signal s
//stops the program
void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count());
    SLAM->Shutdown();
    cvDestroyAllWindows();
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    // set signal handlers
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);

    //get genetal settings
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    //extract the information for the SLAM instance
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string simulatorOutputDirPath = data["simulatorOutputDir"];
    simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true); // creating the SLAM instance


    // entering the main loop of the program, we will exit whem we finish to go over the video/all frames        
    int amountOfAttepmpts = 0;
    while (amountOfAttepmpts++ < 1) {
        cv::VideoCapture capture(videoPath); // open the video
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); // saves when we started to play the video
        for (int i = 0; i < 170; ++i) { // skips the 170 first frames
            capture >> frame;
        }
        int amount_of_frames = 1; // useless - we don't count them anyways

        // we go over all the frames
        for (;;) {
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC)); // finds Map Points, their keyPoints and descriptors from an image and her time stamp and save it inside SLAM

            capture >> frame; // move to the next frame

            if (frame.empty()) {  // when the frame is empty we don't need to check it
                break;
            }
        }
        saveMap(amountOfAttepmpts); // we save our keyPoints, descriptors and Map Points
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); // saves when we ended to play the video

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;  // length of the video
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    // we can save our map using slam
    if (isSavingMap) {
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    //we end our use of slam
    SLAM->Shutdown();
    cvDestroyAllWindows();

    return 0;
}

