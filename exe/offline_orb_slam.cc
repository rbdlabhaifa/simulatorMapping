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
std::unique_ptr<ORB_SLAM2::System> SLAM; // an orb slam system instance allowing the usage of orb slam functionalities
std::string simulatorOutputDir; //path of directory to put the output in

// filters the 3d points found with orb slam and saves them using mapNumber for the name of the directory
// the info of the map points is saved to pointData, the info of the keyPoints is saved to keyPointsData
// and the keyPoints descriptors are saved to descriptorData
void saveMap(int mapNumber) {
    std::ofstream pointData; // the data of the points will be saved to this object
    int i = 0;

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) { // go over each map point created by SLAM
        if (p != nullptr && !p->isBad()) { // if point is "bad" (decided by orbSlam) then don't save it
            auto point = p->GetWorldPos(); // get the position of the point
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point); // convert point to a 3d matrix and put it in vector
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F); // fill worldPos with zeros
            
            // set worldPose to be the position of the point after conversion
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();

            p->UpdateNormalAndDepth(); // update fields of depth of point in relation to camera and the normal of the point
            cv::Mat Pn = p->GetNormal(); // the normal is the average of the normalized distance vectors between the camera 
                                         // (at different positions) and the keypoints relating to the same map point and update depth
            Pn.convertTo(Pn, CV_64F); // convert normal to grayscale format
            
            // add the point and important imformation about it to PointData file (like position in space, normal and more)
            pointData << i << ",";
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);

            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations(); // observations will contain all the key frames that p was observed in
            std::ofstream keyPointsData; // contains information about all instances where p was observed as a key point
            std::ofstream descriptorData; // contains the descriptors of all the key points p correlates to
            keyPointsData.open(simulatorOutputDir + "point" + std::to_string(i) + "_keypoints.csv"); // save keyPointsData to specified path
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_descriptors.csv"); // save descriptorData to specified path
            for (auto obs : observations) { // go over each observation (key frame) that p was observed in
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;

                // Save keyPoints
                cv::KeyPoint currentKeyPoint = currentFrame->mvKeys[obs.second];
                keyPointsData << currentFrame->mnId << "," << currentKeyPoint.pt.x << "," << currentKeyPoint.pt.y <<
                              "," << currentKeyPoint.size << "," << currentKeyPoint.angle << "," <<
                              currentKeyPoint.response << "," << currentKeyPoint.octave << "," <<
                              currentKeyPoint.class_id << std::endl;

                // Save Descriptors
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

// stops the program and closes all windows when signaled
void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count()); // save the points found
    SLAM->Shutdown(); // shut down orb slam
    cvDestroyAllWindows(); // close all open windows related to the program
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    // set the handler of the following signals (mostly termintion signals)
    // to be stopProgramHandler(int s).
    signal(SIGINT, stopProgramHandler); 
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);

    std::string settingPath = Auxiliary::GetGeneralSettingsPath(); // setting the path to the configuration of the program (generalSettings.json)
    std::ifstream programData(settingPath);
    nlohmann::json data; // create an empty json object
    programData >> data; // put programData into data.json
    programData.close(); // close programData file

    // get the current time to put in the directory name
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);


    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"]; // the camera and orb slam paremeters and camera's intial position
    std::string videoPath = data["offlineVideoTestPath"]; // the path of the offline video
    bool loadMap = data["loadMap"]; // whether to load an existing map created or not
    bool isSavingMap = data["saveMap"]; // whether to save the map created or not
    std::string loadMapPath = data["loadMapPath"]; // path to pre-made map in case we don't start from scratch
    std::string simulatorOutputDirPath = data["simulatorOutputDir"]; // the path to save the points to 
    simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir); // create a directory in the path specified
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true); // create an ORB_SLAM2 system instance do to the tracking (finding keyPoints, mapPoints and more)
    int amountOfAttepmpts = 0;
    while (amountOfAttepmpts++ < 1) { // run once
        cv::VideoCapture capture(videoPath); // capture contains the offline video and provides an API for readind and using the video data
        if (!capture.isOpened()) { // was a video put into capture? if not the end program
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame; // a mat array to hold each frame
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); // get the current time
        for (int i = 0; i < 170; ++i) { // skip over the 170 first frames of fotage
            capture >> frame; // go to next frame
        }
        int amount_of_frames = 1; // not changed at all

        for (;;) { // analyze using ORB slam each frame
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC)); // perform the tracking on the current frame  
                                                                            // and update the SLAM object accordingly 
                                                                            // (find map points, keypoints and descriptors)

            capture >> frame;

            if (frame.empty()) {
                break;
            }
        }
        saveMap(amountOfAttepmpts); // save the point map and additional information

        // measuring the printing the amount of time it took the tracking and saving to run
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release(); // close the video file
    }

    // if specified to save the map then the SLAM object will also save the map to simulatorMap.bin
    if (isSavingMap) {
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    SLAM->Shutdown(); // shut down the SLAM object
    cvDestroyAllWindows(); // close all open windows related to the program

    return 0;
}

