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
std::unique_ptr<ORB_SLAM2::System> SLAM; //Slam variable that will allow us to use its functionality
std::string simulatorOutputDir; //path of the directory to put the output in


//saveFrame function is not used at all. saves information about the frame.
void saveFrame(cv::Mat &img, cv::Mat pose, int currentFrameId, int number_of_points) {
    if (img.empty()) //if there is no input, stop the running
    {
        std::cout << "Image is empty!!!" << std::endl;
        return;
    }
    std::ofstream frameData;
    frameData.open(simulatorOutputDir + "frameData" +
                   std::to_string(currentFrameId) + ".csv"); //open the file

    std::ofstream framePointsCount;
    framePointsCount.open(simulatorOutputDir + "framePointsCount" +
                   std::to_string(currentFrameId) + ".txt"); //open the file
    framePointsCount << number_of_points; //write to file the number of points
    framePointsCount.close();

    // Extract position from pose matrix
    double x = pose.at<float>(0,3);
    double y = pose.at<float>(1,3);
    double z = pose.at<float>(2,3);

    cv::Point3d camera_position(x, y, z); //concert the position to Point3d variable

    // Extract orientation from pose matrix
    double yaw, pitch, roll;
    yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0)); 
    pitch = atan2(-pose.at<float>(2,0), sqrt(pose.at<float>(2,1)*pose.at<float>(2,1) + pose.at<float>(2,2)*pose.at<float>(2,2))); 
    roll = atan2(pose.at<float>(2,1), pose.at<float>(2,2)); 

    frameData << currentFrameId << ',' << camera_position.x << ',' << camera_position.y << ',' << camera_position.z << ','
              << yaw << ',' << pitch << ',' << roll << std::endl; //write details about the frame in the file
    cv::imwrite(simulatorOutputDir + "frame_" + std::to_string(currentFrameId) + ".png", img);
    frameData.close();
}

//the function saves the map points (all observations - keypoints and descriptors). 
//mapNumber is always 1
void saveMap(int mapNumber) {
    std::ofstream pointData; //contains the cloud of map points
    std::unordered_set<int> seen_frames; //unused set
    int i = 0;

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv"); //open the cloud of map points
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) { //go over each map point created by SLAM
        if (p != nullptr && !p->isBad()) { //if the point is empty or bad, continue to the next point
            auto point = p->GetWorldPos(); //get the poisition of the point as a matrix
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point); //convert the point to 3d vector, with x y z
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F); //world position. initialized with zeros.
            worldPos.at<double>(0) = vector.x(); //poisiton according to X axis
            worldPos.at<double>(1) = vector.y(); //poisiton according to Y axis
            worldPos.at<double>(2) = vector.z(); //poisiton according to Z axis
            p->UpdateNormalAndDepth();  //update normal (avg of the normalized distance vectors between the camera and the observations of the map points) and depth
            cv::Mat Pn = p->GetNormal(); //gets the normal
            Pn.convertTo(Pn, CV_64F); //convert to grayscale image

            //save details about the point and its normal
            pointData << i << ",";
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);

            //for every map point, go over its observations, and save the KeyPoint&Descriptors
            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations(); //all frames which the point was a key point
            std::ofstream keyPointsData;
            std::ofstream descriptorData;
            keyPointsData.open(simulatorOutputDir + "point" + std::to_string(i) + "_keypoints.csv");
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_descriptors.csv");
            for (auto obs : observations) {
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

//Saving the map and ending the run
void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count()); //save the map
    SLAM->Shutdown(); //turn off the SLAM variable
    cvDestroyAllWindows(); //destory all the windows that were open
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    //if a signal that requires stopping the program is recieved go to stopProgramHandler
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);
    
    //put the general settings path in json variable named data
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    
    //unused
    char currentDirPath[256]; 
    getcwd(currentDirPath, 256); //curr directory

    //save the current time in order to save it in directory name
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now)); //convert time to string and save in time_buf
    std::string currentTime(time_buf); 


    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"]; //the settings of the camera
    std::string videoPath = data["offlineVideoTestPath"]; //path to the video 
    bool loadMap = data["loadMap"]; //whether load a map or start from the beginning
    bool isSavingMap = data["saveMap"]; //if we need to save the map
    std::string loadMapPath = data["loadMapPath"]; //the path to the map (if we want to load it)
    std::string simulatorOutputDirPath = data["simulatorOutputDir"]; //path of the directory to put the output in
    simulatorOutputDir = simulatorOutputDirPath + currentTime + "/"; //providing a specific path to the directory with the currnt time
    std::filesystem::create_directory(simulatorOutputDir); //create a new directory where we will put the output of the simulator
                                                            //the path includes the current time
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true); //instance of System variable, allows access to ORBSLAM functionality
    int amountOfAttepmpts = 0; //unused variable. we can to remove it and the while-loop.
    while (amountOfAttepmpts++ < 1) { //do it one time
        cv::VideoCapture capture(videoPath); //the video
        if (!capture.isOpened()) { // if opening the video has failed, stop the running
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame; //data structure that will save the frames
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 170; ++i) { //skip the 170 first frames (we save it)
            capture >> frame;
        }
        int amount_of_frames = 1; //unused variable (its not updated)

        for (;;) { //switch to do-while loop
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC)); //track all the frames. 
            //receives a frame and finds key points in it. Updates the SLAM with new information about the map points.

            capture >> frame; // save curr frame and go to next frame

            if (frame.empty()) { //if the frame is empty, stop
                break;
            }
        }
        saveMap(amountOfAttepmpts); //save the map (map points, keypoints and descriptors)

        //print the time differece
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();  //when we ended play the video
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;

        std::cout << amount_of_frames << std::endl; //print the amount of frames. the amount_of_frames variable is not updated!
        capture.release(); //release the video
    }

    if (isSavingMap) { //if we need to save the map, do it
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    SLAM->Shutdown(); //turn off the SLAM variable
    cvDestroyAllWindows(); //destory all the windows that were open

    return 0;
}