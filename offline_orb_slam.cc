/*********** add-comments , task1 ***********/

#include <memory> // a library who provides various smart pointers and memory related utilities
#include <string>   // a library used to handle string and characters.
#include <thread>       // a library who enable concurrent programs. (many thread run at the same time)
#include <iostream>     // a library who used for input and ouput 
#include <unistd.h>         // a library who provides access to various system-related functions and constants     
#include <unordered_set>        // a library who provides an implementation of am unordered set container
#include <nlohmann/json.hpp>        // a library who work with JSON data
#include <opencv2/videoio.hpp>  // a library that provides functions and classes for working with video input/output. 
#include <opencv2/highgui.hpp>  //a library that provides functions and classes for creating graphical user interfaces (GUI) and working with images and videos.

//libraries in line 10,11 are header files from the OpenCV (Open Source Computer Vision Library) 

#include "System.h"
#include "Converter.h"
#include "include/Auxiliary.h"

/************* SIGNAL *************/
std::unique_ptr<ORB_SLAM2::System> SLAM;    //'unique_ptr' represent execlusive ownership of a dynamically allocated object.
std::string simulatorOutputDir;


// this function saves data of the frame
/*this function takes 4 parameters: 
1. the image captured by the camera simulator
2.the position of the given frame
3.an integer representing the current frame ID (index)
4.an integer representing the number of points detected in the image*/
void saveFrame(cv::Mat &img, cv::Mat pose, int currentFrameId, int number_of_points) {
    
    //image is empty, so return!
    if (img.empty()) 
    {
        std::cout << "Image is empty!!!" << std::endl;
        return;
    }

    std::ofstream frameData;    //creating instance of 'ofstream' called frameData
    //open frameData and write there the currentFrameId
    frameData.open(simulatorOutputDir + "frameData" +
                   std::to_string(currentFrameId) + ".csv");    

    std::ofstream framePointsCount; //creating instance of 'ofstream' called framePointsCount
    //open framePointsCount and write there the num of points detected in the image
    framePointsCount.open(simulatorOutputDir + "framePointsCount" +
                   std::to_string(currentFrameId) + ".txt");    
    framePointsCount << number_of_points;  //update num of points  
    framePointsCount.close();   

    // Extract position from pose matrix    (num)
    double x = pose.at<float>(0,3);
    double y = pose.at<float>(1,3);
    double z = pose.at<float>(2,3);

    cv::Point3d camera_position(x, y, z);      

    // Extract orientation from pose matrix (angle)
    double yaw, pitch, roll;
    yaw = atan2(pose.at<float>(1,0), pose.at<float>(0,0));
    pitch = atan2(-pose.at<float>(2,0), sqrt(pose.at<float>(2,1)*pose.at<float>(2,1) + pose.at<float>(2,2)*pose.at<float>(2,2)));
    roll = atan2(pose.at<float>(2,1), pose.at<float>(2,2));

    //save frame data (currentFrameId, positions(x,y,z), positions(yaw,pitch,roll))
    frameData << currentFrameId << ',' << camera_position.x << ',' << camera_position.y << ',' << camera_position.z << ','
              << yaw << ',' << pitch << ',' << roll << std::endl;
    cv::imwrite(simulatorOutputDir + "frame_" + std::to_string(currentFrameId) + ".png", img);
    frameData.close();
}

// this function save the entire map
//this function take one paramater -> mapNumber 
void saveMap(int mapNumber) {
    std::ofstream pointData;
    std::unordered_set<int> seen_frames;        //the frames that we have seen
    int i = 0;

    //open an output file to save the data of the map point data
    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    
    //go through all map points
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        //we want only good points, so check every point, if it's a good one or no (and check if it's not NULL)
        if (p != nullptr && !p->isBad()) {      
            //if it's not null, and a good point so:

            //extract the 3D world position of the map point
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth();  //update the normal and depth info for the point
            cv::Mat Pn = p->GetNormal();
            Pn.convertTo(Pn, CV_64F);
           
            //save map point data
            pointData << i << ",";  //save index of point
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);  //save 3D world position
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << "," << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2); //save maximum and minimum distance invariances, and the positions

            //take the keyframe and the observation of the current map point
            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations();
            std::ofstream keyPointsData;
            std::ofstream descriptorData;

            //open two output files to save the keyPoint and the descriptor of the current map point
            keyPointsData.open(simulatorOutputDir + "point" + std::to_string(i) + "_keypoints.csv");
            descriptorData.open(simulatorOutputDir + "point" + std::to_string(i) + "_descriptors.csv");
            
            //go through all the observations and save the keypoint data and desciptor
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
            //close files 
            keyPointsData.close();
            descriptorData.close();

            pointData << std::endl; //inserts a newline character and flushes the data to the output file, ensuring that each map point data entry is written on a separate line in the CSV file. 
            i++;    //update index of point
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;  //map saving is complete

}

// handler function to stop the program gracefully (when a signal is received)
//s -> which signal was sent 
void stopProgramHandler(int s) {
    saveMap(std::chrono::steady_clock::now().time_since_epoch().count());   //save the current map
    SLAM->Shutdown();   //shutDown the SLAM system (freeing resources and cleaning up before the program exits)
    cvDestroyAllWindows();  //close all the CV windows (for example: the lab picture/the visualization window that includes the point...)
    std::cout << "stoped program" << std::endl; 
    exit(1);
}

int main() {
    //signals that help when we need to stop the program gracefully
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);

    //reading settings (from JSON file) and save them 
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;    
    programData >> data;
    programData.close();
    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    //get current time and create an output directory for saving the SLAM results.
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
    
    //Create-initialize an ORB-SLAM2 system
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true);
    int amountOfAttepmpts = 0;
    //video processing loop (processes the video frames from 'videoPath')
    while (amountOfAttepmpts++ < 1) {
        //open the video of the of the lab
        cv::VideoCapture capture(videoPath);
        //check if we can successfully open the video or not.
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        //skip the first 170 frames (to avoid the initial frames that might not be suitable for mapping or have unstable camera pose estimates)
        for (int i = 0; i < 170; ++i) {
            capture >> frame;
        }
        int amount_of_frames = 1;

        
        //processes the subsequent video frames one by one. For each frame
        for (;;) {
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC)); //perform SLAM tracking on the frame and estimate the camera pose and 3D map points.

            capture >> frame;   //read the next frame from the video

            if (frame.empty()) {    //the next frame is empty (end of video), the loop terminates
                break;
            }
        }
        saveMap(amountOfAttepmpts); //save map
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        //The program outputs the time taken for SLAM processing and the total number of frames processed.
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << std::endl;
        std::cout << amount_of_frames << std::endl;
        capture.release();  //Releasing the video resources
    }

    //save map
    if (isSavingMap) {
        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    }

    SLAM->Shutdown();   //shutDown the SLAM system (freeing resources and cleaning up before the program exits)
    cvDestroyAllWindows();  //close all the CV windows (for example: the lab picture/the visualization window that includes the point...)

    return 0;
}

