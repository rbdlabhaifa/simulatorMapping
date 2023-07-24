
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

// stops the program and closes all windows when signaled
void stopProgramHandler(int s) {
    //saveMap(std::chrono::steady_clock::now().time_since_epoch().count()); // save the points found
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

    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"]; // the camera and orb slam paremeters and camera's intial position
    bool loadMap = data["loadMap"]; // whether to load an existing map created or not
    bool isSavingMap = data["saveMap"]; // whether to save the map created or not
    std::string loadMapPath = data["loadMapPath"]; // path to pre-made map in case we don't start from scratch
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, loadMap,
                                               loadMapPath,
                                               true); // create an ORB_SLAM2 system instance do to the tracking (finding keyPoints, mapPoints and more)
    
    cv::VideoCapture capture = cv::VideoCapture(0);  // capture contains the offline video and provides an API for readind and using the video data
    if (!capture.isOpened()) { // was a video put into capture? if not the end program
        std::cout << "Error opening video stream or file" << std::endl;
        return 0;
    } else {
        std::cout << "Success opening video stream or file" << std::endl;
    }

    cv::Mat frame; // a mat array to hold each frame
 
    capture >> frame;
    if (frame.empty()) {
        cout << "frame is empty" << endl;
    }
    for (;;) { // analyze using ORB slam each frame
        // perform the tracking on the current frame and update the SLAM object accordingly (find map points, keypoints and descriptors)
        SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));   
        
        capture >> frame;
        if (frame.empty()) { break; }
    }
    capture.release(); // close the video file

    SLAM->Shutdown(); // shut down the SLAM object
    cvDestroyAllWindows(); // close all open windows related to the program

    return 0;
}