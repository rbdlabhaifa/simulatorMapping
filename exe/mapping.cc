#include <ctime>
#include <fstream>
#include <iostream>
#include <System.h>
#include <unistd.h>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "Auxiliary.h"


std::string create_new_directory_named_current_time() {
    // current date/time based on current system
    time_t now = time(0);

    std::string tmp = std::string("./mapping-");
    std::string directory_named_time = tmp + std::string(ctime(&now));
    directory_named_time = boost::replace_all_copy(directory_named_time, " ", "-");

    std::filesystem::create_directories(directory_named_time);
    return directory_named_time;
}

/**
 * @brief This Program tries to create a map for the executable program
 * tello_main
 *
 * this program is just orbslam with a drone, every movement is manual.
 * This function create the a folder with the files Slam_latest_Map.bin
 * ,pointData.xyz, pointData.csv, drone_destinations.txt
 * @param argc  argv[0]=file name , argv[1]=ORBvoc.txt
 * ,argv[2]=tello_9F5EC2_640.yaml
 * @param argv
 * @return int return 0 if didn't crush , if crushed do map again
 */
int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    bool bReuse = data["continue"];
    std::string directory_named_time =
        create_new_directory_named_current_time();
    std::string Slam_lastest_Map_location =
        !bReuse ? directory_named_time + "/Slam_latest_Map.bin"
                : "Slam_latest_Map.bin";
    ORB_SLAM2::System SLAM(data["VocabularyPath"], data["DroneYamlPathSlam"], ORB_SLAM2::System::MONOCULAR, true,
                           bReuse, Slam_lastest_Map_location, bReuse);
    bool use_drone = true;
    if (data["webcam"])
        use_drone = false;
    cv::VideoCapture cap;
    cv::VideoWriter writer;
    writer.open("./" + directory_named_time + "/mapping.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(640, 480), true);
    if (use_drone) {
        //  drone.tello_stream_on();
    } else {
        cap.open(0);
    }

    int frame_cnt = 0;
    while (true) {
        if (SLAM.shutdown_requested) {
            break;
        }

        frame_cnt++;

        cv::Mat frame;
        if (use_drone) {
            // frame = drone.get_frame();
        }
        else {
            cap >> frame;
        }
        if (frame.empty()) {
            ORB_SLAM2::System::systemUsleep(20000);
            continue;
        }

        cv::resize(frame, frame, cv::Size(640, 480));
        writer.write(frame);

        cv::Mat pose = SLAM.TrackMonocular(frame, frame_cnt);
    }

    std::cout << "Finished..." << std::endl;

    if(!use_drone)
        cap.release();

    writer.release();

    SLAM.Shutdown();
    SLAM.SaveMap(Slam_lastest_Map_location);
    cvDestroyAllWindows();

    return 0;
}
