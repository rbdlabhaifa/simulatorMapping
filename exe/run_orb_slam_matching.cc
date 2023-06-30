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
std::unique_ptr<ORB_SLAM2::System> SLAM;
std::string simulatorOutputDir;

struct MatCompare {
    bool operator()(const cv::Mat& a, const cv::Mat& b) const {
        if (a.rows != b.rows || a.cols != b.cols || a.type() != b.type()) {
            return false;
        }
        for (int i = 0; i < a.rows; i++) {
            const void* a_row = a.ptr(i);
            const void* b_row = b.ptr(i);
            if (memcmp(a_row, b_row, a.cols*a.elemSize()) != 0) {
                return false;
            }
        }
        return true;
    }
};

void stopProgramHandler(int s) {
    SLAM->Shutdown();
    cvDestroyAllWindows();
    std::cout << "stoped program" << std::endl;
    exit(1);
}

int main() {
    signal(SIGINT, stopProgramHandler);
    signal(SIGTERM, stopProgramHandler);
    signal(SIGABRT, stopProgramHandler);
    signal(SIGSEGV, stopProgramHandler);
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    std::string loadMapPath = data["loadMapPath"];
    SLAM = std::make_unique<ORB_SLAM2::System>(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, false,
                                               loadMapPath,
                                               true);
    std::set<cv::Mat, MatCompare> descriptorsSet;
    int amountOfAttepmpts = 0;
    while (amountOfAttepmpts++ < 1) {
        cv::VideoCapture capture(videoPath);
        if (!capture.isOpened()) {
            std::cout << "Error opening video stream or file" << std::endl;
            return 0;
        } else {
            std::cout << "Success opening video stream or file" << std::endl;
        }

        cv::Mat frame;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 170; ++i) {
            capture >> frame;
        }
        int amount_of_frames = 1;

        for (;;) {
            SLAM->TrackMonocular(frame, capture.get(CV_CAP_PROP_POS_MSEC));

            capture >> frame;

            if (frame.empty()) {
                break;
            }

            ORB_SLAM2::Frame& currentFrame = SLAM->GetTracker()->mCurrentFrame;
            cv::Mat descriptorsFrame = currentFrame.mDescriptors;
            descriptorsSet.insert(descriptorsFrame);
        }
        std::cout << amount_of_frames << std::endl;
        capture.release();
    }

    SLAM->Shutdown();
    cvDestroyAllWindows();

    // Concatenate all the descriptors in descriptors_map

    cv::Mat descriptorsOrbSlam;
    std::vector<cv::Mat> descriptorsVec;
    for (auto desc : descriptorsSet)
        descriptorsVec.emplace_back(desc);
    cv::vconcat(descriptorsVec, descriptorsOrbSlam);

    // Set up ORB feature detector and matcher
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2, 8);
    cv::BFMatcher matcher(cv::NORM_HAMMING, false);

    // Process images in the input folder and find the one that matches the most
    std::string frames_folder = data["framesFolder"];
    std::string input_folder_path = frames_folder;
    std::vector<std::string> image_paths;
    std::vector<cv::Mat> descriptors_images;
    std::vector<int> scores;
    for (const auto& entry : std::filesystem::directory_iterator(input_folder_path)) {
        // Load image
        cv::Mat img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);

        // Extract ORB keypoints and descriptors from the image
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(img, cv::Mat(), keypoints, descriptors);

        // Match descriptors with the map
        std::vector<std::vector<cv::DMatch>> matches;
        matcher.knnMatch(descriptors, descriptorsOrbSlam, matches, 2);

        // Find good matches
        std::vector<cv::DMatch> good_matches;
        for (auto& match: matches) {
            if (match[0].distance < 0.7 * match[1].distance) {
                good_matches.push_back(match[0]);
            }
        }

        // Save the image path and descriptors
        image_paths.push_back(entry.path().string());
        descriptors_images.push_back(descriptors);
        scores.push_back((int)good_matches.size());

        // Print the score
        std::cout << "Image: " << entry.path().string() << ", Score: " << good_matches.size() << std::endl;
    }

    // Find the image with the highest similarity score
    int best_index = 0;
    double best_score = 0;
    for (int i = 0; i < image_paths.size(); ++i) {
        if (scores[i] > best_score) {
            best_index = i;
            best_score = scores[i];
        }
    }

    std::cout << "Best Image: " << image_paths[best_index] << ", Score: " << best_score << std::endl;

    return 0;
}
