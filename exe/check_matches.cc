#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include "System.h"

#include "include/Auxiliary.h"

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
  std::string frames_folder = data["framesFolder"];

  // Load ORB_SLAM2 map and extract its descriptors
  ORB_SLAM2::System system(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, map_input_dir + "simulatorMap.bin", true, false);

  std::vector<cv::Mat> orb_slam_map_descriptors_vector;
  cv::Mat descriptors_map;
  for (auto& mp : system.GetMap()->GetAllMapPoints()) {
      orb_slam_map_descriptors_vector.push_back(mp->GetDescriptor());
  }

  // Concatenate all the descriptors in descriptors_map
  cv::vconcat(orb_slam_map_descriptors_vector, descriptors_map);

  // Set up ORB feature detector and matcher
  cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2, 8);
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);

  // Process images in the input folder and find the one that matches the most
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
    matcher.knnMatch(descriptors, descriptors_map, matches, 2);

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
