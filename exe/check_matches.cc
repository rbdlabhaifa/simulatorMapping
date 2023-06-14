#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include "System.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"

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
  ORB_SLAM2::System system(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, true, true, map_input_dir + "simulatorMap.bin", true, false);
  std::vector<cv::Mat> orb_slam_kf_descriptors_vector;
  std::vector<std::vector<cv::KeyPoint>> orb_slam_kf_keypoints_vector;
  std::vector<cv::Mat> orb_slam_kf_image_vector;
  for (auto& KF : system.GetMap()->GetAllKeyFrames()) {
      if (KF->image.empty())
          continue;
      orb_slam_kf_image_vector.emplace_back(KF->image);
      orb_slam_kf_descriptors_vector.emplace_back(KF->mDescriptors);
      orb_slam_kf_keypoints_vector.emplace_back(KF->mvKeys);
  }

  cv::FileStorage fSettings(droneYamlPathSlam, cv::FileStorage::READ);
  int nFeatures = fSettings["ORBextractor.nFeatures"];
  float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
  int nLevels = fSettings["ORBextractor.nLevels"];
  int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
  int fMinThFAST = fSettings["ORBextractor.minThFAST"];

  ORB_SLAM2::ORBextractor* mpORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

  // Set up ORB feature detector and matcher
  // cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2, 8);
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);

  // Process images in the input folder and find the one that matches the most
  std::string input_folder_path = frames_folder;
    std::vector<std::string> image_paths;
    std::vector<cv::Mat> images;
  std::vector<std::vector<cv::KeyPoint>> keypoints_images;
  std::vector<cv::Mat> descriptors_images;
    std::vector<std::vector<std::vector<cv::DMatch>>> scores;
    std::vector<std::vector<std::vector<std::vector<cv::DMatch>>>> all_matches;
  std::vector<std::string> textures;
  for (const auto& entry : std::filesystem::directory_iterator(input_folder_path)) {
      textures.emplace_back(entry.path().string());
  }
  for (int i = 0; i < textures.size(); i++) {
    // Load image
    std::cout << textures[i] << std::endl;
    cv::Mat img = cv::imread(textures[i], cv::IMREAD_GRAYSCALE);

    // Extract ORB keypoints and descriptors from the image
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    (*mpORBextractorLeft)(img,cv::Mat(),keypoints, descriptors);
    // orb->detectAndCompute(img, cv::Mat(), keypoints, descriptors);

    keypoints_images.emplace_back(keypoints);
    image_paths.push_back(textures[i]);
    images.push_back(img);
    descriptors_images.push_back(descriptors);

    scores.emplace_back();
    all_matches.emplace_back();
    for (int j = 0; j < orb_slam_kf_descriptors_vector.size(); j++)
    {
        // Match descriptors with the map
        std::vector<std::vector<cv::DMatch>> matches;
        matcher.knnMatch(descriptors, orb_slam_kf_descriptors_vector[i], matches, 2);

        // Find good matches
        std::vector<cv::DMatch> good_matches;
        for (auto& match: matches) {
            if (match[0].distance < 0.7 * match[1].distance) {
                good_matches.push_back(match[0]);
            }
        }

        all_matches[i].emplace_back(std::vector<std::vector<cv::DMatch>>{good_matches});

        // Save the image path and descriptors
        scores[i].push_back(good_matches);
    }
  }

  // Find the image with the highest similarity score
  int best_index_i = 0;
  int best_index_j = 0;
  int best_score = 0;
  for (int i = 0; i < image_paths.size(); ++i) {
      for (int j = 0; j < orb_slam_kf_descriptors_vector.size(); j++)
      {
          if (scores[i][j].size() > best_score) {
              best_index_i = i;
              best_index_j = j;
              best_score = scores[i][j].size();
          }
      }
  }

  std::cout << "Best Image: " << image_paths[best_index_i] << " with keyframe:" << best_index_j << ", Score: " << best_score << std::endl;

  std::cout << "image_paths size: " << image_paths.size() << std::endl;
  std::cout << "keypoints_images size: " << keypoints_images.size() << std::endl;
  std::cout << "orb_slam_kf_image_vector size: " << orb_slam_kf_image_vector.size() << std::endl;
  std::cout << "orb_slam_kf_keypoints_vector size: " << orb_slam_kf_keypoints_vector.size() << std::endl;
  std::cout << "all_matches size: " << all_matches.size() << std::endl;
  std::cout << "all_matches[best_index_i] size: " << all_matches[best_index_i].size() << std::endl;
  std::cout << "all_matches[best_index_i][best_index_j] size: " << all_matches[best_index_i][best_index_j].size() << std::endl;

  if (image_paths.empty() || keypoints_images.empty() || orb_slam_kf_image_vector.empty() || orb_slam_kf_keypoints_vector.empty() || all_matches.empty() || all_matches[best_index_i].empty() || all_matches[best_index_i][best_index_j].empty()) {
      std::cerr << "Error: empty vectors" << std::endl;
      return -1;
  }

  cv::Mat final_image;
  cv::drawMatches(images[best_index_i], keypoints_images[best_index_i], orb_slam_kf_image_vector[best_index_j], orb_slam_kf_keypoints_vector[best_index_j], all_matches[best_index_i][best_index_j], final_image);
  cv::imwrite("/home/liam/Downloads/a/best_matching_texture" + std::to_string(best_index_i) + "_frame" + std::to_string(best_index_j) + ".png", final_image);

  for (int i = 0; i < images.size(); i++)
  {
      for (int j=0; j < orb_slam_kf_image_vector.size(); j++)
      {
          cv::drawMatches(images[i], keypoints_images[i], orb_slam_kf_image_vector[j], orb_slam_kf_keypoints_vector[j], all_matches[i][j], final_image);
          cv::imwrite("/home/liam/Downloads/a/matching_texture" + std::to_string(i) + "_frame" + std::to_string(j) + ".png", final_image);
      }
  }

  return 0;
}
