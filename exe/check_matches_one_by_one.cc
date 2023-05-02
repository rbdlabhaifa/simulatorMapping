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

  std::string map_input_dir = data["mapInputDir"];
  std::string frames_folder = data["framesFolder"];

  // Set up ORB feature detector and matcher
  cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2, 8);
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);

  // Process images in the input folder and find the one that matches the most
  std::string first_image_path = frames_folder + "wall2_1.png";
  std::string second_image_path = map_input_dir + "frame_719.png";

  // Load First Image
  cv::Mat first_image = cv::imread(first_image_path, cv::IMREAD_GRAYSCALE);

  // Load First Image
  cv::Mat second_image = cv::imread(second_image_path, cv::IMREAD_GRAYSCALE);

  // Extract ORB keypoints and descriptors from the images
  std::vector<cv::KeyPoint> key_points_first_image;
  std::vector<cv::KeyPoint> key_points_second_image;
  cv::Mat descriptors_first_image;
  cv::Mat descriptors_second_image;
  orb->detectAndCompute(first_image, cv::Mat(), key_points_first_image, descriptors_first_image);
  orb->detectAndCompute(second_image, cv::Mat(), key_points_second_image, descriptors_second_image);

  std::vector<std::vector<cv::DMatch>> matches;
  matcher.knnMatch(descriptors_first_image, descriptors_second_image, matches, 2);

  // Find good matches
  std::vector<cv::DMatch> good_matches;
  for (auto& match: matches) {
      if (match[0].distance < 0.7 * match[1].distance) {
          good_matches.push_back(match[0]);
      }
  }

  cv::Mat final_image;
  cv::drawMatches(first_image, key_points_first_image, second_image, key_points_second_image, good_matches, final_image);

  cv::resize(final_image, final_image, cv::Size(1920, 1080), cv::INTER_LINEAR);
  cv::imwrite("/home/liam/matching_texture.png", final_image);

  cv::imshow("Matches", final_image);
  cv::waitKey(0);

  return 0;
}
