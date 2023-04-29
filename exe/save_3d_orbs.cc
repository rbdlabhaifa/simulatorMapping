#include <opencv2/opencv.hpp>
#include <vector>

#include "include/Auxiliary.h"

cv::Mat read_matrix_4d_from_csv(std::string filename)
{
    cv::Mat matrix = cv::Mat(4, 4, CV_64F);

    std::ifstream csv_file(filename);

    if (!csv_file.is_open()) {
        std::cerr << "Error: could not open file '" << filename << "' for reading." << std::endl;
        return matrix;
    }

    // Read matrix elements from file
    std::string line;
    int row = 0;
    while (std::getline(csv_file, line) && row < 4) {
        std::stringstream line_stream(line);
        std::string cell;
        int col = 0;
        while (std::getline(line_stream, cell, ',') && col < 4) {
            matrix.row(row).col(col) = std::stod(cell);
            col++;
        }
        row++;
    }

    csv_file.close();

    std::cout << matrix << std::endl;
    return matrix;
}

// cv::Mat to Eigen::Matrix4d conversion function
Eigen::Matrix4d cvMatToEigen(const cv::Mat& mat) {
  // Create a 4x4 matrix
  Eigen::Matrix4d mat_eigen;

  // Check if the input matrix is of size 4x4 and has the correct type
  CV_Assert(mat.rows == 4 && mat.cols == 4 && mat.type() == CV_64FC1);

  // Copy the values from cv::Mat to Eigen::Matrix4d
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat_eigen(i, j) = mat.at<double>(i, j);
    }
  }

  return mat_eigen;
}

std::vector<float> read_depth_buffer(const std::string& filename, int width, int height) {
    std::vector<float> depthBuffer(width * height);

    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile) {
        std::cerr << "Error: Unable to open depth buffer file." << std::endl;
        return depthBuffer;
    }

    inFile.read(reinterpret_cast<char*>(depthBuffer.data()), width * height * sizeof(float));
    inFile.close();

    return depthBuffer;
}

void loadCameraParameters(const std::string& filename, cv::Mat& K, cv::Mat& R, cv::Mat& t) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["K"] >> K;
    fs["R"] >> R;
    fs["t"] >> t;
    fs.release();
}

cv::Mat loadDepthBuffer(const std::string& filename) {
    // Load the depth buffer as a single-channel image with the original depth
    cv::Mat depth = cv::imread(filename, cv::IMREAD_ANYDEPTH);
    // Flip the depth buffer back to the original orientation
    cv::flip(depth, depth, 0);

    // Convert the depth buffer back to floating-point values using the scaling factor
    cv::Mat depth_float;
    depth.convertTo(depth_float, CV_32FC1, 1.0 / 1000.0);

    // Normalize the depth buffer to a range of 0 to 255
    double minVal, maxVal;
    cv::minMaxIdx(depth_float, &minVal, &maxVal);
    cv::Mat depth_normalized;
    depth_float.convertTo(depth_normalized, CV_8UC1, 255 / (maxVal - minVal), -minVal * 255 / (maxVal - minVal));

    // Apply a colormap to the depth image for visualization
    cv::Mat depth_colored;
    cv::applyColorMap(depth_normalized, depth_colored, cv::COLORMAP_JET);

    // Save the depth buffer as a colored image
    cv::imwrite("/home/liam/a.png", depth_colored);

    return depth_float;
}

cv::Point3f convert2Dto3D(cv::Point2f keypoint, const cv::Mat& K, const cv::Mat& depth) {
    cv::Point3f point3D;

    float z = depth.at<float>(static_cast<int>(keypoint.y), static_cast<int>(keypoint.x));
    if (z > 0) {
        float x = (keypoint.x - K.at<float>(0, 2)) * z / K.at<float>(0, 0);
        float y = (keypoint.y - K.at<float>(1, 2)) * z / K.at<float>(1, 1);
        point3D = cv::Point3d(x, y, z);
    }

    return point3D;
}

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);

    // Load the image
    int frame_to_check = data["frameNumber"];
    std::string filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + ".png";
    cv::Mat frame = cv::imread(filename);

    // Read matrices
    std::string mv_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_mv.csv";
    cv::Mat mv_mat = read_matrix_4d_from_csv(mv_filename);
    std::string proj_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_proj.csv";
    cv::Mat proj_mat = read_matrix_4d_from_csv(proj_filename);

    // Create an ORB detector
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(frame, keypoints);

    // Draw keypoints on the image
    cv::Mat image_keypoints;
    cv::drawKeypoints(frame, keypoints, image_keypoints);

    // Display the image with keypoints
    cv::imshow("Keypoints", image_keypoints);
    cv::waitKey(0);

    // Save the x and y values of the keypoints to a vector
    std::vector<cv::Point2f> keypoint_positions;
    for (const auto& keypoint : keypoints)
    {
        cv::Point2f position = cv::Point2f((float)keypoint.pt.x, (float)keypoint.pt.y);
        keypoint_positions.push_back(position);
    }

    std::string depth_buffer_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_depth_buffer.png";
    cv::Mat depth_buffer = loadDepthBuffer(depth_buffer_filename);
    std::string camera_params_yml= std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_camera_params.yml";
    cv::Mat K, R, t;
    loadCameraParameters(camera_params_yml, K, R, t);

    // Convert keypoints pixels to keypoints 3d points
    std::vector<cv::Point3f> keypoint_points;
    for (const auto& keypoint : keypoint_positions)
    {
        cv::Point3f point = convert2Dto3D(keypoint, K, depth_buffer);
        keypoint_points.push_back(point);
    }

    std::string csv_orbs_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_orbs.csv";
    std::ofstream csv_orbs_file(csv_orbs_filename);

    for (const auto& point : keypoint_points) {
        csv_orbs_file << point.x << "," << point.y << "," << point.z << std::endl;
        std::cout << point.x << "," << point.y << "," << point.z << std::endl;
    }

    csv_orbs_file.close();

    return 0;
 }
