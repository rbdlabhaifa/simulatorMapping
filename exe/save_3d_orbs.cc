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

cv::Point3d GetPointFromPixel(cv::Mat mv_mat, cv::Mat proj_mat, cv::Point2f keypoint, double width, double height) {
    // Compute the inverse of the modelview and projection matrices
    cv::Mat mvp_inv = (proj_mat * mv_mat).inv();

    // Compute the normalized device coordinates of the point
    cv::Vec4d ndc;
    ndc << (2.0 * (double)(keypoint.x)) / width - 1.0,
            1.0 - (2.0 * (double)(keypoint.y)) / height,
            0.0,
            1.0;

    // Unproject the point to world coordinates
    cv::Mat world = mvp_inv * ndc;
    cv::Mat point = world.row(0).colRange(0, 3) / world.row(0).col(3);

    // Convert the point to camera coordinates
    cv::Mat modelview_inv = mv_mat.inv();
    cv::Mat camera_coord = modelview_inv * cv::Mat(point.row(0), 1.0);

    // Extract the x, y, and z coordinates of the point in camera coordinates
    cv::Point3d point3d(camera_coord.at<double>(0, 0) / camera_coord.at<double>(3, 0),
                        camera_coord.at<double>(1, 0) / camera_coord.at<double>(3, 0),
                        camera_coord.at<double>(2, 0) / camera_coord.at<double>(3, 0));

    return point3d;
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

cv::Point3d ThreeDReconstruction(cv::Mat mv_mat, cv::Mat proj_mat, cv::Point2d keypoint, double width, double height, std::vector<float> depth_buffer) {
    // Create a point to store the 3D coordinates of the detected orb
    Eigen::Vector3d orb_3d;

    // Convert to eigen
    Eigen::Matrix4d eigen_proj_mat = cvMatToEigen(proj_mat);
    Eigen::Matrix4d eigen_mv_mat = cvMatToEigen(mv_mat);

    int x = static_cast<int>(std::round(keypoint.x));
    int y = static_cast<int>(std::round(keypoint.y));

    // Get the depth value (z-buffer value) associated with the orb's 2D image coordinates
    const double depth_value = depth_buffer[y * (int)width + x];

    // Compute the ModelViewProjection matrix
    Eigen::Matrix4d modelViewProjectionMatrix = eigen_proj_mat * eigen_mv_mat;

    // Compute the inverse ModelViewProjection matrix
    Eigen::Matrix4d inverseModelViewProjectionMatrix = modelViewProjectionMatrix.inverse();

    // Unproject the ORB feature to world coordinates
    Eigen::Vector4d homogenousPixel(x, y, depth_value, 1.0f);
    Eigen::Vector4d worldCoords = inverseModelViewProjectionMatrix * homogenousPixel;

    // Convert to Euclidean coordinates and add to the list of 3D points
    cv::Point3d point3D(worldCoords[0] / worldCoords[3], worldCoords[1] / worldCoords[3], worldCoords[2] / worldCoords[3]);

    // Add the 3D coordinates of the orb to the list of 3D points representing the detected orbs in the scene
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
    std::vector<cv::Point2d> keypoint_positions;
    for (const auto& keypoint : keypoints)
    {
        cv::Point2d position = cv::Point2d((double)keypoint.pt.x, (double)keypoint.pt.y);
        keypoint_positions.push_back(position);
    }

    // Get Zbuffer
    std::string ZBufferStr = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_depth.bin";
    std::vector<float> depth_buffer = read_depth_buffer(ZBufferStr, (int)fSettings["Camera.width"], (int)fSettings["Camera.height"]);

    // Convert keypoints pixels to keypoints 3d points
    std::vector<cv::Point3d> keypoint_points;
    for (const auto& keypoint : keypoint_positions)
    {
        cv::Point3d point = ThreeDReconstruction(mv_mat, proj_mat, keypoint, fSettings["Camera.width"], fSettings["Camera.height"], depth_buffer);
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
