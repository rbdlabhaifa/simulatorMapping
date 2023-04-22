#include <opencv2/opencv.hpp>
#include <vector>

#include "include/Auxiliary.h"

Eigen::Matrix4d read_matrix_4d_from_csv(std::string filename)
{
    Eigen::Matrix4d matrix;

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
            matrix(row, col) = std::stod(cell);
            col++;
        }
        row++;
    }

    csv_file.close();

    std::cout << matrix << std::endl;
    return matrix;
}

Eigen::Vector3d GetPointFromPixel(Eigen::Matrix4d mv_mat, Eigen::Matrix4d proj_mat, cv::Point2f keypoint, double width, double height) {
    // Compute the inverse of the modelview and projection matrices
    Eigen::Matrix4d mvp = proj_mat * mv_mat;
    Eigen::Matrix4d mvp_inv = mvp.inverse();

    // Compute the normalized device coordinates of the point
    Eigen::Vector4d ndc;
    ndc << (2.0 * (double)(keypoint.x)) / width - 1.0,
            1.0 - (2.0 * (double)(keypoint.y)) / height,
            0.0,
            1.0;

    // Unproject the point to world coordinates
    Eigen::Vector4d world = mvp_inv * ndc;
    Eigen::Vector3d point = world.head<3>() / world[3];

    // Convert the point to camera coordinates
    Eigen::Matrix4d modelview_inv = mv_mat.inverse();
    Eigen::Vector4d camera_coord = modelview_inv * Eigen::Vector4d(point[0], point[1], point[2], 1.0);

    // Extract the x, y, and z coordinates of the point in camera coordinates
    double x = camera_coord[0] / camera_coord[3];
    double y = camera_coord[1] / camera_coord[3];
    double z = camera_coord[2] / camera_coord[3];

    return Eigen::Vector3d(x, y, z);
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
    Eigen::Matrix4d mv_mat = read_matrix_4d_from_csv(mv_filename);
    std::string proj_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_proj.csv";
    Eigen::Matrix4d proj_mat = read_matrix_4d_from_csv(proj_filename);

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
        cv::Point2f position = keypoint.pt;
        keypoint_positions.push_back(position);
    }

    // Convert keypoints pixels to keypoints 3d points
    std::vector<Eigen::Vector3d> keypoint_points;
    for (const auto& keypoint : keypoint_positions)
    {
        Eigen::Vector3d point = GetPointFromPixel(mv_mat, proj_mat, keypoint, fSettings["Camera.width"], fSettings["Camera.height"]);
        keypoint_points.push_back(point);
    }

    std::string csv_orbs_filename = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_orbs.csv";
    std::ofstream csv_orbs_file(csv_orbs_filename);

    for (const auto& point : keypoint_points) {
        csv_orbs_file << point.x() << "," << point.y() << "," << point.z() << std::endl;
        std::cout << point.x() << "," << point.y() << "," << point.z() << std::endl;
    }

    csv_orbs_file.close();

    return 0;
 }
