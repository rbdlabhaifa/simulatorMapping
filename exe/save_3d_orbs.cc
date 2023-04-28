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

cv::Point3d ThreeDReconstruction(cv::Mat mv_mat, cv::Mat proj_mat, cv::Point2f keypoint, double width, double height, cv::Point2d orb) {
    cv::Mat invProjMat = proj_mat.inv();
    cv::Mat invModelViewMat = mv_mat.inv();

    // Convert 2D orb point to NDC
    cv::Point3d ndcPt(orb.x / width * 2.0f - 1.0f, 1.0f - orb.y / height * 2.0f, 0.0f);
    cv::Mat ndcMat = (cv::Mat_<float>(4, 1) << ndcPt.x, ndcPt.y, ndcPt.z, 1.0f);

    // Convert NDC to view coordinates
    cv::Mat viewMat = invProjMat * ndcMat;
    viewMat /= viewMat.at<float>(3);
    viewMat = invModelViewMat * viewMat;
    cv::Point3d viewPt(viewMat.at<float>(0), viewMat.at<float>(1), viewMat.at<float>(2));

    // Cast ray from camera position through view coordinate and find intersection point with model
    cv::Point3d camPos(0.0f, 0.0f, 0.0f);
    cv::Point3d dirVec(viewPt.x - camPos.x, viewPt.y - camPos.y, viewPt.z - camPos.z);
    float maxDist = std::numeric_limits<float>::max(); // Maximum distance to check for intersection
    float minDist = std::numeric_limits<float>::epsilon(); // Minimum distance to check for intersection

    // Check intersection
    cv::Point3d end_point = camPos + dirVec * maxDist;
    cv::Point3d ray = end_point - camPos;

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

    // Convert keypoints pixels to keypoints 3d points
    std::vector<cv::Point3d> keypoint_points;
    for (const auto& keypoint : keypoint_positions)
    {
        cv::Point3d point = ThreeDReconstruction(mv_mat, proj_mat, keypoint, fSettings["Camera.width"], fSettings["Camera.height"], keypoint);
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
