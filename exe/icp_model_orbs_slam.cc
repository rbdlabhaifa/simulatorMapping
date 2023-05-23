#include <opencv2/opencv.hpp>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include "include/Auxiliary.h"

// Function to read CSV file into a vector of cv::Point3d
std::vector<cv::Point3d> readPointsFromCSV(const std::string& filePath) {
    std::vector<cv::Point3d> points;
    std::ifstream file(filePath);
    std::string line;
    double x, y, z;

    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ',');
        x = std::stod(cell);

        std::getline(lineStream, cell, ',');
        y = std::stod(cell);

        std::getline(lineStream, cell, ',');
        z = std::stod(cell);

        points.emplace_back(x, y, z);
    }

    return points;
}

// Function to read XYZ file into a vector of cv::Point3d
std::vector<cv::Point3d> readPointsFromXYZ(const std::string& filePath) {
    std::vector<cv::Point3d> points;
    std::ifstream file(filePath);
    std::string line;
    double x, y, z;

    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ' ');
        x = std::stod(cell);

        std::getline(lineStream, cell, ' ');
        y = std::stod(cell);

        std::getline(lineStream, cell, ' ');
        z = std::stod(cell);

        points.emplace_back(x, y, z);
    }

    return points;
}

// Function to convert std::vector<cv::Point3d> to pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector<cv::Point3d>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : points) {
        cloud->push_back(pcl::PointXYZ((float)point.x, (float)point.y, (float)point.z));
    }
    return cloud;
}

void saveMatrixToFile(const Eigen::Matrix4f &matrix, const std::string &filename) {
    std::ofstream outfile(filename);

    if (outfile.is_open()) {
        for (int row = 0; row < matrix.rows(); ++row) {
            for (int col = 0; col < matrix.cols(); ++col) {
                outfile << matrix(row, col);
                if (col != matrix.cols() - 1) {
                    outfile << ",";
                }
            }
            outfile << "\n";
        }
        outfile.close();
    } else {
        std::cerr << "Cannot open file: " << filename << std::endl;
    }
}

void savePointsToXYZ(const std::string& filePath, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filePath << std::endl;
        return;
    }

    for (const auto& point : cloud) {
        file << point.x << " " << point.y << " " << point.z << "\n";
    }

    file.close();
}

// Compute scale using centroids and standard deviations
float compute_scale (const pcl::PointCloud<pcl::PointXYZ>::Ptr src, const pcl::PointCloud<pcl::PointXYZ>::Ptr tgt)
{
    Eigen::Vector4f src_centroid, tgt_centroid;
    pcl::compute3DCentroid(*src, src_centroid);
    pcl::compute3DCentroid(*tgt, tgt_centroid);

    float src_avg_dist = 0.0, tgt_avg_dist = 0.0;
    for (const auto& point : *src)
    {
        src_avg_dist += sqrt(pow(point.x - src_centroid[0], 2) +
                             pow(point.y - src_centroid[1], 2) +
                             pow(point.z - src_centroid[2], 2));
    }
    src_avg_dist /= src->size();

    for (const auto& point : *tgt)
    {
        tgt_avg_dist += sqrt(pow(point.x - tgt_centroid[0], 2) +
                             pow(point.y - tgt_centroid[1], 2) +
                             pow(point.z - tgt_centroid[2], 2));
    }
    tgt_avg_dist /= tgt->size();

    float scale = tgt_avg_dist / src_avg_dist;

    return scale;
}

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Load the image
    std::string orbs_csv_dir = data["framesOutput"];

    std::vector<cv::Point3d> cloudPoints1;
    std::vector<cv::Point3d> cloudPoints2;

    std::string cloud_points_combined_frames_filename = orbs_csv_dir + "b1_combined_frames_points_without_outliers.xyz";
    std::string cloud_points_orb_slam_filename = orbs_csv_dir + "b2_orb_slam_map_points_without_outliers.xyz";

    // Read points from the single XYZ file
    cloudPoints1 = readPointsFromXYZ(cloud_points_combined_frames_filename);
    cloudPoints2 = readPointsFromXYZ(cloud_points_orb_slam_filename);

    // Convert std::vector<cv::Point3d> to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = toPointCloud(cloudPoints1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = toPointCloud(cloudPoints2);

    // Compute scale
    float scale = compute_scale(cloud1, cloud2);

    // Scale the source point cloud
    for (size_t i = 0; i < cloud1->points.size(); ++i)
    {
        cloud1->points[i].x *= scale;
        cloud1->points[i].y *= scale;
        cloud1->points[i].z *= scale;
    }

    savePointsToXYZ(orbs_csv_dir + "c1_combined_frames_points_without_outliers.xyz", *cloud1);
    savePointsToXYZ(orbs_csv_dir + "c2_orb_slam_map_points_without_outliers.xyz", *cloud2);

    // Create GICP instance
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloud1);
    gicp.setInputTarget(cloud2);

    // Perform GICP
    pcl::PointCloud<pcl::PointXYZ> Final;
    gicp.align(Final);

    // Check convergence and obtain transformation matrix
    if (gicp.hasConverged()) {
        std::cout << "ICP has converged with score: " << gicp.getFitnessScore() << std::endl;

        Eigen::Matrix4f transformation = gicp.getFinalTransformation();

        std::cout << "Estimated scale factor: " << scale << std::endl;
        std::cout << "ICP Transformation Matrix with scale:\n" << transformation << std::endl;

        // Apply the updated transformation to the input cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*cloud1, transformed_cloud, transformation);

        std::string transformation_matrix_csv_path = std::string(data["framesOutput"]) + "frames_transformation_matrix.csv";
        saveMatrixToFile(transformation, transformation_matrix_csv_path);

        std::string transformed_points_csv_path = std::string(data["framesOutput"]) + "transformed_points.xyz";
        savePointsToXYZ(transformed_points_csv_path, transformed_cloud);


    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    return 0;
 }
