#include <opencv2/opencv.hpp>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
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

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Load the image
    std::string orbs_csv_dir = data["framesOutput"];
    std::string cloud_point_filename = std::string(data["mapInputDir"]) + "cloud1.csv";

    std::vector<cv::Point3d> cloudPoints1;
    std::vector<cv::Point3d> cloudPoints2;

    // Read points from multiple CSV files in the directory
    for (const auto& entry : std::filesystem::directory_iterator(orbs_csv_dir)) {
        if (entry.path().extension() == ".csv") {
            std::vector<cv::Point3d> points = readPointsFromCSV(entry.path());
            cloudPoints1.insert(cloudPoints1.end(), points.begin(), points.end());
        }
    }

    // Read points from the single CSV file
    cloudPoints2 = readPointsFromCSV(cloud_point_filename);

    // Convert std::vector<cv::Point3d> to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = toPointCloud(cloudPoints1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = toPointCloud(cloudPoints2);

    // Create ICP instance
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);

    // Perform ICP
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // Check convergence and obtain transformation matrix
    if (icp.hasConverged()) {
        std::cout << "ICP has converged with score: " << icp.getFitnessScore() << std::endl;

        // Estimate scale factors using eigenvalues of rotation matrix
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(transformation.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3f scale_factors = svd.singularValues();

        // Apply scale factors to transformation matrix
        transformation.block<3, 3>(0, 0) = transformation.block<3, 3>(0, 0) * scale_factors.asDiagonal();

        std::cout << "Estimated scale factors: " << scale_factors.transpose() << std::endl;
        std::cout << "ICP Transformation Matrix with scale:\n" << transformation << std::endl;

        // Apply the updated transformation to the input cloud
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*cloud1, transformed_cloud, transformation);
        std::string transformation_matrix_csv_path = std::string(data["framesOutput"]) + "frames_transformation_matrix.csv";
        saveMatrixToFile(transformation, transformation_matrix_csv_path);
    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    return 0;
 }
