#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>

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

void savePointsToXYZ(const std::string& filePath, const std::vector<cv::Point3d>& points) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filePath << std::endl;
        return;
    }

    for (const auto& point : points) {
        file << point.x << " " << point.y << " " << point.z << "\n";
    }

    file.close();
}

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string orbs_csv_dir = data["framesOutput"];

    std::vector<cv::Point3d> points = readPointsFromCSV(orbs_csv_dir + "cloud0.csv");
    
    savePointsToXYZ(orbs_csv_dir + "cloud0.xyz", points);

    return 0;
 }
