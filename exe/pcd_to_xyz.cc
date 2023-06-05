#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>

#include "include/Auxiliary.h"

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

int main()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string orbs_csv_dir = data["framesOutput"];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    std::string cloud_points_combined_frames_filename = orbs_csv_dir + "e1_combined_frames_points_without_outliers.pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_points_combined_frames_filename, *cloud1);
    savePointsToXYZ(orbs_csv_dir + "f1_combined_frames_points_without_outliers.xyz", *cloud1);

    return 0;
 }
