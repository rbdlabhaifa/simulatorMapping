#include "include/Auxiliary.h"

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

void savePointsToXYZ(const std::string& filePath, std::vector<cv::Point3d> cloud) {
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

cv::Point3d computeCentroid(std::vector<cv::Point3d>& points) {
    // Compute the centroid
    cv::Point3d centroid(0, 0, 0);
    for (const cv::Point3d& p : points) {
        centroid += p;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();

    return centroid;
}

void removeFarthestPoints(std::vector<cv::Point3d>& points, double percent_to_remove) {
    // Compute the centroid
    cv::Point3d centroid = computeCentroid(points);

    // Calculate distance of each point from centroid and store it along with the point
    std::vector<std::pair<double, cv::Point3d>> distanceAndPoints;
    for (const cv::Point3d& p : points) {
        double distance = cv::norm(p - centroid);
        distanceAndPoints.push_back({distance, p});
    }

    // Sort the points based on distance
    std::sort(distanceAndPoints.begin(), distanceAndPoints.end(),
              [](const std::pair<double, cv::Point3d>& a, const std::pair<double, cv::Point3d>& b) {
                  return a.first < b.first; // ascending order
              });

    // Remove the farthest points based on percent_to_remove
    int removeCount = static_cast<int>(points.size() * percent_to_remove / 100.0);
    distanceAndPoints.resize(distanceAndPoints.size() - removeCount);

    // Update original points vector
    points.clear();
    for (const auto& pair : distanceAndPoints) {
        points.push_back(pair.second);
    }
}

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string orbs_csv_dir = data["framesOutput"];

    std::vector <cv::Point3d> cloudPoints1;
    std::vector <cv::Point3d> cloudPoints2;

    std::string cloud_points_combined_frames_filename = orbs_csv_dir + "a1_combined_frames_points.xyz";
    std::string cloud_points_orb_slam_filename = orbs_csv_dir + "a2_orb_slam_map_points.xyz";

    // Read points from the single XYZ file
    cloudPoints1 = readPointsFromXYZ(cloud_points_combined_frames_filename);
    cloudPoints2 = readPointsFromXYZ(cloud_points_orb_slam_filename);

    // Remove outliers
    removeFarthestPoints(cloudPoints1, 1.0);
    removeFarthestPoints(cloudPoints2, 5.0);

    // Save the updated points
    savePointsToXYZ(orbs_csv_dir + "b1_combined_frames_points_without_outliers.xyz", cloudPoints1);
    savePointsToXYZ(orbs_csv_dir + "b2_orb_slam_map_points_without_outliers.xyz", cloudPoints2);
}
