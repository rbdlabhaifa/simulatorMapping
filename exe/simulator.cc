#include <iostream>
#include <vector>
#include <chrono>
#include "include/Auxiliary.h" // include the header file for your function
#include "opencv2/opencv.hpp" // include the OpenCV library
#include "opencv2/viz.hpp" // include the OpenCV Viz module

int main()
{
    // Initialize the data structure
    std::vector<cv::Point3d> points_seen;
    cv::Point3d start_position(0, 0, 0); // Change to your desired starting position
    double yaw = 0, pitch = 0, roll = 0; // Change to your desired starting orientation

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string map_input_dir = data["mapInputDir"];
    const std::string cloud_points = map_input_dir + "cloud1.csv";

    points_seen = Auxiliary::getPointsFromPos(cloud_points, start_position, yaw, pitch, roll);

    // Initialize the Viz window
    cv::viz::Viz3d viz_window("Viz Window");
    viz_window.showWidget("Coordinate System", cv::viz::WCoordinateSystem());
    cv::viz::WCube cube_widget(cv::Point3d(0.5, 0.5, 0.5), cv::Point3d(-0.5, -0.5, -0.5), true, cv::viz::Color::gray());
    viz_window.showWidget("Cube", cube_widget);

    // Define variables to keep track of position and orientation
    cv::Point3d current_position = start_position;
    double current_yaw = yaw, current_pitch = pitch, current_roll = roll;

    // Define variables for time measurement
    auto last_time = std::chrono::high_resolution_clock::now();
    double delta_time;

    while (true) {
        // Wait for user input
        int key = cv::waitKey(1);
        if (key == 27) break; // Exit if the Esc key is pressed

        // Calculate delta time
        auto current_time = std::chrono::high_resolution_clock::now();
        delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() / 1000.0;
        last_time = current_time;

        // Update position and orientation based on user input
        if (key == 'w') {
            current_position.x += delta_time * std::cos(current_yaw) * std::cos(current_pitch);
            current_position.y += delta_time * std::sin(current_yaw) * std::cos(current_pitch);
            current_position.z += delta_time * std::sin(current_pitch);
        }
        else if (key == 's') {
            current_position.x -= delta_time * std::cos(current_yaw) * std::cos(current_pitch);
            current_position.y -= delta_time * std::sin(current_yaw) * std::cos(current_pitch);
            current_position.z -= delta_time * std::sin(current_pitch);
        }
        else if (key == 'd') {
            current_position.x += delta_time * std::cos(current_yaw - M_PI / 2);
            current_position.y += delta_time * std::sin(current_yaw - M_PI / 2);
        }
        else if (key == 'a') {
            current_position.x += delta_time * std::cos(current_yaw + M_PI / 2);
            current_position.y += delta_time * std::sin(current_yaw + M_PI / 2);
        }
        else if (key == 'up') {
            current_pitch += delta_time;
        }
        else if (key == 'down') {
            current_pitch -= delta_time;
        }
        else if (key == 'right') {
            current_yaw -= delta_time;
        }
        else if (key == 'left') {
            current_yaw += delta_time;
        }

        // Update the points seen
        std::vector<cv::Point3d> new_points_seen = Auxiliary::getPointsFromPos(cloud_points, current_position, current_yaw, current_pitch, current_roll);
        points_seen.insert(points_seen.end(), new_points_seen.begin(), new_points_seen.end());

        // Draw the points seen
        cv::viz::WCloud cloud_widget(points_seen, cv::viz::Color::white());
        viz_window.showWidget("Points Seen", cloud_widget);

        // Draw the new points observed
        cv::viz::WCloud new_cloud_widget(new_points_seen, cv::viz::Color::red());
        viz_window.showWidget("New Points Observed", new_cloud_widget);

        // Wait for a short time
        cv::waitKey(10);
    }
}

return 0;
