#include <opencv2/opencv.hpp>
#include <termios.h> // for getch()
#include <iostream>

#include "include/Auxiliary.h"

int main() {
     // Initialize the data structure
    std::vector<cv::Point3d> points_seen;
    cv::Point3d start_position(0, 0, 0); // Change to your desired starting position
    double yaw = 0, pitch = 0, roll = 0; // Change to your desired starting orientation

    termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string map_input_dir = data["mapInputDir"];
    const std::string cloud_points = map_input_dir + "cloud1.csv";

    points_seen = Auxiliary::getPointsFromPos(cloud_points, start_position, yaw, pitch, roll);

    cv::Point3d current_position = start_position;
    double current_yaw = yaw, current_pitch = pitch, current_roll = roll;

    // Wait for user input and update position/orientation
    char ch = '\0';
    do {
        // Update the points seen
        std::vector<cv::Point3d> new_points_seen = Auxiliary::getPointsFromPos(cloud_points, current_position, current_yaw, current_pitch, current_roll);

        std::vector<cv::Point3d>::iterator it;
        for (it = new_points_seen.begin(); it != new_points_seen.end();)
        {
            if (std::find(points_seen.begin(), points_seen.end(), *it) != points_seen.end())
            {
                it = new_points_seen.erase(it);
            }
            else
            {
                ++it;
            }
        }


        std::cout << "new: " << new_points_seen.size() << std::endl;
        points_seen.insert(points_seen.end(), new_points_seen.begin(), new_points_seen.end());
        std::cout << "total: " << points_seen.size() << std::endl;
        // Wait for user input

        // Print current position and orientation
        std::cout << "Position: (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ")" << std::endl;
        std::cout << "Yaw: " << current_yaw << ", Pitch: " << current_pitch << ", Roll: " << current_roll << std::endl;

        ch = std::cin.get();

        // Update position/orientation based on input
        switch(ch) {
            case 'a':
                current_yaw -= 0.05;
                break;
            case 'd':
                current_yaw += 0.05;
                break;
            case 'w':
                current_pitch += 0.05;
                break;
            case 's':
                current_pitch -= 0.05;
                break;
            case 'i': // up arrow
                current_position.y += 0.1 * cos(current_pitch) * cos(current_yaw);
                current_position.x -= 0.1 * cos(current_pitch) * sin(current_yaw);
                current_position.z -= 0.1 * sin(current_pitch);
                break;
            case 'k': // down arrow
                current_position.y -= 0.1 * cos(current_pitch) * cos(current_yaw);
                current_position.x += 0.1 * cos(current_pitch) * sin(current_yaw);
                current_position.z += 0.1 * sin(current_pitch);
                break;
            case 'j': // left arrow
                current_position.x += 0.1 * cos(current_yaw);
                current_position.y += 0.1 * sin(current_yaw);
                break;
            case 'l': // right arrow
                current_position.x -= 0.1 * cos(current_yaw);
                current_position.y -= 0.1 * sin(current_yaw);
                break;
        }

    } while(ch != 27); // 27 is the ASCII code for escape

    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);

    return 0;
}
