//
// Created by rbdstudent on 17/06/2021.
//

#include "Auxiliary.h"

std::string Auxiliary::GetGeneralSettingsPath()
{
    std::filesystem::path currentDirPath = std::filesystem::current_path();
    std::string settingPath = currentDirPath.string();
    settingPath += "/../generalSettings.json";
    return settingPath;
}

void Auxiliary::drawPoints(std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>> &pointsToDraw)
{
    for (auto &point: pointsToDraw) {
        glPointSize(point.second.first);
        glBegin(GL_POINTS);
        glColor3f((float) (point.second.second.x()), (float) (point.second.second.y()), (float) (point.second.second.z()));
        glVertex3f((float) (point.first.x), (float) (point.first.y), (float) (point.first.z));
        glEnd();
    }
}
