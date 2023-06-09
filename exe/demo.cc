/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <pangolin/pangolin.h>

#include "include/Auxiliary.h"

void drawMapPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen, float pointSize)
{
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : seen_points)
    {
        glVertex3f((float)point.x, (float)point.y, (float)point.z);
    }
    glEnd();

    glPointSize(pointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(auto point : new_points_seen)
    {
        glVertex3f((float)point.x, (float)point.y, (float)point.z);

    }
    glEnd();
}

int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);

    float viewpointX = fSettings["Viewer.ViewpointX"];
    float viewpointY = fSettings["Viewer.ViewpointY"];
    float viewpointZ = fSettings["Viewer.ViewpointZ"];
    float viewpointF = fSettings["Viewer.ViewpointF"];
    // ToDo
    bool isPangolinExists = false;

    std::string map_input_dir = data["mapInputDir"];
    std::string cloudPointPath = map_input_dir + "cloud1.csv";

    double startPointX = data["startingCameraPosX"];
    double startPointY = data["startingCameraPosY"];
    double startPointZ = data["startingCameraPosZ"];

    cv::Point3d currentPosition = cv::Point3d(startPointX, startPointY, startPointZ);
    float currentYaw = data["yawRad"];
    float currentPitch = data["pitchRad"];
    float currentRoll = data["rollRad"];

    float pointSize = fSettings["Viewer.PointSize"];;

    pangolin::OpenGlMatrix Twc_opengl;
    Twc_opengl.SetIdentity();

    cv::Mat Twc;
    std::vector<cv::Point3d> newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
    std::vector<cv::Point3d> pointsSeen = std::vector<cv::Point3d>();

    float movingScale = data["movingScale"];
    float rotateScale = data["rotateScale"];

    // ToDo
    if (isPangolinExists) {
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    } else {
        pangolin::CreateWindowAndBind("Simulator Viewer", 1024, 768);
    }

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (isPangolinExists) {
        pangolin::Panel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    } else {
        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    }
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuMoveLeft("menu.Move Left", false, false);
    pangolin::Var<bool> menuMoveRight("menu.Move Right", false, false);
    pangolin::Var<bool> menuMoveDown("menu.Move Down", false, false);
    pangolin::Var<bool> menuMoveUp("menu.Move Up", false, false);
    pangolin::Var<bool> menuRotateLeft("menu.Rotate Left", false, false);
    pangolin::Var<bool> menuRotateRight("menu.Rotate Right", false, false);
    pangolin::Var<bool> menuRotateDown("menu.Rotate Down", false, false);
    pangolin::Var<bool> menuRotateUp("menu.Rotate Up", false, false);
    pangolin::Var<bool> menuShutDown("menu.ShutDown", false, false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, viewpointF, viewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    bool follow = true;

    while (!menuShutDown) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // ToDo
        Twc_opengl.m[0] = (float)Twc.at<double>(0);
        Twc_opengl.m[1] = (float)Twc.at<double>(1);
        Twc_opengl.m[2] = (float)Twc.at<double>(2);
        Twc_opengl.m[4] = (float)Twc.at<double>(4);
        Twc_opengl.m[5] = (float)Twc.at<double>(5);
        Twc_opengl.m[6] = (float)Twc.at<double>(6);
        Twc_opengl.m[8] = (float)Twc.at<double>(8);
        Twc_opengl.m[9] = (float)Twc.at<double>(9);
        Twc_opengl.m[10] = (float)Twc.at<double>(10);
        Twc_opengl.m[12] = (float)Twc.at<double>(12);
        Twc_opengl.m[13] = (float)Twc.at<double>(13);
        Twc_opengl.m[14] = (float)Twc.at<double>(14);

        if (menuFollowCamera && follow) {
            s_cam.Follow(Twc_opengl);
        } else if (menuFollowCamera && !follow) {
            s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc_opengl);
            follow = true;
        } else if (!menuFollowCamera && follow) {
            follow = false;
        }

        d_cam.Activate(s_cam);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (menuShowPoints) {
            drawMapPoints(pointsSeen, newPointsSeen, pointSize);
        }

        pangolin::FinishFrame();

        if (menuMoveLeft)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPosition.x -= movingScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuMoveLeft = false;
        }

        if (menuMoveRight)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPosition.x += movingScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuMoveRight = false;
        }

        if (menuMoveDown)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPosition.y -= movingScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuMoveDown = false;
        }

        if (menuMoveUp)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPosition.y += movingScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuMoveUp = false;
        }

        if (menuRotateLeft)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentYaw -= rotateScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::cout << "Current Pos: " << currentPosition << ", yaw: " << currentYaw << ", pitch: " << currentPitch << ", roll: " << currentRoll << std::endl;
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuRotateLeft = false;
        }

        if (menuRotateRight)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentYaw += rotateScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuRotateRight = false;
        }

        if (menuRotateDown)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPitch -= rotateScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuRotateDown = false;
        }

        if (menuRotateUp)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            currentPitch += rotateScale;

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            std::vector<cv::Point3d>::iterator it;
            for (it = newPointsSeen.begin(); it != newPointsSeen.end();)
            {
                if (std::find(pointsSeen.begin(), pointsSeen.end(), *it) != pointsSeen.end())
                {
                    it = newPointsSeen.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            menuRotateUp = false;
        }

        if (menuReset) {
            menuShowPoints = true;
            follow = true;
            menuFollowCamera = true;
            menuReset = false;

            currentPosition = cv::Point3d(startPointX, startPointY, startPointZ);
            currentYaw = data["yawRad"];
            currentPitch = data["pitchRad"];
            currentRoll = data["rollRad"];

            newPointsSeen = Auxiliary::getPointsFromPos(cloudPointPath, currentPosition, currentYaw, currentPitch, currentRoll, Twc);
            pointsSeen = std::vector<cv::Point3d>();
        }
    }

    return 0;
}
