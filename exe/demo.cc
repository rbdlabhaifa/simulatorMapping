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

void applyRightToModelCam(pangolin::OpenGlMatrix &Tcw, double value) {
    // Values are opposite
    Tcw.m[3 * 4 + 0] -= value;
}

void applyUpToModelCam(pangolin::OpenGlMatrix &Tcw, double value) {
    // Values are opposite
    Tcw.m[3 * 4 + 1] -= value;
}

void applyForwardToModelCam(pangolin::OpenGlMatrix &Tcw, double value) {
    // Values are opposite
    Tcw.m[3 * 4 + 2] -= value;
}

void applyYawRotationToModelCam(pangolin::OpenGlMatrix &Tcw, double value) {
    Eigen::Matrix4d Tcw_eigen = pangolin::ToEigen<double>(Tcw);

    // Values are opposite
    double rand = -value * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    // Left-multiply the rotation
    Tcw_eigen = pangolinR * Tcw_eigen;

    // Convert back to pangolin matrix and set
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tcw.m[j * 4 + i] = Tcw_eigen(i, j);
        }
    }
}

void applyPitchRotationToModelCam(pangolin::OpenGlMatrix &Tcw, double value) {
    Eigen::Matrix4d Tcw_eigen = pangolin::ToEigen<double>(Tcw);

    // Values are opposite
    double rand = -value * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, c, -s,
        0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    // Left-multiply the rotation
    Tcw_eigen = pangolinR * Tcw_eigen;

    // Convert back to pangolin matrix and set
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tcw.m[j * 4 + i] = Tcw_eigen(i, j);
        }
    }
}

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

void saveOnlyNewPoints(std::vector<cv::Point3d> &pointsSeen, std::vector<cv::Point3d> &newPointsSeen) {
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

    // Build the starting Tcw matrix from the initializers
    pangolin::OpenGlMatrix Tcw, Twc;
    Twc.SetIdentity();
    
    Tcw.SetIdentity();

    // Opengl has inversed Y axis
    // Assign yaw, pitch and roll rotations and translation
    Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
    Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(currentRoll, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(currentYaw, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(currentPitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Tcw_eigen(0, 3) = currentPosition.x;
    Tcw_eigen(1, 3) = -currentPosition.y;
    Tcw_eigen(2, 3) = currentPosition.z;

    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw.m[j * 4 + i] = Tcw_eigen(i,j);
        }
    }

    std::vector<cv::Point3d> newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
    std::vector<cv::Point3d> pointsSeen = std::vector<cv::Point3d>();

    float movingScale = data["movingScale"];
    float rotateScale = data["rotateScale"];

    pangolin::CreateWindowAndBind("Simulator Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuMoveLeft("menu.Move Left", false, false);
    pangolin::Var<bool> menuMoveRight("menu.Move Right", false, false);
    pangolin::Var<bool> menuMoveDown("menu.Move Down", false, false);
    pangolin::Var<bool> menuMoveUp("menu.Move Up", false, false);
    pangolin::Var<bool> menuMoveBackward("menu.Move Backward", false, false);
    pangolin::Var<bool> menuMoveForward("menu.Move Forward", false, false);
    pangolin::Var<bool> menuRotateLeft("menu.Rotate Left", false, false);
    pangolin::Var<bool> menuRotateRight("menu.Rotate Right", false, false);
    pangolin::Var<bool> menuRotateDown("menu.Rotate Down", false, false);
    pangolin::Var<bool> menuRotateUp("menu.Rotate Up", false, false);
    pangolin::Var<bool> menuFinishScan("menu.Finish Scan", false, false);

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

    while (!menuFinishScan) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (menuFollowCamera && follow) {
            s_cam.Follow(Twc);
        } else if (menuFollowCamera && !follow) {
            s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc);
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

            applyRightToModelCam(Tcw, -movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveLeft = false;
        }

        if (menuMoveRight)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyRightToModelCam(Tcw, movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveRight = false;
        }

        if (menuMoveDown)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            // Opengl has inversed Y axis so we pass -value
            applyUpToModelCam(Tcw, movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveDown = false;
        }

        if (menuMoveUp)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            // Opengl has inversed Y axis so we pass -value
            applyUpToModelCam(Tcw, -movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveUp = false;
        }

        if (menuMoveBackward)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyForwardToModelCam(Tcw, -movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveBackward = false;
        }

        if (menuMoveForward)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyForwardToModelCam(Tcw, movingScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuMoveForward = false;
        }

        if (menuRotateLeft)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyYawRotationToModelCam(Tcw, -rotateScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuRotateLeft = false;
        }

        if (menuRotateRight)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyYawRotationToModelCam(Tcw, rotateScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuRotateRight = false;
        }

        if (menuRotateDown)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyPitchRotationToModelCam(Tcw, -rotateScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

            menuRotateDown = false;
        }

        if (menuRotateUp)
        {
            pointsSeen.insert(pointsSeen.end(), newPointsSeen.begin(), newPointsSeen.end());

            applyPitchRotationToModelCam(Tcw, rotateScale);

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            saveOnlyNewPoints(pointsSeen, newPointsSeen);

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

            // Opengl has inversed Y axis
            // Assign yaw, pitch and roll rotations and translation
            Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
            Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(currentRoll, Eigen::Vector3d::UnitZ()) * 
                                    Eigen::AngleAxisd(currentYaw, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(currentPitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
            Tcw_eigen(0, 3) = currentPosition.x;
            Tcw_eigen(1, 3) = -currentPosition.y;
            Tcw_eigen(2, 3) = currentPosition.z;

            for(int i=0;i<4;i++){
                for(int j=0;j<4;j++){
                    Tcw.m[j * 4 + i] = Tcw_eigen(i,j);
                }
            }

            newPointsSeen = Auxiliary::getPointsFromTcw(cloudPointPath, Tcw, Twc);
            pointsSeen = std::vector<cv::Point3d>();
        }
    }

    std:: cout << pointsSeen.size()+newPointsSeen.size() << std::endl;

    return 0;
}
