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

#include "include/simulator.h"

void handleKeyboardEvents(Simulator *simulator) {
    pangolin::RegisterKeyPressCallback('l', [&simulator]() { simulator->ToggleFollowCamera(); });
    pangolin::RegisterKeyPressCallback('\t', [&simulator]() { simulator->ToggleShowPoints(); });
    pangolin::RegisterKeyPressCallback('i', [&simulator]() { simulator->DoReset(); });
    pangolin::RegisterKeyPressCallback('a', [&simulator]() { simulator->MoveLeft(); });
    pangolin::RegisterKeyPressCallback('d', [&simulator]() { simulator->MoveRight(); });
    pangolin::RegisterKeyPressCallback('f', [&simulator]() { simulator->MoveDown(); });
    pangolin::RegisterKeyPressCallback('r', [&simulator]() { simulator->MoveUp(); });
    pangolin::RegisterKeyPressCallback('s', [&simulator]() { simulator->MoveBackward(); });
    pangolin::RegisterKeyPressCallback('w', [&simulator]() { simulator->MoveForward(); });
    pangolin::RegisterKeyPressCallback('q', [&simulator]() { simulator->RotateLeft(); });
    pangolin::RegisterKeyPressCallback('e', [&simulator]() { simulator->RotateRight(); });
    pangolin::RegisterKeyPressCallback('t', [&simulator]() { simulator->RotateDown(); });
    pangolin::RegisterKeyPressCallback('g', [&simulator]() { simulator->RotateUp(); });
    pangolin::RegisterKeyPressCallback('S', [&simulator]() { simulator->FinishScan(); });
}

int main(int argc, char **argv) {
    Simulator simulator = Simulator();

    handleKeyboardEvents(&simulator);
    simulator.Run();

    // Process cloud point generated
    std::vector<cv::Point3d> cloudPoints = simulator.GetCloudPoint();
    std::cout << "Number of points: " << cloudPoints.size() << std::endl;

    return 0;
}
