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

#include "include/MapControl.h"

void handleKeyboardEventsScan(MapControl *mapControl) {
    pangolin::RegisterKeyPressCallback('l', [&mapControl]() { mapControl->ToggleFollowCamera(); });
    pangolin::RegisterKeyPressCallback('\t', [&mapControl]() { mapControl->ToggleShowPoints(); });
    pangolin::RegisterKeyPressCallback('i', [&mapControl]() { mapControl->DoReset(); });
    pangolin::RegisterKeyPressCallback('a', [&mapControl]() { mapControl->MoveLeft(); });
    pangolin::RegisterKeyPressCallback('d', [&mapControl]() { mapControl->MoveRight(); });
    pangolin::RegisterKeyPressCallback('f', [&mapControl]() { mapControl->MoveDown(); });
    pangolin::RegisterKeyPressCallback('r', [&mapControl]() { mapControl->MoveUp(); });
    pangolin::RegisterKeyPressCallback('s', [&mapControl]() { mapControl->MoveBackward(); });
    pangolin::RegisterKeyPressCallback('w', [&mapControl]() { mapControl->MoveForward(); });
    pangolin::RegisterKeyPressCallback('q', [&mapControl]() { mapControl->RotateLeft(); });
    pangolin::RegisterKeyPressCallback('e', [&mapControl]() { mapControl->RotateRight(); });
    pangolin::RegisterKeyPressCallback('t', [&mapControl]() { mapControl->RotateDown(); });
    pangolin::RegisterKeyPressCallback('g', [&mapControl]() { mapControl->RotateUp(); });
    pangolin::RegisterKeyPressCallback('S', [&mapControl]() { mapControl->FinishScan(); });
}

void handleKeyboardEventsResult(MapControl *mapControl) {
    pangolin::RegisterKeyPressCallback('S', [&mapControl]() { mapControl->FinishScan(); });
}

int main(int argc, char **argv) {
    MapControl mapControl = MapControl(true);

    handleKeyboardEventsScan(&mapControl);
    mapControl.Run();

    // Process cloud point generated
    std::vector<OfflineMapPoint*> cloudPoints = mapControl.GetCloudPoint();
    std::cout << "Number of points - User: " << cloudPoints.size() << std::endl;
    std::cout << "Number of points - ORB SLAM: " << mapControl.GetSystem()->GetMap()->GetAllMapPoints().size() << std::endl;

    handleKeyboardEventsResult(&mapControl);

    mapControl.SetResultPoint(cv::Point3d(0.3, 0.2, 0.1));
    mapControl.CheckResults();

    return 0;
}
