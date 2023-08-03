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
    MapControl mapControl = MapControl(false);

    handleKeyboardEventsScan(&mapControl);
    mapControl.Run();

    // Process cloud point generated
    std::vector<OfflineMapPoint*> cloudPoints = mapControl.GetCloudPoint();
    std::cout << "Number of points: " << cloudPoints.size() << std::endl;

    nlohmann::json data = mapControl.GetData();

    std::string fileLocation = data["savePartialPointsPath"];

    std::cout << "Saving points to " << fileLocation << std::endl;

    std::ofstream pointsData(fileLocation);
    for (auto &point : cloudPoints) {
        pointsData << point->point.x << "," << point->point.y << "," << point->point.z << std::endl;
    }
    pointsData.close();

    std::cout << "Points saved!" << std::endl;

    return 0;
}
