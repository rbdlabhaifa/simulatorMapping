#include "include/simulator.h"

void handleKeyboardEventsScan(Simulator *simulator) {
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

void handleKeyboardEventsResult(Simulator *simulator) {
    pangolin::RegisterKeyPressCallback('S', [&simulator]() { simulator->FinishScan(); });
}

int main(int argc, char **argv) {
    Simulator simulator = Simulator();

    handleKeyboardEventsScan(&simulator);
    simulator.Run();

    // Process cloud point generated
    std::vector<OfflineMapPoint*> cloudPoints = simulator.GetCloudPoint();
    std::cout << "Number of points: " << cloudPoints.size() << std::endl;

    std::string fileLocation = "/home/liam/firstFramePoints.csv";

    std::cout << "Saving points to " << fileLocation << std::endl;

    std::ofstream pointsData(fileLocation);
    for (auto &point : cloudPoints) {
        pointsData << point->point.x << "," << point->point.y << "," << point->point.z << std::endl;
    }
    pointsData.close();

    std::cout << "Points saved!" << std::endl;

    return 0;
}
