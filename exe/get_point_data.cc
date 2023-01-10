#include <string>
#include <iostream>
#include <nlohmann/json.hpp>

#include "include/Point.h"
#include "include/Auxiliary.h"

#define X (0-0.497015)
#define Y (-0.150409)
#define Z (0.551933)

void getFramesWithPoints(std::vector<std::string> row, std::map<int, Point2D>* framesWithPoint) {
    for(int i=3; i<row.size(); i+=3) {
        Point2D currentPoint(std::stod(row[i+1]), std::stod(row[i+2]));
        (*framesWithPoint).insert(std::make_pair(std::stoi(row[i]), currentPoint))  ;
    }
}

bool searchPoint(std::string csvPath, Point point, std::map<int, Point2D>* framesWithPoint) {
    std::fstream pointData;
    pointData.open(csvPath, std::ios::in);

    std::vector<std::string> row;
    std::string line, word, temp;

    while (!pointData.eof()) {
        Point pointToCompare;

        row.clear();
        
        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            row.push_back(word);
        }
        
        pointToCompare = Point(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
        if (point.compare(pointToCompare)) {
            getFramesWithPoints(row, framesWithPoint);
            pointData.close();
            return true;
        }
    }
    pointData.close();
    return false;
}

void printFrames(std::map<int, Point2D> framesWithPoint) {
    for(const auto &myPair : framesWithPoint)
    {
        std::cout << myPair.first << ": (" << myPair.second.x << ", " << myPair.second.y << ")" << std::endl;
    }
}

int main() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string csvPath = data["getPointDataCsv"];
    std::map<int, Point2D> framesWithPoint;

    if (!searchPoint(csvPath, Point(X, Y, Z), &framesWithPoint)) {
        std::cout << "There is no such point!" << std::endl;
        return -1;
    }

    printFrames(framesWithPoint);

    return 0;
}

