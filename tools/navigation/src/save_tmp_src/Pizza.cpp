//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Pizza.h"

std::vector<Line> Pizza::createPizzaLines(const Point& center, int angle) {
    std::vector<Line> lines;
    int amountOfSlices = ceil(360 / angle);
    double radianAngle = Auxiliary::angleToRadians(angle);
    for (int i = 0; i < amountOfSlices; i++) {
        double slope = tan(i * radianAngle);
        lines.emplace_back(Line(center, slope));
    }
    return lines;
}

std::map<int, std::vector<std::pair<Point, double>>>
Pizza::createPizzaSlices(const Point& pizzaCenter, const std::vector<std::pair<Point, double>>& pointsWithDistance, int angle) {
    std::unordered_map<int, std::vector<std::pair<Point, double>>> slices;
    for (const auto& pointWithDistance : pointsWithDistance) {
        int degree = int(Auxiliary::radiansToAngle(
                atan2(pointWithDistance.first.y - pizzaCenter.y, pointWithDistance.first.x - pizzaCenter.x)
        ) + 360) % 360;
        int sliceKey = int(degree / angle);
        auto slice = slices.find(sliceKey);
        if (slice == slices.end()) {
            slices.insert({sliceKey, std::vector<std::pair<Point, double>>{}});
        }
        slices.at(sliceKey).push_back(pointWithDistance);
    }
    std::map<int, std::vector<std::pair<Point, double>>> ordered_slices(slices.begin(), slices.end());
    return ordered_slices;
}