//
// Created by tzuk on 6/25/23.
//

#ifndef ORB_SLAM2_ROOMEXIT_H
#define ORB_SLAM2_ROOMEXIT_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <unordered_map>
#include <map>

class LineEigen {
public:
    LineEigen(Eigen::Vector2d &point, double slope);

    LineEigen();

    LineEigen(Eigen::Vector2d &point1, Eigen::Vector2d &point2);

    double getSumOfDistanceToCloud(std::vector<Eigen::Vector3d> &cloud);

    double getSlope() { return slope; };

    double getDistanceToPoint(Eigen::Vector2d &point);

private:

    double slope;
    Eigen::Vector2d origin;
    Eigen::Vector2d dest;
    double yIntercept;
};

class RoomExit {
public:
    RoomExit(std::vector<Eigen::Vector3d> &data);

    std::vector<std::pair<double, Eigen::Vector3d>> getExitPoints();

private:
    Eigen::Vector3d findCenter(std::vector<Eigen::Vector3d> &data);

    void filterByVariance();

    void calculateVariances();

    void getPolygonVertices();

    void createLines();

    void findBestLines();

    std::vector<std::pair<double, Eigen::Vector3d>> getExitPointsByVariance();

    bool isInsidePolygon(Eigen::Vector2d &point);

    std::vector<LineEigen> findBestLinesInSector(std::vector<Eigen::Vector3d> &sector, std::vector<LineEigen> &sectorLines);

    std::vector<std::vector<Eigen::Vector3d>> dividePointsToAxesSectors();


    std::unordered_map<int, double> slicesVariances;
    std::map<int, Eigen::Vector2d> polygonVertices;
    std::map<int, std::vector<Eigen::Vector3d >> slices;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normalizedPoints;
    std::vector<LineEigen> lines;
};


#endif //ORB_SLAM2_ROOMEXIT_H
