#ifndef DBSCAN_H
#define DBSCAN_H

#include <utility>
#include <vector>
#include <cmath>
#include "Point.h"
#include "Auxiliary.h"

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

class DBSCAN {
public:
    DBSCAN(unsigned int minPts, double eps, const std::vector<Point> &points,
           std::function<double(Point, Point)> func = Auxiliary::calculateDistanceXY) {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        clusterFunction = std::move(func);
    }

    ~DBSCAN() {}

    int run();

    std::vector<int> calculateCluster(const Point &point);

    int expandCluster(Point point, int clusterID);

    std::vector<Point> getPoints() { return m_points; }

    void setMinPoints(int newMinPoints) { m_minPoints = newMinPoints; }

private:
    std::vector<Point> m_points;
    unsigned int m_minPoints;
    double m_epsilon;
    std::function<double(Point, Point)> clusterFunction;
};

#endif // DBSCAN_H