#ifndef TELLO_POINT_H
#define TELLO_POINT_H

#include <string>
#include <sstream>
#include <opencv2/core/mat.hpp>

class Point {
public:
    Point();

    Point(const Point &point);

    /*
     * a wrapper over the orb_slam2 representation of point, every point carry with it all the data we need
     * in order to calculate easy navigation to it
     * all the q* are the quaternion value of the frame that contains the point, the label is the label we
     * give to the point when we cluster it with other points
     * and the frame id is the orb_slam2 frame id in ORBSLAM2::Frame
     *
     * x is right left
     * y is up down
     * z is depth
     * q* is quaternion
     *
     * */
    Point(double x, double y, double z,const cv::Mat &rotationMatrix, int frameId = -1,
          int label = -1);

    Point(double x, double y, double z);
    bool operator==(const Point &ref) const {
        return this->x == ref.x && this->y == ref.y && this->z == ref.z;
    }

    Point operator -(const Point &ref) const {
        return {this->x - ref.x,this->y - ref.y, this->z - ref.z,ref.rotationMatrix};
    }
    std::string to_string() const {
        std::ostringstream ss;
        ss << this->x << "," << this->y << "," << this->z << "," << this->label;
        return ss.str();
    }

    bool compare(Point point);

    double x;
    double y;
    double z;
    cv::Mat rotationMatrix;
    int label;
    int frameId;

    Point &operator=(const Point &point) = default;

};
namespace std {

    template<>
    struct hash<Point> {
        std::size_t operator()(const Point &k) const {
            using std::size_t;
            using std::hash;
            using std::string;

            // Compute individual hash values for first,
            // second and third and combine them using XOR
            // and bit shifting:

            return ((hash<double>()(k.x)
                     ^ (hash<double>()(k.y) << 1)) >> 1)
                   ^ (hash<double>()(k.z) << 1);
        }
    };

};

class Point2D {
public:
    Point2D(double x, double y);

    double x;
    double y;
};

#endif //TELLO_POINT_H
