
#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

#include <opencv2/core.hpp>

namespace boost{
  namespace serialization{
template<class Archive>
        void serialize(Archive &ar, cv::Point3f &pt3, const unsigned int version){
            ar & pt3.x;
            ar & pt3.y;
            ar & pt3.z;
        }
        template<class Archive>
        void serialize(Archive &ar, cv::Point2f &pt2, const unsigned int version){
            ar & pt2.x;
            ar & pt2.y;
        }
  }
}


#endif