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

#ifndef RASPBERRY_KEYFRAME_H
#define RASPBERRY_KEYFRAME_H
#include <iostream>
using namespace std;
#include "MapPoint.h"
#include "../Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "../Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "KeyFrame.h"
#include "Serialization.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/serialization/split_member.hpp>
#include <mutex>

#include <eigen3/Eigen/Core>

// #define _BAR_

namespace ORB_SLAM2
{
    // struct id_map
    // {
    // 	bool is_valid;
    // 	long unsigned int id;
    // };
    // class Map;
    // class MapPoint;
    // class Frame;
    // class KeyFrameDatabase;
    // struct id_map;

    // class CameraMatrix{
    //     public:
    //         CameraMatrix()
    //         {

    //         }

    //         CameraMatrix(cv::Mat mtx):fx_(mtx.at<float>(0, 0)), fy_(mtx.at<float>(1, 1)), cx_(mtx.at<float>(0, 2)), cy_(mtx.at<float>(1, 2))
    //         {
    // 	        invfx_ = 1/fx_;
    // 	        invfy_ = 1/fy_;
    //         }

    //         float fx_;
    //         float fy_;
    //         float cx_;
    //         float cy_;
    //         float invfx_;
    //         float invfy_;

    //     private:
    //         friend class boost::serialization::access;
    //         // When the class Archive corresponds to an output archive, the
    //         // & operator is defined similar to <<.  Likewise, when the class Archive
    //         // is a type of input archive the & operator is defined similar to >>.
    //         template<class Archive>
    //         void serialize(Archive & ar, const unsigned int version)
    //         {
    //             ar & fx_;
    //             ar & fy_;
    //             ar & cx_;
    //             ar & cy_;
    //             ar & invfx_;
    //             ar & invfy_;
    //         }
    // };

    class KeyFrame;

    class RaspberryKeyFrame
    {
    public:
        RaspberryKeyFrame(KeyFrame &kf);

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
    public:
        cv::Mat image;

        // static long unsigned int nNextId;
        // long unsigned int mnId;
        // const long unsigned int mnFrameId;

        // const double mTimeStamp;

        // // Grid (to speed up feature matching)
        // const int mnGridCols;
        // const int mnGridRows;
        // const float mfGridElementWidthInv;
        // const float mfGridElementHeightInv;

        // // Variables used by the tracking
        // long unsigned int mnTrackReferenceForFrame;
        // long unsigned int mnFuseTargetForKF;

        // // Variables used by the local mapping
        // long unsigned int mnBALocalForKF;
        // long unsigned int mnBAFixedForKF;

        // // Variables used by the keyframe database
        // long unsigned int mnLoopQuery;
        // int mnLoopWords;
        // float mLoopScore;
        // long unsigned int mnRelocQuery;
        // int mnRelocWords;
        // float mRelocScore;

        // // Variables used by loop closing
        // cv::Mat mTcwGBA;
        // cv::Mat mTcwBefGBA;
        // long unsigned int mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy; //, mbf, mb, mThDepth;

        // // Number of KeyPoints
        // const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<cv::KeyPoint> mvKeysUn;
        // const std::vector<float> mvuRight; // negative value for monocular points
        // const std::vector<float> mvDepth; // negative value for monocular points
        cv::Mat mDescriptors;

        //BoW
        // DBoW2::BowVector mBowVec;
        // DBoW2::FeatureVector mFeatVec;

        // // Pose relative to parent (this is computed when bad flag is activated)
        // cv::Mat mTcp;

        // Scale
        // const int mnScaleLevels;
        // const float mfScaleFactor;
        // const float mfLogScaleFactor;
        // const std::vector<float> mvScaleFactors;
        // const std::vector<float> mvLevelSigma2;
        // const std::vector<float> mvInvLevelSigma2;

        // // Image bounds and calibration
        // const int mnMinX;
        // const int mnMinY;
        // const int mnMaxX;
        // const int mnMaxY;
        // const cv::Mat mK;

        // The following variables need to be accessed trough a mutex to be thread safe.
    protected:
        // SE3 Pose and camera center
        cv::Mat Tcw;
        // cv::Mat Twc;
        // cv::Mat Ow;

        // cv::Mat Cw; // Stereo middel point. Only for visualization

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;
        // std::map<long unsigned int, id_map> 	   mmMapPoints_nId;

        // BoW
        // KeyFrameDatabase* mpKeyFrameDB;
        // ORBVocabulary* mpORBvocabulary;

        // Grid over the image to speed up feature matching
        // std::vector< std::vector <std::vector<size_t> > > mGrid;

        // std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
        // 	std::map<long unsigned int, int> 	   mConnectedKeyFrameWeights_nId;
        // std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
        // 	std::map<long unsigned int, id_map> 	mvpOrderedConnectedKeyFrames_nId;
        // std::vector<int> mvOrderedWeights;

        // Spanning Tree and Loop Edges
        // bool mbFirstConnection;

        // KeyFrame* mpParent;
        // 	id_map mparent_KfId_map;
        // std::set<KeyFrame*> mspChildrens;
        // 	std::map<long unsigned int, id_map> 	   mmChildrens_nId;
        // std::set<KeyFrame*> mspLoopEdges;
        // 	std::map<long unsigned int, id_map> 	   mmLoopEdges_nId;

        // Bad flags
        // bool mbNotErase;
        // bool mbToBeErased;
        // bool mbBad;

        // float mHalfBaseline; // Only for visualization

        // Map* mpMap;
        // #ifndef _BAR_
        // 	friend class boost::serialization::access;
        //  	template<class Archive>
        //     void serialize(Archive & ar, const unsigned int version)
        // 	{
        // 		boost::serialization::split_member(ar, *this, version);
        // 	}

        // 	template<class Archive>
        // 	void save(Archive & ar, const unsigned int version) const;

        // 	template<class Archive>
        // 	void load(Archive & ar, const unsigned int version);
        // #endif

        // std::mutex mMutexPose;
        // std::mutex mMutexConnections;
        // std::mutex mMutexFeatures;

    public:
        //   void rpi_save(const std::string& file_name){
        //     std::ofstream bin_file(file_name, std::ios::out | std::ios::binary);

        //     boost::archive::binary_oarchive oa(bin_file);

        //     oa << *this;

        //     bin_file.close();

        //   }

    private:
        friend class boost::serialization::access;

        template <class Archive>
        void save(Archive &ar, const unsigned int version) const
        {
            std::vector<cv::Point2f> kpnts;
            std::vector<cv::Point2f> kpnts_undist;
            std::vector<int> kpnts_octave;
            std::vector<cv::Point3f> list_points3d;

            uint nKeyPoints = this->mvKeys.size();
            kpnts.reserve(nKeyPoints);
            kpnts_undist.reserve(nKeyPoints);
            kpnts_octave.reserve(nKeyPoints);
            for (uint i = 0; i < nKeyPoints; i++)
            {
                kpnts.push_back(this->mvKeys[i].pt);
                kpnts_undist.push_back(this->mvKeysUn[i].pt);
                kpnts_octave.push_back(this->mvKeysUn[i].octave);
            }

            uint nDescs = this->mDescriptors.rows;
            list_points3d.reserve(nDescs);
            for (uint i = 0; i < nDescs; i++)
            {
                MapPoint *p = this->mvpMapPoints[i];
                if (p && !p->isBad())
                {
                    auto point = p->GetWorldPos();
                    Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
                    list_points3d.emplace_back(v.x(), v.y(), v.z());
                }
                else
                {
                    list_points3d.emplace_back(0, 0, 0);
                }
            }

            cv::Mat R = this->Tcw.rowRange(0, 3).colRange(0, 3);
            cv::Mat t = this->Tcw.rowRange(0, 3).col(3);

            R.convertTo(R, CV_32F);
            t.convertTo(t, CV_32F);

            // ar & this->image;
            std::cout << "Descriptors: " << mDescriptors << std::endl;
            ar &this->mDescriptors.clone();
            ar &kpnts;
            ar &kpnts_undist;
            ar &const_cast<std::vector<int> &>(kpnts_octave);
            ar &list_points3d;
            ar &R;
            ar &t;
            ar &this->fx;
            ar &this->fy;
            ar &this->cx;
            ar &this->cy;
            ar &this->invfx;
            ar &this->invfy;
        }

        template <class Archive>
        void load(Archive &ar, const unsigned int version)
        {
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()
    };

} //namespace ORB_SLAM

#endif // KEYFRAME_H
