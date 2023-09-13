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

#ifndef KEYFRAME_H
#define KEYFRAME_H
#include <iostream>
using namespace std;
#include "MapPoint.h"
#include "DBoW3/src/BowVector.h"
#include "DBoW3/src/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "Serialization.h"

#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/serialization/split_member.hpp>
#include <mutex>

#include <Eigen/Core>

namespace ORB_SLAM2
{
struct id_map
{
	bool is_valid;
	long unsigned int id;
};
class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
struct id_map;

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



class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    KeyFrame();	/* Default constructor for serialization */

    void align(const cv::Mat& R_align, const cv::Mat& mu_align);
    
	// Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::unordered_map<KeyFrame*, int> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::unordered_map<KeyFrame*, int> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::unordered_map<KeyFrame*, int> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::unordered_map<MapPoint*, int> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

	void SetMap(Map* map);
	void SetKeyFrameDatabase(KeyFrameDatabase* pKeyFrameDB);
	void SetORBvocabulary(ORBVocabulary* pORBvocabulary);
	void SetMapPoints(std::vector<MapPoint*> spMapPoints);
	void SetSpanningTree(std::vector<KeyFrame*> vpKeyFrames);
	void SetGridParams(std::vector<KeyFrame*> vpKeyFrames);



    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    cv::Mat image;

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    cv::Mat mDescriptors;

    //BoW
    DBoW3::BowVector mBowVec;
    DBoW3::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
	std::map<long unsigned int, id_map> 	   mmMapPoints_nId;

 	// BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
		std::map<long unsigned int, int> 	   mConnectedKeyFrameWeights_nId;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
		std::map<long unsigned int, id_map> 	mvpOrderedConnectedKeyFrames_nId;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
		
    KeyFrame* mpParent;
		id_map mparent_KfId_map;
        std::unordered_map<KeyFrame*, int> mspChildrens;
        std::unordered_map<long unsigned int, id_map> mmChildrens_nId;
        std::unordered_map<KeyFrame*, int> mspLoopEdges;
        std::unordered_map<long unsigned int, id_map> mmLoopEdges_nId;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;
	friend class boost::serialization::access;
 	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
	{
		boost::serialization::split_member(ar, *this, version);
	}
		
	template<class Archive>
	void save(Archive & ar, const unsigned int version) const;
	

	template<class Archive>
	void load(Archive & ar, const unsigned int version);
// #endif
	
    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
