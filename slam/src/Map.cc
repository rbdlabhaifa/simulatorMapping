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

#include "Map.h"
#define TEST_DATA 0xdeadbeef
#include<mutex>
namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

template<class Archive>
    void Map::save(Archive & ar, const unsigned int version) const
    {
        unsigned int test_data = TEST_DATA;
        int nItems = mspMapPoints.size();
        ar & nItems;
        cout << "{INFO}mspMapPoints size = " << nItems << endl;

        std::for_each(mspMapPoints.begin(), mspMapPoints.end(), [&ar](MapPoint* pMapPoint) {
            ar & *pMapPoint;
        });
        
        nItems = mspKeyFrames.size();
        cout << "{INFO}mspKeyFrames size = " << nItems << endl;
        ar & nItems;
        std::for_each(mspKeyFrames.begin(), mspKeyFrames.end(), [&ar](KeyFrame* pKeyFrame) {
            ar & *pKeyFrame;
        });

        nItems = mvpKeyFrameOrigins.size();
        cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;
        ar & nItems;
        std::for_each(mvpKeyFrameOrigins.begin(), mvpKeyFrameOrigins.end(), [&ar](KeyFrame* pKeyFrameOrigin) {
            ar & *pKeyFrameOrigin;
        });
        // Pertaining to map drawing
        //nItems = mvpReferenceMapPoints.size();
        //cout << "$${INFO}mvpReferenceMapPoints size = %d " << nItems << endl;
        //ar & nItems;
        //std::for_each(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end(), [&ar](MapPoint* pMapPointReference) {
        //    ar & *pMapPointReference;
        //});
        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
    }

    template<class Archive>
    void Map::load(Archive & ar, const unsigned int version)
    {
        unsigned int test_data;

        int nItems;
        ar & nItems;
        cout << "{INFO}mspMapPoints size = " << nItems << endl;
        
        for (int i = 0; i < nItems; ++i) {
            
            MapPoint* pMapPoint = new MapPoint();
            ar & *pMapPoint;
            mspMapPoints.insert(pMapPoint);
        }
        
        ar & nItems;
        cout << "{INFO}mspKeyFrames size = " << nItems << endl;

        for (int i = 0; i < nItems; ++i) {

            KeyFrame* pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
            mspKeyFrames.insert(pKeyFrame);
        }     
          

        ar & nItems;
        cout << "{INFO}mvpKeyFrameOrigins size = " << nItems << endl;

        for (int i = 0; i < nItems; ++i) {             

            KeyFrame* pKeyFrame = new KeyFrame;
            ar & *pKeyFrame;
			/* TODO : VerifyHere*/
            mvpKeyFrameOrigins.push_back(*mspKeyFrames.begin());
        }     

        ar & const_cast<long unsigned int &> (mnMaxKFid);

        ar & test_data;
        if (test_data == TEST_DATA)
            cout <<">>Map Loading Validated as True" << endl;
        else
            cout <<"ERROR Map Loading Validated as False: Got -" << test_data << " :( Check Load Save sequence" << endl;

    }


// Explicit template instantiation
template void Map::save<boost::archive::binary_oarchive>(
	boost::archive::binary_oarchive &, 
	const unsigned int) const;
template void Map::save<boost::archive::binary_iarchive>(
	boost::archive::binary_iarchive &, 
	const unsigned int) const;
template void Map::load<boost::archive::binary_oarchive>(
	boost::archive::binary_oarchive &, 
	const unsigned int);
template void Map::load<boost::archive::binary_iarchive>(
	boost::archive::binary_iarchive &, 
	const unsigned int);

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


cv::Mat points3d_to_mat(const std::vector<cv::Point3f>& points3d)
{
    std::size_t nPoints = points3d.size();
    cv::Mat mat((int)nPoints, 3, CV_32F);
    for (std::size_t i = 0; i < nPoints; i++)
    {
        mat.at<float>(i, 0) = points3d[i].x;
        mat.at<float>(i, 1) = points3d[i].y;
        mat.at<float>(i, 2) = points3d[i].z;
    }

    return mat.t();
}


std::pair<cv::Mat, cv::Mat> Map::calculate_align_matrices(){
    std::vector<ORB_SLAM2::MapPoint *> mapPoints = GetAllMapPoints();
    cv::Mat mu_align1;
    cv::Mat R_align;
    
    std::vector<cv::Point3f> points;
    points.reserve(mapPoints.size());
    for(const auto& mp: mapPoints){
        cv::Mat pos = mp->GetWorldPos().clone();
        points.emplace_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

    cv::Point3f mu_align_pnt(mu_align1.at<float>(0), mu_align1.at<float>(1), mu_align1.at<float>(2));
    cv::Mat mu_align(mu_align_pnt);

    std::cout << "Centering points" << std::endl;
    for(auto& p : points){
        p = p - mu_align_pnt;
    }

    cv::Mat A = points3d_to_mat(points);
    cv::Mat w,u,vt;
    // cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::SVDecomp(A,w,u,vt);
    R_align = u.t();
    std::cout << "R_align:" << R_align << std::endl;

    
    
    return {R_align, mu_align};
}


std::ofstream aligned_pos("results/aligned.xyz");
std::pair<cv::Mat, cv::Mat> Map::align_map()
{
    auto [R_align, mu_align] = calculate_align_matrices();

    // Update map
    bool update_map = false;
    // if(false)
    {
        std::vector<ORB_SLAM2::MapPoint *> mapPoints = GetAllMapPoints();
        for (auto &mp : mapPoints) {
            if (!mp || mp->isBad())
                continue;
            auto pnt3d = mp->GetWorldPos().clone();
            {
                cv::Mat align_pos;
                align_pos = R_align * (pnt3d - mu_align);
                if(update_map)
                    mp->SetWorldPos(align_pos);
                aligned_pos << align_pos.at<float>(0) << " " << align_pos.at<float>(1) << " " << align_pos.at<float>(2) << std::endl;
            }
        }
        
        // Align keyframes
        if(update_map)
        {
            std::vector<ORB_SLAM2::KeyFrame *> keyframes = GetAllKeyFrames();
            for (auto &kf : keyframes) {
                // break;
                if (!kf || kf->isBad())
                    continue;
                kf->align(R_align, mu_align);
            }
        }
        
        // Update MapPoints normal and depth after aligning keyframes
        if(update_map)
            for(auto& mp : mapPoints){
                mp->UpdateNormalAndDepth();
            }
    }

    return {R_align, mu_align};
}

} //namespace ORB_SLAM
