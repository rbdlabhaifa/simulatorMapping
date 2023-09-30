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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"

#include<mutex>

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}

void KeyFrameDatabase::set_vocab(ORBVocabulary *voc)
{
    mpVoc = voc;
}
template<class Archive>
void KeyFrameDatabase::save(Archive& ar, const unsigned int version) const
{
    int nItems_a, nItems_b;
    nItems_a = mvInvertedFile.size();
    ar& nItems_a;
    cout << "{INFO}Database elmnts = %d " << nItems_a << endl;

    for (auto it = mvInvertedFile.begin(); it != mvInvertedFile.end(); ++it) {
        nItems_b = (*it).size();
        cout << "{INFO}kfs no elmnts = %d " << nItems_b << endl;

        ar& nItems_b;
        for (auto lit = (*it).begin(); lit != (*it).end(); ++lit) {
            ar& ((**lit));
        }
    }
#if 0
    std::for_each(mvInvertedFile.begin(), mvInvertedFile.end(), [&ar](list<KeyFrame*>* plKeyFrame) {

        nItems_b = (*plKeyFrame).size();
        ar& nItems_b;
        std::for_each((*plKeyFrame).begin(), (*plKeyFrame).end(), [&ar](KeyFrame* pKeyFrame) {
            ar&* pKeyFrame;
            });



        });
#endif      

    }

template<class Archive>
void KeyFrameDatabase::load(Archive& ar, const unsigned int version)
{
    int nItems_a, nItems_b;
    int j, i;
    vector<KeyFrame*> temp_list;
    ar& nItems_a;
    cout << "{INFO}Database elmnts = %d " << nItems_a << endl;

    for (i = 0; i < nItems_a; ++i) {

        ar& nItems_b;
        cout << "{INFO}kfs no elmnts = %d " << nItems_b << endl;

        for (j = 0; j < nItems_b; ++j) {
            KeyFrame* pKeyFrame = new KeyFrame;
            ar&* pKeyFrame;
            temp_list.push_back(pKeyFrame);
        }
        mvInvertedFile.push_back(temp_list);
    }
}


// Explicit template instantiation
template void KeyFrameDatabase::save<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &, 
    const unsigned int) const;
template void KeyFrameDatabase::save<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &, 
    const unsigned int) const;
template void KeyFrameDatabase::load<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &, 
    const unsigned int);
template void KeyFrameDatabase::load<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &, 
    const unsigned int);


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // std::cout << "Adding keyframes to database." << std::endl;

    for(DBoW3::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
    {
        // List of keyframes that share the word
        vector<KeyFrame*>& lKFs = mvInvertedFile[vit->first];

        for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
        {
            if (pKF == *lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    unordered_map<KeyFrame*, int> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    vector <KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for (auto vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            vector<KeyFrame*>& lKFs = mvInvertedFile[vit->first];

            for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame* pKFi = *lit;
                if (pKFi->mnLoopQuery != pKF->mnId)
                {
                    pKFi->mnLoopWords = 0;
                    if (!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }

        if (lKFsSharingWords.empty())
            return vector<KeyFrame*>();
        vector<pair<float, KeyFrame*> > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (auto lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            maxCommonWords = ((*lit)->mnLoopWords > maxCommonWords) ? (*lit)->mnLoopWords : maxCommonWords; //change to ternary operator
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for (auto lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame* pKFi = *lit;

            if (pKFi->mnLoopWords > minCommonWords)
            {
                nscores++;

                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                si >= minScore ? lScoreAndMatch.push_back(make_pair(si, pKFi)) : void();//change to ternary operator

            }
        }

        if (lScoreAndMatch.empty())
            return vector<KeyFrame*>();

        vector<pair<float, KeyFrame*> > lAccScoreAndMatch;
        float bestAccScore = minScore;

        // Lets now accumulate score by covisibility
        for (auto it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame* pKFi = it->second;
            vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = it->first;
            KeyFrame* pBestKF = pKFi;
            for (auto vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame* pKF2 = *vit;
                if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
                {
                    accScore += pKF2->mLoopScore;
                    if (pKF2->mLoopScore > bestScore)
                    {
                        pBestKF = pKF2;
                        bestScore = pKF2->mLoopScore;
                    }
                }
            }

            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            bestAccScore = (accScore > bestAccScore) ? accScore : bestAccScore;// change to  ternary operator

        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.75f * bestAccScore;

        unordered_map<KeyFrame*, float> spAlreadyAddedKF;
        vector<KeyFrame*> vpLoopCandidates;
        vpLoopCandidates.reserve(lAccScoreAndMatch.size());

        for (auto it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            if (it->first > minScoreToRetain)
            {
                KeyFrame* pKFi = it->second;
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    vpLoopCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert({ pKFi,it->first });
                }
            }
        }
        return vpLoopCandidates;
    }
}
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame* F)
{
    vector<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for (auto vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
        {

            vector<KeyFrame*>& lKFs = mvInvertedFile[vit->first];

            for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {


                KeyFrame* pKFi = *lit;
                if (pKFi->mnRelocQuery != F->mnId)
                {


                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    // BAR
    // return lKFsSharingWords;
    if (lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        maxCommonWords = (*lit)->mnRelocWords > maxCommonWords ? (*lit)->mnRelocWords : maxCommonWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    vector<pair<float, KeyFrame*> > lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (auto lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if (pKFi->mnRelocWords > minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    vector<pair<float, KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (auto it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for (vector<KeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if (pKF2->mnRelocQuery != F->mnId)
                continue;

            accScore += pKF2->mRelocScore;
            if (pKF2->mRelocScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        bestAccScore = accScore > bestAccScore ? accScore : bestAccScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    unordered_map<KeyFrame*, float> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (auto it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        const float& si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert({ pKFi ,it->first });
            }
        }
    }

    return vpRelocCandidates;
}
} //namespace ORB_SLAM
