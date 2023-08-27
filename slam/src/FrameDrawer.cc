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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2 {

    FrameDrawer::FrameDrawer(Map *pMap, bool bReuse) : mpMap(pMap) {
        mState = Tracking::SYSTEM_NOT_READY;
        if (bReuse)
            mState = Tracking::LOST;

        mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    // to do: make a void function and use draw arrays from opengl to draw the points (circles and rectangles)
    void FrameDrawer::DrawFrame() {
        cv::Mat im;
        vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
        vector<int> vMatches; // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
        int state; // Tracking state

        glColor3f(1.0f, 1.0f, 1.0f);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        //Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            state = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;

            mIm.copyTo(im);

            if (mState == Tracking::NOT_INITIALIZED) {
                vCurrentKeys = mvCurrentKeys;
                vIniKeys = mvIniKeys;
                vMatches = mvIniMatches;
            } else if (mState == Tracking::OK) {
                vCurrentKeys = mvCurrentKeys;
                vbVO = mvbVO;
                vbMap = mvbMap;
            } else if (mState == Tracking::LOST) {
                vCurrentKeys = mvCurrentKeys;
            }
        } // destroy scoped mutex -> release mutex

        if (!im.empty() && im.channels() < 3) //this should be always true
            cvtColor(im, im, CV_GRAY2BGR);

        cv::Mat imWithInfo;
        DrawTextInfo(im, state, imWithInfo);

        pangolin::GlTexture imageTexture(imWithInfo.cols, imWithInfo.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
        imageTexture.Upload(imWithInfo.data, GL_BGR, GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();

        GLuint texture[2];
        glGenTextures(2, texture);
        glBindTexture(GL_TEXTURE_2D, texture[1]);
        glBindTexture(GL_TEXTURE_2D, texture[0]);

        //make the gl_lines draw on top of imagetexture
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glLineWidth(2.0f);

        //Draw
        // make it in GL
        if (state == Tracking::NOT_INITIALIZED) //INITIALIZING
        {

            // lines
            glBegin(GL_LINES);
            glColor3f(0, 1.0f, 0);
            for (unsigned int i = 0; i < vMatches.size(); i++) {
                if (vMatches[i] >= 0 && !vCurrentKeys.empty() && vCurrentKeys.size() > i) {

                    // draw the lines between the matches
                    float x1 = ((2 * vIniKeys[i].pt.x / imWithInfo.cols) - 1.0f);
                    float y1 = ((2 * vIniKeys[i].pt.y / imWithInfo.rows) - 1.0f);
                    float x2 = ((2 * vCurrentKeys[vMatches[i]].pt.x / imWithInfo.cols) - 1.0f);
                    float y2 = ((2 * vCurrentKeys[vMatches[i]].pt.y / imWithInfo.rows) - 1.0f);
                    glVertex3f(x1, y1, 0);
                    glVertex3f(x2, y2, 0);

                }
            }
            glEnd();

        } else if (state == Tracking::OK) //TRACKING
        {
            mnTracked = 0;
            mnTrackedVO = 0;
            const float r = 5;
            for (int i = 0; i < N; i++) {
                if ((i < vbVO.size() && vbVO[i]) || (i < vbMap.size() && vbMap[i])) {
                    float x1 = ((2 * vCurrentKeys[i].pt.x / imWithInfo.cols) - 1.0f);
                    float y1 = ((2 * vCurrentKeys[i].pt.y / imWithInfo.rows) - 1.0f);
                    float width = 0.01f;

                    // This is a match to a MapPoint in the map
                    if (vbMap[i]) {

                        // draw rectangle
                        glColor3f(0, 1.0f, 0);
                        glBegin(GL_LINE_LOOP);
                        glVertex2f(x1 - width, y1 - width);
                        glVertex2f(x1 - width, y1 + width);
                        glVertex2f(x1 + width, y1 + width);
                        glVertex2f(x1 + width, y1 - width);
                        glEnd();

                        glBegin(GL_POINTS);
                        glVertex2f(x1, y1);
                        glEnd();

                        mnTracked++;
                    }
                    else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        glColor3f(0, 1.0f, 0);
                        glBegin(GL_LINE_LOOP);
                        glVertex2f(x1 - width, y1 - width);
                        glVertex2f(x1 - width, y1 + width);
                        glVertex2f(x1 + width, y1 + width);
                        glVertex2f(x1 + width, y1 - width);
                        glEnd();

                        glBegin(GL_POINTS);
                        glVertex2f(x1, y1);
                        glEnd();

                        mnTrackedVO++;
                    }
                }
            }
        }
    }


    void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
        stringstream s;
        if (nState == Tracking::NO_IMAGES_YET)
            s << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            s << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK) {
            if (!mbOnlyTracking)
                s << "SLAM MODE |  ";
            else
                s << "LOCALIZATION | ";
            int nKFs = mpMap->KeyFramesInMap();
            int nMPs = mpMap->MapPointsInMap();
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
            if (mnTrackedVO > 0)
                s << ", + VO matches: " << mnTrackedVO;
        } else if (nState == Tracking::LOST) {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        } else if (nState == Tracking::SYSTEM_NOT_READY) {
            s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
        im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
        imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
        cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(255, 255, 255), 1, 8);

    }

    void FrameDrawer::Update(Tracking *pTracker) {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImGray.copyTo(mIm);
        mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
        N = mvCurrentKeys.size();
        mvbVO = vector<bool>(N, false);
        mvbMap = vector<bool>(N, false);
        mbOnlyTracking = pTracker->mbOnlyTracking;


        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
            mvIniKeys = pTracker->mInitialFrame.mvKeys;
            mvIniMatches = pTracker->mvIniMatches;
        } else if (pTracker->mLastProcessedState == Tracking::OK) {
            for (int i = 0; i < N; i++) {
                MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP) {
                    if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
                        if (pMP->Observations() > 0)
                            mvbMap[i] = true;
                        else
                            mvbVO[i] = true;
                    }
                }
            }
        }
        mState = static_cast<int>(pTracker->mLastProcessedState);
    }

} //namespace ORB_SLAM
