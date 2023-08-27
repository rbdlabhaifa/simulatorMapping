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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

#include "include/Auxiliary.h"

namespace ORB_SLAM2
{

    Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking,
        const string& strSettingPath, bool bReuse, bool isPangolinExists) : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
        mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
        mbReuse = bReuse;
        this->isPangolinExists = isPangolinExists;

        // std::string settingPath = Auxiliary::GetGeneralSettingsPath();
        // std::ifstream programData(settingPath);
        // nlohmann::json data;
        // programData >> data;
        // programData.close();

        // std::string map_input_dir = data["mapInputDir"];
        // mCloudPoints = map_input_dir + "cloud1.csv";

        // double startPointX = data["startingCameraPosX"];
        // double startPointY = data["startingCameraPosY"];
        // double startPointZ = data["startingCameraPosZ"];
        // mCurrentPosition = cv::Point3d(startPointX, startPointY, startPointZ);
        // mCurrentYaw = data["yawRad"];
        // mCurrentPitch = data["pitchRad"];
        // mCurrentRoll = data["rollRad"];

        // mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
        // mPointsSeen = std::vector<cv::Point3d>();

        // mMovingScale = data["movingScale"];
        // mRotateScale = data["rotateScale"];
    }

    void Viewer::Run()
    {
        mbFinished = false;
        if (isPangolinExists)
        {
            pangolin::BindToContext("ORB-SLAM2: Map & Camera Viewer");
        }
        else
        {
            pangolin::CreateWindowAndBind("ORB-SLAM2: Map & Camera Viewer", 1300, 750);
        }
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if (isPangolinExists)
        {
            pangolin::Panel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        }
        else
        {
            pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        }
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", mbReuse, true);
        pangolin::Var<bool> menuOpenSimulator("menu.Open Simulator", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);
        pangolin::Var<bool> menuShutDown("menu.ShutDown", false, false);
        pangolin::Var<bool> menuMoveLeft("menu.Move Left", false, false);
        pangolin::Var<bool> menuMoveRight("menu.Move Right", false, false);
        pangolin::Var<bool> menuMoveDown("menu.Move Down", false, false);
        pangolin::Var<bool> menuMoveUp("menu.Move Up", false, false);
        pangolin::Var<bool> menuRotateLeft("menu.Rotate Left", false, false);
        pangolin::Var<bool> menuRotateRight("menu.Rotate Right", false, false);
        pangolin::Var<bool> menuRotateDown("menu.Rotate Down", false, false);
        pangolin::Var<bool> menuRotateUp("menu.Rotate Up", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        pangolin::View& view1 = pangolin::Display("view1")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f);

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::Display("cam")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::Display("multi")
            .SetBounds(pangolin::Attach::Pix(175), 1.0, 0.0, 1.0)
            .SetLayout(pangolin::LayoutEqualHorizontal)
            .AddDisplay(d_cam)
            .AddDisplay(view1);

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        bool bFollow = true;
        bool bLocalizationMode = mbReuse;

        while (1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (!menuOpenSimulator)
            {
                mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            }
            else
            {
                Twc.m[0] = (float)mTwc.at<double>(0);
                Twc.m[1] = (float)mTwc.at<double>(1);
                Twc.m[2] = (float)mTwc.at<double>(2);
                Twc.m[4] = (float)mTwc.at<double>(4);
                Twc.m[5] = (float)mTwc.at<double>(5);
                Twc.m[6] = (float)mTwc.at<double>(6);
                Twc.m[8] = (float)mTwc.at<double>(8);
                Twc.m[9] = (float)mTwc.at<double>(9);
                Twc.m[10] = (float)mTwc.at<double>(10);
                Twc.m[12] = (float)mTwc.at<double>(12);
                Twc.m[13] = (float)mTwc.at<double>(13);
                Twc.m[14] = (float)mTwc.at<double>(14);
            }
            if (menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if (menuLocalizationMode && !bLocalizationMode && !menuOpenSimulator)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if (!menuLocalizationMode && bLocalizationMode && !menuOpenSimulator)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }
            d_cam.Activate(s_cam);

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            // mpMapDrawer->DrawCurrentCamera(Twc);
            if (!menuOpenSimulator && (menuShowKeyFrames || menuShowGraph))
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
            if (menuShowPoints)
            {
                if (!menuOpenSimulator)
                {
                    mpMapDrawer->DrawMapPoints();
                }
                else
                {
                    mpMapDrawer->DrawMapPoints(true, mPointsSeen, mNewPointsSeen);
                }
            }

            if (!menuOpenSimulator)
            {
                if (mpFrameDrawer != nullptr)
                {
                    view1.Activate();

                    glColor3f(1.0f, 1.0f, 1.0f);
                    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

                    mpFrameDrawer->DrawFrame();
                    // moved the functions up
                    cv::Mat rawIm = mpFrameDrawer->RetImage();
                    pangolin::GlTexture imageTexture(rawIm.cols, rawIm.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
                    imageTexture.Upload(rawIm.data, GL_BGR, GL_UNSIGNED_BYTE);
                    imageTexture.RenderToViewportFlipY();

                    GLuint texture[2];
                    glGenTextures(2, texture);
                    glBindTexture(GL_TEXTURE_2D, texture[1]);
                    glBindTexture(GL_TEXTURE_2D, texture[0]);

                    int state = mpFrameDrawer->currState;
                    cout << "state: " << state << endl;
                    cout << "currState: " << mpFrameDrawer->currState << endl;

                    //make the gl_lines draw on top of imagetexture
                    glDisable(GL_DEPTH_TEST);
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glEnable(GL_LINE_SMOOTH);
                    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
                    glLineWidth(2.0f);


                    vector<int> vMatches = mpFrameDrawer->currVMatches;
                    vector<cv::KeyPoint> vIniKeys = mpFrameDrawer->currVIniKeys;
                    vector<cv::KeyPoint> vCurrentKeys = mpFrameDrawer->currVCurrentKeys;
                    int mnTracked = mpFrameDrawer->currmnTracked;
                    int mnTrackedVO = mpFrameDrawer->currmnTrackedVO;
                    vector<bool> vbVO = mpFrameDrawer->currmvbVO;
                    vector<bool> vbMap = mpFrameDrawer->currmvbMap;
                    int N = mpFrameDrawer->currN;

                    cout << "state is: " << state << endl;
                    cout << "N is: " << N << endl;

                    if (state == 1) //INITIALIZING
                    {

                        // lines
                        glBegin(GL_LINES);
                        glColor3f(0, 1.0f, 0);
                        for (unsigned int i = 0; i < vMatches.size(); i++) {
                            if (vMatches[i] >= 0 && !vCurrentKeys.empty() && vCurrentKeys.size() > i) {

                                // draw the lines between the matches
                                float x1 = ((2 * vIniKeys[i].pt.x / mImageWidth) - 1.0f);
								float y1 = ((2 * vIniKeys[i].pt.y / mImageHeight) - 1.0f);
								float x2 = ((2 * vCurrentKeys[vMatches[i]].pt.x / mImageWidth) - 1.0f);
								float y2 = ((2 * vCurrentKeys[vMatches[i]].pt.y / mImageHeight) - 1.0f);
								glVertex3f(x1, y1, 0);
                                glVertex3f(x2, y2, 0);

                            }
                        }
                        glEnd();
                    }
                    else if (state == 2){ //TRACKING
                    
                        mnTracked = 0;
                        mnTrackedVO = 0;
                        const float r = 5;
                        for (int i = 0; i < N; i++) {
                            if ((i < vbVO.size() && vbVO[i]) || (i < vbMap.size() && vbMap[i])) {
                                float x1 = ((2 * vCurrentKeys[i].pt.x / mImageWidth) - 1.0f);
                                float y1 = ((2 * vCurrentKeys[i].pt.y / mImageHeight) - 1.0f);
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

         //                           //draw circle around the point
         //                           glBegin(GL_LINE_LOOP);
									//glColor3f(1.0f, 0, 0);
         //                           for (int j = 0; j < 360; j++)
         //                           {
         //                               float degInRad = 2 * 3.142 * j / 300;
         //                               glVertex2f(cos(degInRad)*width + x1, sin(degInRad)*r + y1);
         //                           }
         //                           glEnd();

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


                                    ////draw circle around the point
                                    //glBegin(GL_LINE_LOOP);
                                    //glColor3f(1.0f, 0, 0);
                                    //for (int j = 0; j < 360; j++)
                                    //{
                                    //    float degInRad = 2 * 3.142 * j / 300;
                                    //    glVertex2f(cos(degInRad) * width + x1, sin(degInRad) * r + y1);
                                    //}
                                    //glEnd();
                                    mnTrackedVO++;
                                }
                            }
                        }
                        mpFrameDrawer->currmnTracked = mnTracked;
                        mpFrameDrawer->currmnTrackedVO = mnTrackedVO;
                    }
                                       
                    //// List coordinates of a triangle
                    //// These vertices will be relative to the coordinates of the window
                    //// which default in OpenGL to +/- 1.0 in X and Y (first two vertex ordinates)
                    //const float vertices[] = {
                    //    -0.5f, -0.5f, 0.0f,
                    //     0.5f, -0.5f, 0.0f,
                    //     0.0f,  0.5f, 0.0f
                    //};

                    //// We want our triangle to be a pleasant shade of blue!
                    //glColor3f(0.29f, 0.71f, 1.0f);

                    //// Clear the window
                    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    //// This buffer contains floating point vertices with 3 dimensions.
                    //// They starts from the 0th element and are packed without padding.
                    //glVertexPointer(3, GL_FLOAT, 0, vertices);

                    //// Use Them!
                    //glEnableClientState(GL_VERTEX_ARRAY);

                    //// Connect the first 3 of these vertices to form a triangle!
                    //glDrawArrays(GL_TRIANGLES, 0, 3);

                    //// Disable the stuff we enabled...
                    //glDisableClientState(GL_VERTEX_ARRAY);

                    
                }
            }

            pangolin::FinishFrame();


            if (menuMoveLeft)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPosition.x -= mMovingScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuMoveLeft = false;
            }

            if (menuMoveRight)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPosition.x += mMovingScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuMoveRight = false;
            }

            if (menuMoveDown)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPosition.y -= mMovingScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuMoveDown = false;
            }

            if (menuMoveUp)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPosition.y += mMovingScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuMoveUp = false;
            }

            if (menuRotateLeft)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentYaw -= mRotateScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::cout << "Current Pos: " << mCurrentPosition << ", yaw: " << mCurrentYaw << ", pitch: " << mCurrentPitch << ", roll: " << mCurrentRoll << std::endl;
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuRotateLeft = false;
            }

            if (menuRotateRight)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentYaw += mRotateScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuRotateRight = false;
            }

            if (menuRotateDown)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPitch -= mRotateScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuRotateDown = false;
            }

            if (menuRotateUp)
            {
                mPointsSeen.insert(mPointsSeen.end(), mNewPointsSeen.begin(), mNewPointsSeen.end());

                mCurrentPitch += mRotateScale;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                std::vector<cv::Point3d>::iterator it;
                for (it = mNewPointsSeen.begin(); it != mNewPointsSeen.end();)
                {
                    if (std::find(mPointsSeen.begin(), mPointsSeen.end(), *it) != mPointsSeen.end())
                    {
                        it = mNewPointsSeen.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                menuRotateUp = false;
            }

            if (menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                menuOpenSimulator = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;

                mCurrentPosition = cv::Point3d(0, 0, 0);
                mCurrentYaw = 0;
                mCurrentPitch = 0;
                mCurrentRoll = 0;

                mNewPointsSeen = Auxiliary::getPointsFromPos(mCloudPoints, mCurrentPosition, mCurrentYaw, mCurrentPitch, mCurrentRoll, mTwc);
                mPointsSeen = std::vector<cv::Point3d>();
            }

            if (menuShutDown)
            {
                mpSystem->shutdown_requested = true;
            }

            if (Stop())
            {
                while (isStopped())
                {
                    Sleep(3);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}