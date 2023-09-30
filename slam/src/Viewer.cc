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

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

            if (bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if (!bFollow)
            {
                s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }

            if (!bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }

            d_cam.Activate(s_cam);

            mpMapDrawer->DrawKeyFrames(true, true);

            mpMapDrawer->DrawMapPoints();

            if (mpFrameDrawer != nullptr)
            {
                view1.Activate();

                mpFrameDrawer->DrawFrame();
            }

            pangolin::FinishFrame();

            if (Stop())
            {
                while (isStopped())
                {
                    System::systemUsleep(3);
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
