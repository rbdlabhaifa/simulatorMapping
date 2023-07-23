//
// Created by Liam Vanunu
//

#include <unistd.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "System.h"
#include "include/OfflineMapPoint.h"

#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

class Simulator {
public:
    // Methods
    Simulator(); // constructor for Simulator
    ~Simulator(); // destructor for Simulator

    void Run(); // runs the simulator, opens 2 window, one for orb slam and one for movement in the model
                // and updates them according to user input

    // options for user input (camera movement and such)
    void ToggleFollowCamera();
    void ToggleShowPoints();
    void DoReset();
    void MoveLeft();
    void MoveRight();
    void MoveDown();
    void MoveUp();
    void MoveBackward();
    void MoveForward();
    void RotateLeft();
    void RotateRight();
    void RotateDown();
    void RotateUp();

    // finish the scan
    void FinishScan();

    // the function returns the cloud of Map points found
    std::vector<OfflineMapPoint*> GetCloudPoint();

    void SetResultPoint(const cv::Point3d resultPoint); // sets this->mResultPoint to be resultPoint
    void CheckResults(); // draws the scanned map with the result and real result point

private:
    // Methods
    void initPoints(); // initiates the simulator, fills this->mPoints with existing map points

    void createSimulatorSettings(); // defines the simulator settings according to demoSettings.json

    void build_window(std::string title); // creates the window

    void trackOrbSlam(); // performs tracking (finds map points)

    std::vector<OfflineMapPoint*> getPointsFromTcw(); // get the points the camera can currently see

    void reset(); // resets the simulator, empties found points and resets the camera

    // functions for movement and rotation of camera and user by value
    void applyUpToModelCam(double value);
    void applyRightToModelCam(double value);
    void applyForwardToModelCam(double value);

    void applyYawRotationToModelCam(double value);
    void applyPitchRotationToModelCam(double value);

    void drawMapPoints(); // draw the map points giving different color according to category 
                            // of point (new in frame, not new in frame, all other points)
    
    void drawResultPoints(); // draws the points with the result point and real result point in different color

    void updateTwcByResultPoint(); // not implemented - Change Twc to center the result point when I do check results

    void saveOnlyNewPoints(); // only saves the point that were not already seen

    void BuildCloudScanned(); // add all scanned points to  this->mCloudScanned

    // Members
    nlohmann::json mData; // a file of meta data

    std::vector<OfflineMapPoint*> mPoints; // the map points in the simulator

    bool mUseOrbSlam; // whether to use orb slam
    std::string mVocPath; // path to vocabulary
    bool mTrackImages;
    bool mLoadMap; // whether to load pre-made map points
    std::string mLoadMapPath; // path to pre-made map points
    ORB_SLAM2::System *mSystem; // an orb slam system instance allowing the usage of orb slam functionality

    std::vector<OfflineMapPoint*> mCurrentFramePoints; // map points in current frame
    std::vector<OfflineMapPoint*> mPointsSeen; // all seen points
    std::vector<OfflineMapPoint*> mNewPointsSeen; // point that were not seen previously

    std::vector<OfflineMapPoint*> mCloudScanned; // all the map points that were scanned

    cv::Point3d mResultPoint;
    cv::Point3d mRealResultPoint;

    std::string mSimulatorViewerTitle;
    std::string mResultsWindowTitle;

    double mPointSize;
    double mResultsPointSize;

    double mRotateScale;
    double mMovingScale;

    pangolin::OpenGlRenderState mS_cam; // the camera instance

    pangolin::View mD_cam;

    // transformation from real world vector space to camera vector space and vice versa
    pangolin::OpenGlMatrix mTcw;
    pangolin::OpenGlMatrix mTwc; 

    // options for user and camera movement/rotation
    bool mFollow;

    bool mFollowCamera;
    bool mShowPoints;
    bool mReset;
    bool mMoveLeft;
    bool mMoveRight;
    bool mMoveDown;
    bool mMoveUp;
    bool mMoveBackward;
    bool mMoveForward;
    bool mRotateLeft;
    bool mRotateRight;
    bool mRotateDown;
    bool mRotateUp;
    bool mFinishScan;

    float mViewpointX;
    float mViewpointY;
    float mViewpointZ;
    float mViewpointF;

    cv::Point3d mStartPosition;
    cv::Point3d mCurrentPosition;

    double mStartYaw;
    double mCurrentYaw;
    double mStartPitch;
    double mCurrentPitch;
    double mStartRoll;
    double mCurrentRoll;

    std::string mSimulatorPath; // path to save simulator related files to
    std::string mCloudPointPath; // path to save the cloud of points files to
    std::string mConfigPath; // path to configuration of simulator (camera settings, orb slam settings)

    bool mCloseResults;
};

#endif //ORB_SLAM2_SIMULATOR_H
