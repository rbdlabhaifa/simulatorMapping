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
    Simulator(); //constructor. Initializes all variables.
    ~Simulator(); //destructor

    void Run(); //runs the simulator

    void ToggleFollowCamera(); //The user has changed the state of whether the camera is being followed
    void ToggleShowPoints(); //The user changed the state of whether to show the points
    
    //Various options the user can set/do
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
    void FinishScan();

    std::vector<OfflineMapPoint*> GetCloudPoint(); //returns the cloud of map points

    void SetResultPoint(const cv::Point3d resultPoint); //set new result point
    void CheckResults(); 

private:
    // Methods
    void initPoints(); //Initializes the offline map points from the cloud

    void createSimulatorSettings(); //save the demoSettings.json in attribute mData

    void build_window(std::string title); //creates a new window with the title

    void trackOrbSlam();//finds map points from key points&descriptors

    std::vector<OfflineMapPoint*> getPointsFromTcw(); //returns the points we can see from the camera

    void reset(); //reset the simulator (position&camera settings)
    void applyUpToModelCam(double value); //move the drone up by value
    void applyRightToModelCam(double value); //move the drone right by value
    void applyForwardToModelCam(double value); //move the drone forward by value

    void applyYawRotationToModelCam(double value); //rotate the drone yaw rotation by value
    void applyPitchRotationToModelCam(double value); //rotate the drone pitch rotation by value

    void drawMapPoints(); //draws the points in three colors (new points from curr frame, old (seen) points from curr frame, points from other frames)
    void drawResultPoints(); // draws the result points and the map points

    void updateTwcByResultPoint(); // TODO: Change Twc to center the result point when I do check results

    void saveOnlyNewPoints(); //removes all the seen (old) points

    void BuildCloudScanned(); //builds a new cloud of map points (from old(seen) map points & new map points)

    // Members
    nlohmann::json mData; //contains demoSettings.json

    std::vector<OfflineMapPoint*> mPoints; //all the map points

    bool mUseOrbSlam; //whether we use orbslam or not
    std::string mVocPath;
    bool mTrackImages;
    bool mLoadMap; //whether load the map
    std::string mLoadMapPath; //path to the map
    ORB_SLAM2::System *mSystem; //instance of ORBSLAM(if we use it)

    //vectors of map points
    std::vector<OfflineMapPoint*> mCurrentFramePoints; //map points from curr frame
    std::vector<OfflineMapPoint*> mPointsSeen; //seen map points
    std::vector<OfflineMapPoint*> mNewPointsSeen; //new map points

    std::vector<OfflineMapPoint*> mCloudScanned; //cloud of map points

    cv::Point3d mResultPoint;
    cv::Point3d mRealResultPoint;

    std::string mSimulatorViewerTitle;
    std::string mResultsWindowTitle;

    //size of points
    double mPointSize;
    double mResultsPointSize;

    //scales of rotating and moving
    double mRotateScale;
    double mMovingScale;

    pangolin::OpenGlRenderState mS_cam;

    pangolin::View mD_cam;

    
    pangolin::OpenGlMatrix mTcw; //transformation from camera points to real points
    pangolin::OpenGlMatrix mTwc; //transformation from real points to camera points   

    bool mFollow;

    //different options and unctionalities the user
    bool mFollowCamera; //Is a user following the camera
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

    //the view of the camera
    float mViewpointX;
    float mViewpointY;
    float mViewpointZ;
    float mViewpointF;

    //positions
    cv::Point3d mStartPosition;
    cv::Point3d mCurrentPosition;

    //values of drone rotations
    double mStartYaw;
    double mCurrentYaw;
    double mStartPitch;
    double mCurrentPitch;
    double mStartRoll;
    double mCurrentRoll;

    //different paths
    std::string mSimulatorPath;
    std::string mCloudPointPath;
    std::string mConfigPath;

    bool mCloseResults;
};

#endif //ORB_SLAM2_SIMULATOR_H
