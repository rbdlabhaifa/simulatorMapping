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

    //create a new object
    Simulator();

    //destroys an object
    ~Simulator();

    //run and control our scan
    void Run();

    //change the state of the camera - either follow the camera movement or not
    void ToggleFollowCamera();

    //determine wether we draw the scanned Map Points when we scan whem
    void ToggleShowPoints();

    //tells the simulator to do a reset
    void DoReset();

    //tells the simulator to move left
    void MoveLeft();

    //tells the simulator to move right
    void MoveRight();

    //tells the simulator to move down
    void MoveDown();

    //tells the simulator to move up
    void MoveUp();

    //tells the simulator to move backwards
    void MoveBackward();

    //tells the simulator to move forward
    void MoveForward();

    //tells the simulator to rotate left in x axis
    void RotateLeft();

    //tells the simulator to rotate right in x axis
    void RotateRight();

    //tells the simulator to rotate down in y axis
    void RotateDown();

    //tells the simulator to rotate up in y axis
    void RotateUp();

    //tells the simulator to finish scan
    void FinishScan();


    //returns the Map Points of the current scan
    std::vector<OfflineMapPoint*> GetCloudPoint();

    //set a Point as the result Point
    void SetResultPoint(const cv::Point3d resultPoint);

    //draw the scanned Map Points from the result Point view
    void CheckResults();

private:
    // Methods

    // initialize the Map Points 
    void initPoints();

    //get the simulator settings
    void createSimulatorSettings();

    //create a display window
    void build_window(std::string title);

    //find Map Points in SLAM instance
    void trackOrbSlam();

    //returns which Map Points we can see from a certain position and angle
    std::vector<OfflineMapPoint*> getPointsFromTcw();

    //reset scanned Map Points and camera settings
    void reset();

    //gets a value
    //moves the camera up by the value of value
    void applyUpToModelCam(double value);

    //gets a value
    //moves the camera right by the value of value
    void applyRightToModelCam(double value);

    //gets a value
    //moves the camera forward by the value of value
    void applyForwardToModelCam(double value);

    //gets a value
    //spins the camera in the y axis by the value of value
    void applyYawRotationToModelCam(double value);

    //gets a value
    //spins the camera in the x axis by the value of value
    void applyPitchRotationToModelCam(double value);

    //draws an image of the Map Points that we have seen so far
    void drawMapPoints();

    //draws an image of the Map Points that we have seen so far with the result Points
    void drawResultPoints();

    //change the camera posioion to the result point
    void updateTwcByResultPoint();

    //extract the new seen Map Points in the current frame
    void saveOnlyNewPoints();

    //saves the scanned Map Points
    void BuildCloudScanned();

    // Members
    nlohmann::json mData;

    std::vector<OfflineMapPoint*> mPoints; // all the Map Points existing

    bool mUseOrbSlam;
    std::string mVocPath;
    bool mTrackImages;
    bool mLoadMap;
    std::string mLoadMapPath;
    ORB_SLAM2::System *mSystem;  

    std::vector<OfflineMapPoint*> mCurrentFramePoints; // Map Points in current frame
    std::vector<OfflineMapPoint*> mPointsSeen; // Map Points seen until now in the scan
    std::vector<OfflineMapPoint*> mNewPointsSeen; // new Map Points in the current frame

    std::vector<OfflineMapPoint*> mCloudScanned; // All of the scanned Points in the scan(only after it ends)

    cv::Point3d mResultPoint;
    cv::Point3d mRealResultPoint;

    std::string mSimulatorViewerTitle;
    std::string mResultsWindowTitle;

    double mPointSize;
    double mResultsPointSize;

    double mRotateScale;
    double mMovingScale;

    pangolin::OpenGlRenderState mS_cam; // camera instance

    pangolin::View mD_cam;

    //transformation from camera points to real cordinates and vice versa
    pangolin::OpenGlMatrix mTcw;
    pangolin::OpenGlMatrix mTwc;    

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

    std::string mSimulatorPath;
    std::string mCloudPointPath;
    std::string mConfigPath;

    bool mCloseResults;
};

#endif //ORB_SLAM2_SIMULATOR_H
