/*********** add-comments , task1 ***********/
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
    Simulator(); // Constructor, Initializes the Simulator object
    ~Simulator(); // Destructor, Cleans up resources and memory, in the end

    void Run(); //  Run the simulation

    // Camera Control Methods
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
    void FinishScan();

    std::vector<OfflineMapPoint*> GetCloudPoint(); // Returns the point cloud data

    void SetResultPoint(const cv::Point3d resultPoint); // Sets the result point for visualization
    void CheckResults(); // Checks the results of the simulation against ground truth (if available)

private:
    // Helper Methods
    void initPoints(); // Initializes the map points for the simulation

    void createSimulatorSettings(); // Creates the settings for the simulator

    void build_window(std::string title); // Builds the visualization window

    void trackOrbSlam(); // Tracks the camera movement using ORB-SLAM2

    std::vector<OfflineMapPoint*> getPointsFromTcw(); // Extracts map points from camera pose

    void reset(); // Resets the simulation state

    //Moves the camera up,right,forward relative to the model coordinates
    void applyUpToModelCam(double value);
    void applyRightToModelCam(double value);
    void applyForwardToModelCam(double value);

    void applyYawRotationToModelCam(double value); // Rotates the camera around the yaw
    void applyPitchRotationToModelCam(double value); // Rotates the camera around the pitch

    void drawMapPoints(); // Draws the map points in the visualization window
    void drawResultPoints(); // Draws the result points in the visualization window

    void updateTwcByResultPoint(); // Updates the camera pose by the result point

    void saveOnlyNewPoints(); // Saves only the newly observed map points

    void BuildCloudScanned(); // Builds the scanned cloud using the observed map points

    // Members
    nlohmann::json mData;

    std::vector<OfflineMapPoint*> mPoints; // Container for map points in the simulation

    // Configuration Flags
    bool mUseOrbSlam;
    std::string mVocPath;
    bool mTrackImages;
    bool mLoadMap;
    std::string mLoadMapPath;
    ORB_SLAM2::System *mSystem;

    // Container for current frame map points and newly observed points
    std::vector<OfflineMapPoint*> mCurrentFramePoints;
    std::vector<OfflineMapPoint*> mPointsSeen;
    std::vector<OfflineMapPoint*> mNewPointsSeen;

    std::vector<OfflineMapPoint*> mCloudScanned; // Container for the scanned point cloud

    // Result Point Data
    cv::Point3d mResultPoint;
    cv::Point3d mRealResultPoint;

    std::string mSimulatorViewerTitle;
    std::string mResultsWindowTitle;

    // Visualization Parameters
    double mPointSize;
    double mResultsPointSize;

    // Scaling factor for rotation and movement
    double mRotateScale;
    double mMovingScale;

    // Camera and Pose Variables
    pangolin::OpenGlRenderState mS_cam;
    pangolin::View mD_cam;

    pangolin::OpenGlMatrix mTcw; // Transformation matrix for world to camera
    pangolin::OpenGlMatrix mTwc; // Transformation matrix for camera to world

    bool mFollow; // Flag indicating whether the camera is following

    //  Flags for Camera Movement and Rotation
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

    // Viewpoint Parameters
    float mViewpointX;
    float mViewpointY;
    float mViewpointZ;
    float mViewpointF;

    // Camera Position and Orientation Variables
    cv::Point3d mStartPosition;
    cv::Point3d mCurrentPosition;

    double mStartYaw;
    double mCurrentYaw;
    double mStartPitch;
    double mCurrentPitch;
    double mStartRoll;
    double mCurrentRoll;

    // File Paths
    std::string mSimulatorPath;
    std::string mCloudPointPath;
    std::string mConfigPath;

    bool mCloseResults; // Flag indicating whether to close the results window

};

#endif //ORB_SLAM2_SIMULATOR_H
