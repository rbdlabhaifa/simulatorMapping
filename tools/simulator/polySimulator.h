//
// Created by Dean on 9/30/23.
//

#ifndef ORB_SLAM2_SIMULATOR_H_2
#define ORB_SLAM2_SIMULATOR_H_2

#define NEAR_PLANE 0.1
#define FAR_PLANE 20
#define LOOK_AT_INTERPOLATE_STEP 15000
#define INTERPOLATE_STEP 15000
#define ALIGN_INTERPOLATION_STEP 15000

#include <memory>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>
#include <functional>

#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>
#include "ORBextractor.h"
#include "System.h"
#include <Eigen/SVD>
#include <filesystem>
#include "include/run_model/TextureShader.h"

//rearrange

#include <thread>
#include <future>
#include <queue>
#include <typeinfo>


#include "include/Auxiliary.h"

#include "ORBextractor.h"
#include "System.h"

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <unordered_set>
#include <utility>
#include "../slam/include/System.h"
#include "simulatorMainClass.h"

//end - rearrange

/*
 *  @class PolySimulator
 *  @brief This class inherit from rootSimulator class and created as a singleton class for the avoidance of memory leaks and overlapping uses.
 *  The PolySimulator class integrates ORB-SLAM2 and a 3D PolyCam model to create a comprehensive testing environment
 *  for SLAM (Simultaneous Localization and Mapping) and navigation algorithms. It negates the need for
 *  a physical robot interface.
 *
 *  With this simulator and a 3D blender model, a robot's movement can be simulated in a variety of virtual environments.
 *  It uses the Pangolin library to capture the current viewer image from the display window, which is then
 *  sent to ORB-SLAM2. This approach allows users to observe, analyze and improve the efficiency of their implemented
 *  algorithms in real-time with expansive and flexible datasets.
 *
 *  Features include:
 *  - Virtual robotic navigation in 3D environments using A,S,D,W,E,Q,R,F to move in the model
 *  - On-the-fly ORB-SLAM2 map generation and navigation from the 3D model, and extraction of current location and full map.
 *  - Real-time visualization using Pangolin
 */
class PolySimulator : public rootSimulator{
public:
    enum axis {posX, negX, posZ, negZ};

    axis chosenAxis;

    void RunPolyModel();

    //static function that allows defining the PolySimulator class as a singleton.
    static PolySimulator* getInstance(bool _loadCustomMap) {
        static PolySimulator *classInstance = nullptr;
        if (classInstance == nullptr){
            std::cout << "New instance of simulator has been created by the user" << std::endl;
            classInstance = new PolySimulator(_loadCustomMap);
        }
        else {
            std::cout << "An instance of the simulator is already created and being returned" << std::endl;
        }
        return classInstance;
    }

    //overload operators, copy constructor and destructors.
   ~PolySimulator();
    PolySimulator(const PolySimulator&) = delete;
    PolySimulator& operator=(const PolySimulator&) = delete;

/**
 *Starts the 3D model viewer (pangolin), and wait for the user or code signal to start sending the view to the ORBSLAM2 object
 * */
    std::thread run() override;

   //New Functionalities

    void setInPlaceFlag(bool val);

    bool isSetInPlace() const;
   
    void startRolling();

    bool isTracking();

   //This function returns the value of the loop closer flag of ORB-SLAM2 System
    bool slamFinishedLoopCloser(){
      return SLAM->GetLoopClosing()->getLoopClosed();
   }

   //Getting the Information whether ORB-SLAM system finished its local map release.
    bool slamFinishMapReleasing(){
       return SLAM->GetLoopClosing()->getMapReleased();
   }

   //Setting SLAM-SYS into localization mode.
    void setSlamIntoLocalizationMode(){
       SLAM->ActivateLocalizationMode();
   }

    pangolin::View& getSimulatorD_Cam(){
        return this->simD_CAM;
   }

    //SYSTEM_NOT_READY=-1,
    //NO_IMAGES_YET=0,
    //NOT_INITIALIZED=1,
    //OK=2,
    //LOST=3
    int getSlamState(){
      return this->SLAM->GetTracker()->mState;
   }

    void setFoundExitFlag(){
      this->foundExitPoints = true;
   }

    void setPointsToRender(vector<vector<double>> pointsToRend){
        pointsToRender = std::move(pointsToRend);
   }

    void renderPoints(bool showDrone = true);

    std::vector<vector<double>> getCurrentMapPoint();

    [[maybe_unused]] void addPointToRender(const vector<double>& p){
      this->pointsToRender.push_back(p);
   }

    static void generateGLPoint(vector<GLfloat> colors, GLfloat size, vector<double> point);

    static void generateGLLine(vector<GLfloat> colors, vector<double> point1, vector<double> point2);

    void alignCameraWithFloor(bool withAnimation, PolySimulator::axis axis); //align according to current drone location

    [[maybe_unused]] bool rotateAndSlam(bool &down_up_flag, double& timestamp, double step);

    void printCurrentLocation();

    std::vector<double> getDroneLocation();

    static vector<vector<double>> fromMapPointsToDoubleVec(vector<ORB_SLAM2::MapPoint*> points);

    void setCamera();

    pangolin::View& setDCAM(pangolin::Handler3D *handler);

    void definePangolinRegKeys();

    cv::Mat getLastDroneView(){
       return this->lastImageCap;
   }

    void CaptureFrame(pangolin::View& camera_view);

    void printCurrentSCam(){
       this->locationLock.lock();
       auto s_cam_mat = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix().Inverse());
       this->locationLock.unlock();
       std::cout << s_cam_mat << std::endl;
   }

    void addDroneLocation(const std::vector<double>& currentLocation);

    void renderDroneLocations();

    void showDroneLocations(){
        this->renderDronePathPoints = !this->renderDronePathPoints;
    }

    void shouldSaveDroneLocations(){
       this->addPointsToRender = !this->addPointsToRender;
    }

    void setStopFlag(bool b) {
        this->stopFlag = b;
    }

    void reflectViewBool(){
       this->shouldReflectView = !this->shouldReflectView;
    }

    bool getReflectMode() const{
       return this->shouldReflectView;
   }
    const cv::Mat& getCurrentViewFromUAV();

    const cv::Mat* getCurrentViewPointerBW(){
       this->simulatorLock.lock();
       cv::Mat *temp = &this->currentImageBW;
       this->simulatorLock.unlock();
       return temp;
   }

    const cv::Mat* getCurrentViewPointerColored(){
        this->simulatorLock.lock();
        cv::Mat *temp = &this->currentImageColored;
        this->simulatorLock.unlock();
        return temp;
    }

    //anyways return as const
    std::vector<std::vector<double>> getDronePath(){
       return this->dronePath;
    }

    void setDronePath(vector<vector<double>> points){
        this->dronePath = points;
    }

    void slamShouldSaveMap(){
        isSaveMap = !isSaveMap;
    }

    void lookAtPoint(vector<double> whereToLook);

    void moveToXYZ(vector<double> whereToMove);

private:
    cv::Mat lastImageCap;
    cv::Mat currentImageColored;
    cv::Mat currentImageBW;

    float viewpointX;
    float viewpointY;
    float viewpointZ;

    bool run_exit_room_algo = false;
    bool startRoll = false;
    bool setInPlace = false;
    bool foundExitPoints = false;
    bool loadCustomMap;
    bool addPointsToRender = false;
    bool renderDronePathPoints = false;
    bool shouldReflectView = false;

    std::string ORBSLAMConfigFile;
    std::string vocPath;

    vector<vector<double>> pointsToRender;
    vector<vector<double>> dronePath;

    pangolin::View simD_CAM;

    //singleton class constructor, copy const and assign operator overloaded.
    explicit PolySimulator(bool _loadCustomMap);
};


#endif
