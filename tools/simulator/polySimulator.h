//
// Created by Dean on 9/30/23.
//

#ifndef ORB_SLAM2_SIMULATOR_H_2
#define ORB_SLAM2_SIMULATOR_H_2

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

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
#include "../slam/include/System.h"

using std::vector;
//end - rearrange

/**
 *  @class PolySimulator
 *  @brief This class provides a simulation environment for virtual robotic navigation and mapping.
 *
 *  The Simulator class integrates ORBSLAM2 and a 3D blender model to create a comprehensive testing bed
 *  for SLAM (Simultaneous Localization and Mapping) and navigation algorithms. It negates the need for
 *  a physical robot interface.
 *
 *  With this simulator and a 3D blender model, a robot's movement can be simulated in a variety of virtual environments.
 *  It uses the Pangolin library to capture the current viewer image from the display window, which is then
 *  sent to ORBSLAM2. This approach allows users to observe, analyze and improve the efficiency of their implemented
 *  algorithms in real-time with expansive and flexible datasets.
 *
 *  Features include:
 *  - Virtual robotic navigation in 3D environments using A,S,D,W,E,Q,R,F to move in the model
 *  - On-the-fly ORBSLAM2 map generation and navigation from the 3D model, and extraction of current location and full map.
 *  - Real-time visualization using Pangolin
 */
class PolySimulator {
public:
    /**
 * Constructs a Simulator instance with specified parameters, and loads the ORBSLAM2 object.
 *
 * @param ORBSLAMConfigFile: A string representing the path to the ORBSLAM2 configuration file, an example can be found in the "config" folder in the project.
 *
 * @param model_path: A string representing the path to the 3D blender model used for simulation.
 *
 * @param modelTextureNameToAlignTo: A string representing the texture name used for alignment in the 3D model.
 *
 * @param saveMap: A boolean to determine whether the generated SLAM map should be saved or not. Defaults to false if not specified.
 *
 * @param simulatorOutputDirPath: A string representing the output directory for saving SLAM maps. Defaults to "../slamMaps/" if not specified.
 *
 * @param loadMap: A boolean to determine whether to load a previously saved SLAM map. Defaults to false if not specified.
 *
 * @param mapLoadPath: A string representing the path to a previously saved SLAM map to load. Defaults to "../slamMaps/example.bin" if not specified.
 *
 * @param movementFactor: A double value representing the movement speed in the simulator. Defaults to 0.01 if not specified.
 *
 * @param vocPath: A string representing the path to the ORBSLAM2 vocabulary file. Defaults to "../Vocabulary/ORBvoc.txt" if not specified.
 */
    vector<double> curr_cam_pose;
    std::shared_ptr<ORB_SLAM2::System> SLAM;
//   PolySimulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo,bool trackImages = true,
//              bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
//              std::string mapLoadPath = "../slamMaps/example.bin",
//              double movementFactor = 0.01,
//              std::string vocPath = "../Vocabulary/ORBvoc.txt",
//              double speedFactor=1.0);

    //static function that allows defining the PolySimulator class as a singleton.
    static PolySimulator* getInstance(bool _loadCustomMap) {
        static PolySimulator *classInstance = NULL;
        if (classInstance == NULL){
            std::cout << "New instance of simulator has been created by the user" << std::endl;
            classInstance = new PolySimulator(_loadCustomMap);
//            PolySimulator::systemLoaded = true;
        }
        else {
            std::cout << "An instance of the simulator is already created and being returned" << std::endl;
        }
        return classInstance;
    }

//    static void resetPolySimulator(bool _loadCustomMap){
//         = new PolySimulator(_loadCustomMap);
//    }
   ~PolySimulator();


/**
 *Starts the 3D model viewer (pangolin), and wait for the user or code signal to start sending the view to the ORBSLAM2 object
 * */
    std::thread run();
/**
 * sample the simulator state, this changes from false to true once the model is loaded
 * @return is the simulator is ready
 * */
    bool isReady(){return ready;}

    /**
 * @brief Fetches the current location matrix from ORBSLAM2.
 *
 * @return A 4x4 location matrix where:
 * - The first 3x3 sub-matrix represents the rotation matrix.
 * - The last column represents the translation vector.
 * - The y-axis indicates the height in reverse (i.e., negative values correspond to upward direction).
 */
    cv::Mat getCurrentLocation();
/**
 * @brief Retrieves the current map from ORBSLAM2.
 *
 * @return A vector of pointers to ORB_SLAM2::MapPoint objects.
 */
    std::vector<ORB_SLAM2::MapPoint *> getCurrentMap() { return SLAM->GetMap()->GetAllMapPoints(); };
/**
 * @brief Executes a specific command for controlling the virtual robot in the simulation, NOTICE: the available commands are in the commandMap object .

 * @param command A string specifying the command to be executed.
 * @param intervalUsleep Optional parameter setting the sleep interval between command execution in microseconds. Default is 50000 microseconds.
 * @param fps Optional parameter defining the frames per second rate for visualization. Default is 30.0.
 * @param totalCommandTimeInSeconds Optional parameter setting the total duration for the command execution in seconds. Default is 1 second.
 *
 * This method enables the users to navigate the virtual robot in the simulation by executing specific commands.
 */
    void command(std::string &command, int intervalUsleep = 50000,
                 double fps = 30.0,
                 int totalCommandTimeInSeconds = 1);
/**
 * @brief kills the run thread
 *
 */
    void stop() { stopFlag = true; }
/**
 * @brief enabling or disabling the ORBSLAM process.
 *
 */
    void setTrack() { track = !track; }

    void setSpeed(double speed);

    double getSpeed() const;

   //New Functionalities

   void setInPlaceFlag(bool val);

   bool isSetInPlace();
   
   void startRolling();

   bool isTracking();

   //This function returns the value of the loop closer flag of ORB-SLAM2 System
   bool slamFinishedLoopCloser(){
      return SLAM->GetLoopClosing()->isFinished();
   }

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
      pointsToRednder = pointsToRend;
   }

   void renderPoints(bool showDrone = true);

   std::vector<vector<double>> getCurrentMapPoint();

   void addPointToRender(vector<double> p){
      this->pointsToRednder.push_back(p);
   }

   void generateGLPoint(vector<GLfloat> colors, GLfloat size, vector<double> point);

   void generateGLLine(vector<GLfloat> colors, vector<double> point1, vector<double> point2);

   void alignCameraWithFloor(int x, int y, int z); //align according to x.y.z

   void alignCameraWithFloor(); //align according to current drone location

   bool rotateAndSlam(cv::Mat& img, bool &down_up_flag, double& timestamp, int &i, double step, double& aggregator);

   void printCurrentLocation();

   std::vector<double> getDroneLocation();

   vector<vector<double>> fromMapPointsToDoubleVec(vector<ORB_SLAM2::MapPoint*> points);

   void lookAtPoint(double x, double y, double z);

   void setCamera();

   pangolin::View& setDCAM(pangolin::Handler3D *handler);

   void definePangolinRegKeys();

   cv::Mat getLastDroneView(){
       return this->lastImageCap;
   }

   bool SlamCurrentView(cv::Mat img, double timestamp);

   void RunPolyModel();

   void moveToXYZ(std::vector<double> dest, int steps);

   void moveCameraByStep(double step_x, double step_y, double step_z, double step_size_x1, double step_size_y1, double step_size_z1, double step_size_x2, double step_size_y2, double step_size_z2, double step_size_x3, double step_size_y3, double step_size_z3);
   // void moveCameraByStep(double step_x, double step_y, double step_z);
   cv::Mat CaptureFrame(pangolin::View& camera_view);

   void printCurrentSCam(){
       this->locationLock.lock();
       auto s_cam_mat = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
       this->locationLock.unlock();
       std::cout << s_cam_mat << std::endl;
   }

   void addDroneLocation(std::vector<double> currentLocation);

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

    bool getReflectMode(){
       return this->shouldReflectView;
   }



private:
    /**
 * @brief A map for controlling the virtual robot's actions.
 *
 * This property is an unordered map where:
 * - The key is a string representing a command for the virtual robot.
 * - The value is a boolean indicating whether the command is executable (true) or not (false).
 */
   std::unordered_map<std::string, bool> commandMap = {
         {"cw",      true},
         {"ccw",     true},
         {"forward", true},
         {"back",    true},
         {"right",   true},
         {"up",      true},
         {"down",    true},
         {"left",    true},
         {"flip",    false},
         {"rc",      false}};


   pangolin::OpenGlRenderState s_cam;
   Eigen::Matrix3d K;
   ORB_SLAM2::ORBextractor *orbExtractor;
   std::string simulatorOutputDir;
   bool stopFlag;
   bool ready;

   bool saveMapSignal;
   bool track;
   double movementFactor{};
   std::string modelPath;
   std::string modelTextureNameToAlignTo;
   std::vector<Eigen::Vector3d> Picks_w;
   bool isSaveMap;
   bool trackImages;
   bool cull_backfaces;
   pangolin::GlSlProgram program;
   pangolin::GlGeometry geomToRender;
   Eigen::Vector2i viewportDesiredSize;
   cv::Mat Tcw;
   cv::Mat lastImageCap;
   std::mutex locationLock;
   std::mutex simulatorLock;

   double speedFactor;

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

   vector<vector<double>> pointsToRednder;
   vector<vector<double>> dronePath;

   pangolin::View simD_CAM;

   void saveMap(std::string prefix = "");

   void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                           double value, int intervalUsleep,
                           double fps,
                           int totalCommandTimeInSeconds);

   void applyCommand(std::string &command, double value,
               int intervalUsleep,
               double fps,
               int totalCommandTimeInSeconds);

   void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);

   void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);

   void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

   void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);

   void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

   void faster();

   void slower();

   //singleton class constructor, copy const and assign operator overloaded.
   explicit PolySimulator(bool _loadCustomMap);
   PolySimulator(const PolySimulator&) = delete;
   PolySimulator& operator=(const PolySimulator&) = delete;
};


#endif
