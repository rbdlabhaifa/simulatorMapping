//
// Created by tzuk on 6/4/23.
//

#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>
#include <functional>
#include <pangolin/display/display.h>

#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>
#include "ORBextractor.h"
#include "System.h"
#include <Eigen/SVD>
#include <filesystem>
#include "include/run_model/TextureShader.h"
/**
 *  @class Simulator
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
class Simulator
{
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
   Simulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo, bool trackImages = true,
             bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
             std::string mapLoadPath = "../slamMaps/example.bin",
             double movementFactor = 0.01,
             std::string vocPath = "../Vocabulary/ORBvoc.txt");

   /**
    *Starts the 3D model viewer (pangolin), and wait for the user or code signal to start sending the view to the ORBSLAM2 object
    * */
   std::thread run();
   /**
    * sample the simulator state, this changes from false to true once the model is loaded
    * @return is the simulator is ready
    * */
   bool isReady() { return ready; }

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
    *
    * @param command A string specifying the command to be executed.
    * @param intervalUsleep Optional parameter setting the sleep interval between command execution in microseconds. Default is 50000 microseconds.
    * @param fps Optional parameter defining the frames per second rate for visualization. Default is 30.0.
    * @param totalCommandTimeInSeconds Optional parameter setting the total duration for the command execution in seconds. Default is 1 second.
    *
    * This method enables the users to navigate the virtual robot in the simulation by executing specific commands.
    */
   void command(std::string &command, int intervalUsleep = 50,
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
   void setTrack(bool value) { track = value; }
   /**
   * @brief this function is the main function that runs the simulator in a separate thread
   *
   */
   void simulatorRunThread();
private:
   /**
    * @brief A map for controlling the virtual robot's actions.
    *
    * This property is an unordered map where:
    * - The key is a string representing a command for the virtual robot.
    * - The value is a boolean indicating whether the command is executable (true) or not (false).
    */
   std::unordered_map<std::string, bool> commandMap = {
       {"cw", true},
       {"ccw", true},
       {"forward", true},
       {"back", true},
       {"right", true},
       {"up", true},
       {"down", true},
       {"left", true},
       {"flip", false},
       {"rc", false}};

   std::shared_ptr<ORB_SLAM2::System> SLAM; // will be used to store and manage the memory of the ORB_SLAM2::System object
   pangolin::OpenGlRenderState s_cam;
   Eigen::Matrix3d K;   // Eigen matrix that represent the camera intrinsic matrix
   std::shared_ptr < ORB_SLAM2::ORBextractor> orbExtractor; //shared pinter from type shared_ptr ORB_SLAM2::ORBextractor 
   std::string simulatorOutputDir;  // simulatorOutputDirPath: A string representing the output directory for saving SLAM maps.
   bool stopFlag;
   bool ready;  //saves true if the model is loaded else false  

   bool saveMapSignal;
   bool track;  // use for enabling or disabling the ORBSLAM process
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
   std::mutex locationLock; //lock to avoid race condition 

  
   /**
   * @brief The extractSurface function extracts the surface data (vertices) of a specific part of a 3D model and and stores the data in the surface matrix
   * @param modelGeometry represents a 3D model geometry, which includes information about vertices, normals, texture coordinates, and other attributes of a 3D model
   * @param modelTextureNameToAlignTo contain the model Texture Name associated with a 3D model
   * @param surface saves the surface of the some part of the 3D model defined by the texture name
   
   
   */
   void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                       Eigen::MatrixXf &surface);
   /**
   * @brief the function aligns the model view camera to face a surface on the model geometry
   * @param modelGeometry represents a 3D model geometry, which includes information about vertices, normals, texture coordinates, and other attributes of a 3D model
   * @param modelTextureNameToAlignTo contain the model Texture Name associated with a 3D model
   
   */
   void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);
    /**
    * @brief  saving the Map data in the CSV file 
    * @param prefix string that holds the prefix for the CSV file name
    
    */
   void saveMap(std::string prefix = "");
   /**
   * @brief this function function execute a callback function func at regular intervals over a some time duration 
   * @param func callback function 
   * @param value represents the initial value to calulate intervalValue
   * @param intervalUsleep delay is used to regulate the speed at which the intervals are executed
   * @param fps  parameter defining the frames per second rate for visualization.
   * @param totalCommandTimeInSeconds Optional parameter setting the total duration for the command execution in seconds
   
   */
   void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                            double value, int intervalUsleep,
                            double fps,
                            int totalCommandTimeInSeconds);
   /**
   * @brief Check the type of the command call the appropriate intervalOverCommand function based on the command.
   * @param command string specifies the type of camera movement 
   * @param value  represents the initial value to calulate intervalValue
   * @param intervalUsleep delay is used to regulate the speed at which the intervals are execu
   * @param fps parameter defining the frames per second rate for visualization.
   * @param totalCommandTimeInSeconds Optional parameter setting the total duration for the command execution in seconds
   */
   void
   applyCommand(std::string &command, double value,
                int intervalUsleep,
                double fps,
                int totalCommandTimeInSeconds);
   /**
    * @brief the function move the camera forward or backward (z axis ) according to value
    * @param cam represents the rendering state of an OpenGL camera view
    * @param value holds the amout of how much should the camera move forward or backward

    */
   void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);
   /**
    * @brief the function move the camera right or left (z axis ) according to value
    * @param cam represents the rendering state of an OpenGL camera view
    * @param value holds the amout of how much should the camera move right or left

    */
   void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);
   /**
    * @brief function applies a yaw rotation to the model view camera
    * @param cam represents the rendering state of an OpenGL camera view
    * @param value holds the amout of how much should the camera move clockwise or counter clockwise 

    */
   void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);
   /**
    * @brief the function move the camera up or down (y axis ) according to value 
    * @param cam represents the rendering state of an OpenGL camera view
    * @param value holds the amout of how much should the camera move up or down 

    */
   void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);
   /**
  * @brief the function applies pitch rotation to the model view camera
  * @param cam represents the rendering state of an OpenGL camera view
  * @param value hold the angle value

  */
   void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);
};

#endif // ORB_SLAM2_SIMULATOR_H
