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
class Simulator {
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
 * trackImages
 *
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

    std::vector<ORB_SLAM2::MapPoint*> getCurrentMap() { return SLAM->GetMap()->GetAllMapPoints(); };
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
    void command(std::string& command, int intervalUsleep = 50000,
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
            {"rc",      false} };
    std::shared_ptr<ORB_SLAM2::System> SLAM;
    pangolin::OpenGlRenderState s_cam;
    Eigen::Matrix3d K;
    ORB_SLAM2::ORBextractor* orbExtractor;
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
    std::mutex locationLock;
    /*unction is a core function that runs in a separate thread to simulate a 3D environment using the Pangolin library
     and perform operations related to SLAM (Simultaneous Localization and Mapping)*/
    void simulatorRunThread();



    // extract surface information from a given 3D model geometry. 
    //parmeters:
    // modelGeometry: constant reference to a pangolin::Geometry object (the input 3D model geometry)
    // modelTextureNameToAlignTo: which is a std::string representing the name of the texture to align the surface to
    // surface :  reference to an Eigen::MatrixXf (a 2D matrix of floats) that will store the extracted surface information.
    void extractSurface(const pangolin::Geometry& modelGeometry, std::string modelTextureNameToAlignTo,
        Eigen::MatrixXf& surface);



    //calculates the appropriate model-view and projection matrices to align the virtual camera's viewpoint with a specific surface in the 3D environment. 
    //parms:
    //modelGeometry (const pangolin::Geometry&): reference to a pangolin::Geometry object, which represents the 3D geometry of the model.
    //modelTextureNameToAlignTo(std::string) :  specifies the name of the texture associated with the surface to which the virtual camera should be aligned.
    //
    void alignModelViewPointToSurface(const pangolin::Geometry& modelGeometry, std::string modelTextureNameToAlignTo);


    /*
    *  saveMap function iterates through all the map points in the SLAM system, extracts and saves their 3D world positions, normal
    vectors, and observations (keypoint coordinates in corresponding keyframes) to a CSV file.

    */
    //parms
    //prefix: This parameter is of type std::string. It represents a prefix that will be used to create a filename for the output file
    void saveMap(std::string prefix = "");




    //parms
//func:  reference to a function that accepts an pangolin::OpenGlRenderState objectand and double  represents the command that excuted to the camera.

//value :  representing the  amount of change that will happen  by the command.

//intervalUsleep :representing the time  between each step of the command. controls how fast the command is executed.

//fps : representing the frames per second(FPS)

//totalCommandTimeInSeconds :  representing the time for  the command should be executed.
// 
    //intervalOverCommand is a utility function that helps control the application of a specific command over a specified time duration. 
    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState&, double&)>& func,
        double value, int intervalUsleep,
        double fps,
        int totalCommandTimeInSeconds);






    //parmetrs: command(witch command we want to excute)  |  value(number of steps (e.g number of steps to go back)  |  intervalUsleep(the time that the 
    //robot need to rest between two opreations  |   fps(updat the image every  fps time)  |    totalCommandTimeInSeconds(some command work for alimited peroid (this parmeter
    //is the limited peroid)))
    //The function checks the value of the command parameter to identify which type of camera movement should be applied.
    //Depending on the command, the function calls the appropriate intervalOverCommand function with the corresponding camera manipulation functionand the provided command 
    //parameters.The intervalOverCommand function is used to gradually apply the command in smaller increments over a specified time duration.
    void applyCommand(std::string& command, double value,
        int intervalUsleep,
        double fps,
        int totalCommandTimeInSeconds);






    //applyForwardToModelCam function adjusts the camera's position in the 3D environment by moving it forward along its viewing direction by a specified distance value. 
    //parameters:
    //pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
    // value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.
    void static applyForwardToModelCam(pangolin::OpenGlRenderState& cam, double value);



    //adjusts the camera's position in the 3D environment by moving it rightward along its right direction by a specified distance value. 
    //parms:
    //pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
    // value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.
    void static applyRightToModelCam(pangolin::OpenGlRenderState& cam, double value);





    //parms:
    //pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
    // value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.
    //adjusts the camera's orientation in the 3D environment by applying a yaw rotation around its vertical axis by a specified angle value. 
    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState& cam, double value);




    //  applyUpModelCam function adjusts the camera's position in the 3D environment by moving it upward by a specified distance value. 
    // 
    //cam:  reference to the camera's OpenGL render state
    //value:The amount of upward translation to apply to the camera's position. 
    void static applyUpModelCam(pangolin::OpenGlRenderState& cam, double value);



    //apply a pitch rotation to the camera view in the 3D model visualization.
    //parms
    //pangolin::OpenGlRenderState &cam: A reference to the camera's OpenGL render state, which contains the current model view matrix that represents the camera's position and orientation.
    // value: The amount of pitch rotation to be applied, specified in degrees.
    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState& cam, double value);
};


#endif //ORB_SLAM2_SIMULATOR_H
