#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <Eigen/SVD>
#include <filesystem>
#include <functional>
#include <pangolin/gl/glsl.h>
#include <pangolin/pangolin.h>
#include <pangolin/gl/glvbo.h>
#include <pangolin/display/display.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/display/pangolin_gl.h>
#include <pangolin/geometry/glgeometry.h>

#include "ORBextractor.h"
#include "System.h"

#include "RunModel/TextureShader.h"
#include "Auxiliary.h"



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
    Simulator(std::string ORBSLAMConfigFile, std::string model_path, bool alignModelToTexture, std::string modelTextureNameToAlignTo,
              Eigen::Vector3f startingPoint, bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
              std::string mapLoadPath = "../slamMaps/example.bin",
              double movementFactor = 0.01,
              double speedFactor = 1.0,
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

    bool startScanning() { return start; }

    /**
     * @brief Fetches the current location matrix from ORBSLAM2.
     *
     * @return A 4x4 location matrix where:
     * - The first 3x3 sub-matrix represents the rotation matrix.
     * - The last column represents the translation vector.
     * - The y-axis indicates the height in reverse (i.e., negative values correspond to upward direction).
     */
    cv::Mat getCurrentLocationSlam();
    /**
     * @brief Fetches the current location matrix from Pangolin.
     *
     * @return A 4x4 location matrix where:
     * - The first 3x3 sub-matrix represents the rotation matrix.
     * - The last column represents the translation vector.
     * - The y-axis indicates the height in reverse (i.e., negative values correspond to upward direction).

     */
    Eigen::Matrix4d getCurrentLocation();
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
    void command(const std::string &command, int intervalUsleep = 50000,
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
    void setTrack(bool value);

    void setSpeed(double speed);

    double getSpeed() const;

    void simulatorRunThread();

    std::shared_ptr<ORB_SLAM2::System> GetSLAM() { return SLAM; }

    void drawPoint(cv::Point3d point, float size, Eigen::Vector3d color);

    void cleanPoints();

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
    std::shared_ptr<ORB_SLAM2::System> SLAM;
    std::string vocPath;
    std::string ORBSLAMConfigFile;
    std::string mapLoadPath;
    bool loadMap;
    pangolin::OpenGlRenderState s_cam;
    Eigen::Matrix3d K;
    std::shared_ptr < ORB_SLAM2::ORBextractor> orbExtractor;
    std::string simulatorOutputDir;
    bool stopFlag;
    bool stopFlagSLAM;
    bool ready;
    bool start;
    bool initSlam;

    std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>> points;

    Eigen::Vector3f startingPoint;
    bool saveMapSignal;
    bool track;
    double movementFactor{};
    std::string modelPath;
    bool alignModelToTexture;
    std::string modelTextureNameToAlignTo;
    std::vector<Eigen::Vector3d> Picks_w;
    bool isSaveMap;
    bool isInitalized;
    bool isLocalized;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    cv::Mat currentImg;
    std::mutex locationLock;
    std::mutex imgLock;
    int numberOfFeatures;
    int trackingNumberOfFeatures;
    double speedFactor;

    void SLAMThread();

    bool feedSLAM(cv::Mat &img);

    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);

    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);

    void saveMap(std::string prefix = "");

    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                             double value, int intervalUsleep,
                             double fps,
                             int totalCommandTimeInSeconds);

    void
    applyCommand(std::string &command, double value,
                 int intervalUsleep,
                 double fps,
                 int totalCommandTimeInSeconds);

    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void slower();

    void faster();
};

#endif // ORB_SLAM2_SIMULATOR_H