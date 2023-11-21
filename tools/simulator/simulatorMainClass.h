//
// Created by dean on 11/11/23.
//

#ifndef ORB_SLAM2_SIMULATORMAINCLASS_H
#define ORB_SLAM2_SIMULATORMAINCLASS_H

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
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <unordered_set>
#include <utility>
#include "../slam/include/System.h"

// *  @class rootSimulator
// *  @brief This abstract class provides the base abilities of simulation environment for virtual robotic navigation and mapping.

class rootSimulator {
public:

    vector<double> curr_cam_pose;

    rootSimulator() : stopFlag(false), ready(false), saveMapSignal(false),
                                     track(false),
                                     isSaveMap(false),
                                     cull_backfaces(false),
                                     viewportDesiredSize(640, 480){}
    /**
 * Constructs a Simulator instance with specified parameters, and loads the ORBSLAM2 object.
 * @param saveMap: A boolean to determine whether the generated SLAM map should be saved or not. Defaults to false if not specified.
     *


    /**
 *Starts the 3D model viewer (pangolin), and wait for the user or code signal to start sending the view to the ORBSLAM2 object
 * */
    virtual std::thread run() = 0;


    /**
 * sample the simulator state, this changes from false to true once the model is loaded
 * @return is the simulator is ready
 * */
    bool isReady() const{return ready;}

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

    std::shared_ptr<ORB_SLAM2::System> getSLAM() { return SLAM; }

    ~rootSimulator();

protected:
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


    std::shared_ptr<ORB_SLAM2::System> SLAM;

protected:

//    std::shared_ptr<pangolin::OpenGlRenderState> shared_cam;

    pangolin::OpenGlRenderState s_cam;
    Eigen::Matrix3d K;
    ORB_SLAM2::ORBextractor *orbExtractor{};
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
    bool trackImages{};
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    std::mutex locationLock;
    std::mutex simulatorLock;

    double speedFactor{};

    void saveMap(const std::string& prefix = "");

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

    void static applyRollRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void faster();

    void slower();

    //special section
//    void static applyUpModelCamSpecial(std::shared_ptr<pangolin::OpenGlRenderState> &cam, double value);
//    void static applyYawRotationToModelCamSpecial(std::shared_ptr<pangolin::OpenGlRenderState> &cam, double value);

private:
    //None
};


#endif //ORB_SLAM2_SIMULATORMAINCLASS_H
