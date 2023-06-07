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

class Simulator {
public:
    Simulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo,
              bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
              std::string mapLoadPath = "../slamMaps/example.bin",
              double movementFactor = 0.01, int nFeatures = 2000, float fScaleFactor = 1.2, int fIniThFAST = 20,
              int fMinThFAST = 10, int nLevels = 8,
              std::string vocPath = "../Vocabulary/ORBvoc.txt");

    std::thread run();

    bool ready;

    void command(std::string &command, int intervalUsleep = 50000,
                 double fps = 30.0,
                 int totalCommandTimeInSeconds = 1);

    void stop() { stopFlag = true; }
    void setTrack(bool value){track = value;}

private:
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
    pangolin::OpenGlRenderState s_cam;
    Eigen::Matrix3d K;
    ORB_SLAM2::ORBextractor *orbExtractor;
    std::string simulatorOutputDir;
    bool stopFlag;
    bool saveMapSignal;
    bool track;
    double movementFactor{};
    std::string modelPath;
    std::string modelTextureNameToAlignTo;
    std::vector<Eigen::Vector3d> Picks_w;
    bool isSaveMap;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;

    void simulatorRunThread();

    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);

    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);

    void saveMap(std::string prefix = "");

    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &,double &)> &func,
                             double value, int intervalUsleep,
                             double fps,
                             int totalCommandTimeInSeconds);

    void
    applyCommand(std::string &command, double value,
                 int intervalUsleep,
                 double fps,
                 int totalCommandTimeInSeconds);

    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam,double value);

    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam,double value);

    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam,double value);

    void static applyUpModelCam(pangolin::OpenGlRenderState &cam,double value);

    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam,double value);
};


#endif //ORB_SLAM2_SIMULATOR_H
