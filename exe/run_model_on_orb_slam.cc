#include <thread>
#include <future>
#include <queue>

#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>

#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "include/run_model/TextureShader.h"
#include "include/Auxiliary.h"

#include "ORBextractor.h"
#include "System.h"

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <unordered_set>

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

void applyForwardToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyYawRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyUpModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void drawPoints(std::vector<cv::Point3d> &seen_points, std::vector<cv::Point3d> &new_points_seen) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (auto &point: seen_points) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (auto &point: new_points_seen) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();
}

cv::Point3f convert2Dto3D(cv::Point2f &keypoint, const cv::Mat &K, const cv::Mat &depth,
                          const pangolin::OpenGlRenderState &cam_state) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];

    glGetIntegerv(GL_VIEWPORT, viewport);
    for (int i = 0; i < 16; ++i) {
        modelview[i] = cam_state.GetModelViewMatrix().m[i];
        projection[i] = cam_state.GetProjectionMatrix().m[i];
    }

    GLdouble x, y, z;
    GLdouble worldX, worldY, worldZ;

    x = keypoint.x;
    y = (float) viewport[3] -
        keypoint.y;                           // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);

    return cv::Point3f((float) worldX, (float) worldY, (float) worldZ);
}

void saveMap(int mapNumber, std::string &simulatorOutputDir, ORB_SLAM2::System *SLAM) {
    std::ofstream pointData;
    std::unordered_set<int> seen_frames;

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        if (p != nullptr && !p->isBad()) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth();
            cv::Mat Pn = p->GetNormal();
            Pn.convertTo(Pn, CV_64F);
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","
                      << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
            std::map<ORB_SLAM2::KeyFrame *, size_t> observations = p->GetObservations();
            for (auto obs: observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;
                if (!currentFrame->image.empty()) {
                    size_t pointIndex = obs.second;
                    cv::KeyPoint keyPoint = currentFrame->mvKeysUn[pointIndex];
                    cv::Point2f featurePoint = keyPoint.pt;
                    pointData << "," << currentFrame->mnId << "," << featurePoint.x << "," << featurePoint.y;
                    // cv::Mat image = cv::imread(simulatorOutputDir + "frame_" + std::to_string(currentFrame->mnId) + ".png");
                    // cv::arrowedLine(image, featurePoint, cv::Point2f(featurePoint.x, featurePoint.y - 100), cv::Scalar(0, 0, 255), 2, 8, 0, 0.1);
                    // cv::imshow("image", image);
                    // cv::waitKey(0);
                }
            }
            pointData << std::endl;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

void saveKeypointsToCSV(const std::vector<cv::Point3d> &keypoints, const std::string &filename) {
    std::ofstream csv_file(filename);

    for (const auto &keypoint: keypoints) {
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl;
    }

    csv_file.close();
}

void HandleKeyboardInput(unsigned char key, int x, int y) {
    // Handle WASD key events
    // Update camera position based on key inputs
}

void runModelAndOrbSlam(std::string &settingPath, bool *stopFlag, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam,
                        bool *ready) {
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float viewpointX = fSettings["RunModel.ViewpointX"];
    float viewpointY = fSettings["RunModel.ViewpointY"];
    float viewpointZ = fSettings["RunModel.ViewpointZ"];

    Eigen::Matrix3d K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    Eigen::Vector2i viewport_desired_size(640, 480);

    cv::Mat img;

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    ORB_SLAM2::ORBextractor *orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST,
                                                                        fMinThFAST);

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;

    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    double movementFactor = data["movementFactor"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string simulatorOutputDirPath = data["simulatorOutputDir"];
    std::string simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
    ORB_SLAM2::System SLAM = ORB_SLAM2::System(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, loadMap,
                                               loadMapPath,
                                               true);

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewport_desired_size[0], viewport_desired_size[1]);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    s_cam = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2),
                                       K(1, 2), NEAR_PLANE, FAR_PLANE),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY));

    // Create Interactive View in window
    pangolin::Handler3D handler(*s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float) -viewport_desired_size[0] / (float) viewport_desired_size[1]))
            .SetHandler(&handler);

    // Load Geometry asynchronously
    std::string model_path = data["modelPath"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path);

    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);
    const auto mvm = pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE);
    s_cam->SetModelViewMatrix(mvm);
    s_cam->SetProjectionMatrix(proj);
    pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
    for (auto &buffer: geomToRender.buffers) {
        buffer.second.attributes.erase("normal");
    }
    // Render tree for holding object position
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        default_prog.Link();
    };
    LoadProgram();
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });

    // Show axis and axis planes
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('k', [&]() { *stopFlag = !*stopFlag; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });
    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 1); });
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -1); });
    pangolin::RegisterKeyPressCallback('r', [&]() {
        applyUpModelCam(s_cam, -movementFactor);
    });// ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor); });
    Eigen::Vector3d Pick_w = handler.Selected_P_w();
    std::vector<Eigen::Vector3d> Picks_w;
    std::vector<cv::Point3d> seenPoints{};
    while (!pangolin::ShouldQuit() && !*stopFlag) {
        *ready = true;
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                      << std::endl;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Load any pending geometry to the GPU.
        if (d_cam.IsShown()) {
            d_cam.Activate();

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            default_prog.Bind();
            default_prog.SetUniform("KT_cw", s_cam->GetProjectionMatrix() * s_cam->GetModelViewMatrix());
            pangolin::GlDraw(default_prog, geomToRender, nullptr);
            default_prog.Unbind();

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size);

            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY);
            img.convertTo(img, CV_8UC1);
            cv::flip(img, img, 0);

            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
            double timestamp = value.count() / 1000.0;

            SLAM.TrackMonocular(img, timestamp);
//            // Detect keypoints
//            std::vector<cv::KeyPoint> keypoints;
//            cv::Mat descriptors;
//            (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);
//
//            // Save the x and y values of the keypoints to a vector
//            std::vector<cv::Point2f> keypoint_positions;
//            for (const auto &keypoint: keypoints) {
//                cv::Point2f position = cv::Point2f((float) keypoint.pt.x, (float) (keypoint.pt.y));
//                keypoint_positions.push_back(position);
//            }
//
//            cv::Mat depth(viewport_size[3], viewport_size[2], CV_32FC1);
//            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_DEPTH_COMPONENT, GL_FLOAT, depth.data);
//
//            // Convert keypoints pixels to keypoints 3d points
//            std::vector<cv::Point3d> keypoint_points;
//            for (auto &keypoint: keypoint_positions) {
//                cv::Point3f point_float = convert2Dto3D(keypoint, K_cv, depth, *s_cam);
//                cv::Point3d point = cv::Point3d(point_float.x, point_float.y, point_float.z);
//                keypoint_points.push_back(point);
//            }
//
//             int frame_to_check = data["frameNumber"];
//             std::string keypoints_csv_path = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_orbs.csv";
//
//             saveKeypointsToCSV(keypoint_points, keypoints_csv_path);

            s_cam->Apply();

            glDisable(GL_CULL_FACE);

//             drawPoints(seenPoints, keypoint_points);
        }

        pangolin::FinishFrame();
    }
    saveMap(0, simulatorOutputDir, &SLAM);
    SLAM.SaveMap(simulatorOutputDir + "/simulatorCloudPoint.bin");
    SLAM.Shutdown();
}


void applyUpModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(1, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

void applyForwardToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(2, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(0, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

void applyYawRotationToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);
    
    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;
    Eigen::Matrix<double, 3, 3> pangolinR;
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pangolinR(i, j) = camMatrix(i, j);
        }
    }
    pangolinR = pangolinR * R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camMatrix(i, j) = pangolinR(i, j);
        }
    }
    s_cam->SetModelViewMatrix(camMatrix);
}

void applyPitchRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, c, -s,
            0, s, c;
    Eigen::Matrix<double, 3, 3> pangolinR;
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pangolinR(i, j) = camMatrix(i, j);
        }
    }
    pangolinR = pangolinR * R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camMatrix(i, j) = pangolinR(i, j);
        }
    }
    s_cam->SetModelViewMatrix(camMatrix);
}


void intervalOverCommand(const std::function<void(std::shared_ptr<pangolin::OpenGlRenderState> &, double &)> &func,
                         std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value, int intervalUsleep,
                         double fps,
                         int totalCommandTimeInSeconds) {
    double intervalValue = value / fps * totalCommandTimeInSeconds;
    int intervalIndex = 0;
    while (intervalIndex <= fps * totalCommandTimeInSeconds) {
        usleep(intervalUsleep);
        func(s_cam, intervalValue);
        intervalIndex += 1;
    }
}

void
applyCommand(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, std::string &command, double value,
             int intervalUsleep,
             double fps,
             int totalCommandTimeInSeconds) {

    if (command == "cw") {
        intervalOverCommand(&applyYawRotationToModelCam, s_cam, value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "ccw") {
        intervalOverCommand(&applyYawRotationToModelCam, s_cam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "forward") {
        intervalOverCommand(&applyForwardToModelCam, s_cam, value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "back") {
        intervalOverCommand(&applyForwardToModelCam, s_cam, -1 * value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "right") {
        intervalOverCommand(&applyRightToModelCam, s_cam, -1 * value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "left") {
        intervalOverCommand(&applyRightToModelCam, s_cam, value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "up") {
        intervalOverCommand(&applyUpModelCam, s_cam, -1 * value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else if (command == "down") {
        intervalOverCommand(&applyUpModelCam, s_cam, value, intervalUsleep, fps, totalCommandTimeInSeconds);
    }

}

int main(int argc, char **argv) {
    std::unordered_map<std::string, bool> commandMap = {
            {"cw",      true},
            {"ccw",     true},
            {"forward", true},
            {"right",   true},
            {"up",      true},
            {"down",    true},
            {"left",    true},
            {"flip",    false},
            {"rc",      false}};
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    bool stopFlag = false;
    bool ready = false;
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam = std::make_shared<pangolin::OpenGlRenderState>();
    std::thread t(runModelAndOrbSlam, std::ref(settingPath), &stopFlag, std::ref(s_cam), &ready);
//    int startSleepTime = 3;
//    std::cout << "wating " << startSleepTime << " to init commands " << std::endl;
//    while (!ready) {
//        usleep(500);
//    }
    applyPitchRotationToModelCam(s_cam, 25);
    applyUpModelCam(s_cam, -1);
//    sleep(startSleepTime);
//    int intervalUsleep = 50000;
//    std::vector<std::string> commnads = {"cw 25", "forward 30", "back 30", "cw 30"};
//    int currentYaw = 0;
//    int angle = 10;
//    for (int i = 0; i < std::ceil(360 / angle); i++) {
//        std::string c = "forward";
//        double value = 0.50;
//        applyCommand(s_cam, c, value, intervalUsleep, 30.0, 1);
//        usleep(500000);
//        c = "back";
//        value = 0.50;
//        applyCommand(s_cam, c, value, intervalUsleep, 30.0, 1);
//        usleep(500000);
//        c = "cw";
//        value = angle;
//        applyCommand(s_cam, c, value, intervalUsleep, 30.0, 1);
//        sleep(1);
//
//    }
//    stopFlag = true;
    t.join();

    //    if (isSavingMap) {
    //        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    //    }
    //
    //    SLAM->Shutdown();

    return 0;
}
