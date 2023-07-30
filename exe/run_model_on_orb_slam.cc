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

void applyPitchRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

// the function receives both a vector of seen points and new points 
// and draws them on their own screen (these are the red dots)
// not currently used.
void drawPoints(std::vector<cv::Point3d> &seen_points, std::vector<cv::Point3d> &new_points_seen) {
    // putting the general settings of the project a json file named data
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];

    // choosing the colour and appearance of the seen_points and drawing them
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (auto &point: seen_points) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();

    // choosing the colour and appearance of the new_seen_points and drawing them
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (auto &point: new_points_seen) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();
}

// this function receives a keypoint, a mat array of depths and the camera state (the K variable is not used)
// the function returns a 3d point (based on depth and camera position) that represents the 2d point input
// not currently used
cv::Point3f convert2Dto3D(cv::Point2f &keypoint, const cv::Mat &K, const cv::Mat &depth,
                          const pangolin::OpenGlRenderState &cam_state) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];

    // set the camera position and projection over the model in modelview and projection
    glGetIntegerv(GL_VIEWPORT, viewport);
    for (int i = 0; i < 16; ++i) {
        modelview[i] = cam_state.GetModelViewMatrix().m[i];
        projection[i] = cam_state.GetProjectionMatrix().m[i];
    }

    GLdouble x, y, z;
    GLdouble worldX, worldY, worldZ;

    x = keypoint.x;
    y = (float) viewport[3] - keypoint.y;                          // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ); // converts the point to 3d with regard to camera position and other variables
                                                                                        // and puts the output in &worldX, &worldY, &worldZ

    return cv::Point3f((float) worldX, (float) worldY, (float) worldZ);
}

// filters the 3d points found with orb slam and saves them using mapNumber for the name of the directory
// the info of the map points and the coordinats of the keypoints are saved to pointData
void saveMap(int mapNumber, std::string &simulatorOutputDir, ORB_SLAM2::System *SLAM) {
    std::ofstream pointData; // the data of the points will be saved to this object
    std::unordered_set<int> seen_frames; // useless

    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) { // go over each point in the map created by SLAM
        if (p != nullptr && !p->isBad()) { // if point is "bad" (decided by orbSlam) then don't save it
            auto point = p->GetWorldPos(); // get the position of the point
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point); // convert point to a matrix and put it in vector
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F); // fill worldPos with zeros
            
            // set worldPose to be the position of the point after conversion
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();


            p->UpdateNormalAndDepth(); // update fields of depth of point in relation to camera and the normal of the point
            cv::Mat Pn = p->GetNormal(); // the normal is the average of the normalized distance vectors between the camera 
                                         // (at different positions) and the keypoints relating to the same map point
            Pn.convertTo(Pn, CV_64F); // convert normal to grayscale

            // add the point and important imformation about it to PointData file (like position in space, normal and more)
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","
                      << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
                
            std::map<ORB_SLAM2::KeyFrame *, size_t> observations = p->GetObservations(); // observations will contain all the key frames that p was observed in
            for (auto obs: observations) { // go over each observation (key frame) that p was observed in
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;
                // if frame not empty than add the keypoint's coordinats corrosponding to the mappoint to pointData
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

// this function receives keypoints and a path to a file
// the function saves the points' coordinates to the file in a csv format
//currently not used
void saveKeypointsToCSV(const std::vector<cv::Point3d> &keypoints, const std::string &filename) {
    std::ofstream csv_file(filename);

    for (const auto &keypoint: keypoints) {
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl;
    }

    csv_file.close();
}

// currently not used
void HandleKeyboardInput(unsigned char key, int x, int y) {
    // Handle WASD key events
    // Update camera position based on key inputs
}

// the function receives the path to the settings in generalSettings.json, whether to stop (stopFlag), a camera object and a bool called ready
// the function runs both the orb slam system and the navigation in the model and the communication between them
void runModelAndOrbSlam(std::string &settingPath, bool *stopFlag, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam,
                        bool *ready) {
    // putting the general settings into data.json
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // storing and extracting the camera and orb slam settings in fSettings
    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    // initializing the position and resolution of the camera
    Eigen::Matrix3d K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    Eigen::Vector2i viewport_desired_size(640, 480);

    cv::Mat img;

    // setting other characteristics of orb slam (like maximum number of keyPoints)
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // an object from orb slam capable of extracting the keypoints from the video
    ORB_SLAM2::ORBextractor *orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST,
                                                                        fMinThFAST);

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;

    char currentDirPath[256]; // useless, not used again
    getcwd(currentDirPath, 256);

    // get the current time to put in the directory name
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);


    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"]; // the camera and orb slam paremeters and the camera's intial position
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"]; // align the camera with the floor (parallel to it)
    std::string videoPath = data["offlineVideoTestPath"]; // path the offline video location, not currently used
    bool loadMap = data["loadMap"]; // whether to load an existing map created or not
    double movementFactor = data["movementFactor"]; // adjust the moving speed of the drone
    bool isSavingMap = data["saveMap"]; // whether to save the map created or not
    std::string loadMapPath = data["loadMapPath"]; // path to pre-made map in case we don't start from scratch
    std::string simulatorOutputDirPath = data["simulatorOutputDir"]; // the path to save the points to 
    std::string simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir); // create a directory in the path specified
    ORB_SLAM2::System SLAM = ORB_SLAM2::System(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, false, loadMap,
                                               loadMapPath,
                                               true); // create an ORB_SLAM2 system instance do to the tracking (finding keyPoints, mapPoints and more)
    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewport_desired_size[0], viewport_desired_size[1]);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    s_cam = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2),
                                       K(1, 2), NEAR_PLANE, FAR_PLANE),
            pangolin::ModelViewLookAt(0.1, -0.1, 0.3, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)); // the first 3 value are meaningless because we change them later

    // Create Interactive View in window
    pangolin::Handler3D handler(*s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float) -viewport_desired_size[0] / (float) viewport_desired_size[1]))
            .SetHandler(&handler);

    // Load Geometry asynchronously
    std::string model_path = data["modelPath"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path);
    std::vector<Eigen::Vector3<unsigned int>> floorIndices;
    for (auto &o: geom_to_load.objects) {
        if (o.first == modelTextureNameToAlignTo) {
            const auto &it_vert = o.second.attributes.find("vertex_indices");
            if (it_vert != o.second.attributes.end()) {
                const auto &vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);
                for (size_t i = 0; i < vs.h; ++i) {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i));
                    floorIndices.emplace_back(v);
                }
            }
        }
    }
    Eigen::MatrixXf floor(floorIndices.size() * 3, 3);
    int currentIndex = 0;
    for (const auto &b: geom_to_load.buffers) {
        const auto &it_vert = b.second.attributes.find("vertex");
        if (it_vert != b.second.attributes.end()) {
            const auto &vs = std::get<pangolin::Image<float>>(it_vert->second);
            for (auto &row: floorIndices) {
                for (auto &i: row) {
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i));
                    floor.row(currentIndex++) = v;
                }
            }
        }
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(floor, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.computeV();
    Eigen::Vector3f v = svd.matrixV().col(2);
    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,
                                               -1.0,
                                               pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE);
    s_cam->SetModelViewMatrix(mvm);
    s_cam->SetProjectionMatrix(proj);
    applyPitchRotationToModelCam(s_cam, -90);
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

    cv::VideoWriter writer;
    writer.open(simulatorOutputDir + "/scan.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(viewport_desired_size[0], viewport_desired_size[1]), true);

    std::vector<cv::Point3d> seenPoints{}; // not used

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
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGB24");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGB, GL_UNSIGNED_BYTE, buffer.ptr);

            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC3, buffer.ptr);
            cv::cvtColor(imgBuffer, img, cv::COLOR_RGB2GRAY);
            img.convertTo(img, CV_8UC1);
            cv::flip(img, img, 0);
            cv::flip(imgBuffer, imgBuffer, 0);

            writer.write(imgBuffer);

            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
            double timestamp = value.count() / 1000.0;

           // Detect keypoints
           std::vector<cv::KeyPoint> keypoints;
           cv::Mat descriptors;
           (*orbExtractor)(img, cv::Mat(), keypoints, descriptors); // detect keyPoints and descriptors and save them to keypoints and descriptors
            SLAM.TrackMonocular(descriptors, keypoints, timestamp); // finds map points accoring to keypoints, descriptors and time stamp

            s_cam->Apply();

            glDisable(GL_CULL_FACE);
        }

        pangolin::FinishFrame();
    }
    writer.release();
    saveMap(0, simulatorOutputDir, &SLAM); //save the map information in simulatorOutputDir
    SLAM.SaveMap(simulatorOutputDir + "/simulatorCloudPoint.bin");
    SLAM.Shutdown();
}

// the function gets the position of the camera and a value
// the function moves the camera up by value
void applyUpModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(1, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

// the function gets the position of the camera and a value
// the function moves the camera forward by value
void applyForwardToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(2, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

// the function gets the position of the camera and a value
// the function moves the camera right by value
void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(0, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

// the function gets the position of the camera and a value
// the function turns the camera left or right by value (if positive - right, negative - left)
void applyYawRotationToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    s_cam->SetModelViewMatrix(newModelView);
}

// the function gets the position of the camera and a value
// the function turns the camera up or down by value (if positive - up, negative - down)
void applyPitchRotationToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, c, -s,
            0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();;
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    s_cam->SetModelViewMatrix(newModelView);
}

// not used
// the function receives a function (camera movement), the camera's position, a value for the movement,
// a time interval, the fps and the amount of time the command should take
// the function makes the camera move/turn by value in the appropriate direction (decided by the given function)
// over a time period of totalCommandTimeInSeconds at a consistent rate
void intervalOverCommand(const std::function<void(std::shared_ptr<pangolin::OpenGlRenderState> &, double &)> &func,
                         std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value, int intervalUsleep,
                         double fps,
                         int totalCommandTimeInSeconds) {
    double intervalValue = value / fps * totalCommandTimeInSeconds; // how much movement should be done each frame
    int intervalIndex = 0;
    while (intervalIndex <= fps * totalCommandTimeInSeconds) { // run the amount of frames necessary for the action
        usleep(intervalUsleep); // wait intervalUsleep
        func(s_cam, intervalValue); // move/turn by intervalValue
        intervalIndex += 1;
    }
}

// not used
// this function receives a string describing a command and calls the appropriate function to perform it
// allows for control over the camera
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
    std::string settingPath = Auxiliary::GetGeneralSettingsPath(); // the path general settings of the project
    bool stopFlag = false;
    bool ready = false;
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam = std::make_shared<pangolin::OpenGlRenderState>();
    std::thread t(runModelAndOrbSlam, std::ref(settingPath), &stopFlag, std::ref(s_cam), &ready); // a thread that calls to runModelAndOrbSlam and initiates the running of the program
//    int startSleepTime = 3;
//    std::cout << "wating " << startSleepTime << " to init commands " << std::endl;
    while (!ready) {
        usleep(500);
    }
//    applyPitchRotationToModelCam(s_cam, -20);
//    applyUpModelCam(s_cam, -0.5);
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
    t.join(); // wait until runModelAndOrbSlam finished running

    //    if (isSavingMap) {
    //        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    //    }
    //
    //    SLAM->Shutdown();

    return 0;
}
