#include <thread>
#include <future>
#include <queue>

#include <typeinfo>

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

//vector<double> getAvgDistance3D(vector<ORB_SLAM2::MapPoint*>& currentFrameKP, pangolin::OpenGlMatrix& M);
// vector<double> getAvgDistance3D(set<ORB_SLAM2::MapPoint*> currentFrameKP, pangolin::OpenGlMatrix& M);

//void visualizeKeypoints(const std::vector<cv::KeyPoint>& keypoints, const cv::Size& canvasSize);

//double getAvgDistanceInAngle2D(vector<cv::KeyPoint> &currentFrameKP, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam);

void alignCameraWithFloor(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, int x, int y, int z);

//set<ORB_SLAM2::MapPoint*> getCurrentFrameKeyPoints(ORB_SLAM2::System &SLAM);

bool rotateAndSlam(double &max_ratio, pangolin::OpenGlMatrix& max_s_cam, vector<double> &dists, cv::Mat& img, bool &down_up_flag, double& timestamp, int &i, ORB_SLAM2::System &SLAM, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double step, double& aggregator);


void printCurrentLocation(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam);

void saveKeyPointsAndDescriptors(std::vector<cv::Mat> listOfDescriptors, std::vector< std::vector<cv::KeyPoint> > listOfKeyPoints, ORB_SLAM2::System *SLAM);



void applyForwardToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyYawRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyUpModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

void applyPitchRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

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

void applyCommand(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, std::string &command, double value,
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

void makeARound(std::string &settingPath, bool *stopFlag, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, bool *ready) {

    bool startRoll = false;
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

    //int nFeatures = fSettings["ORBextractor.nFeatures"];
    int nFeatures = 5000;
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    
                                                                    

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;
    bool track = false;
    bool finishSlamming = false;
    char currentDirPath[256];
    getcwd(currentDirPath, 256);

    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    std::string vocPath = data["VocabularyPath"];
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    double movementFactor = data["movementFactor"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string simulatorOutputDirPath = data["simulatorOutputDir"];
    std::string simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);

    ORB_SLAM2::System SLAM = ORB_SLAM2::System(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, false, "../slamMaps/example.bin", true);
    //ORB_SLAM2::ORBextractor *orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    // ORB_SLAM2::System SLAM(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR);



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


    std::string model_path = data["modelPath"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path);
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);
    const auto mvm = pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE);
    s_cam->SetModelViewMatrix(mvm);
    s_cam->SetProjectionMatrix(proj);
    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
    // for (auto &buffer: geomToRender.buffers) {
    //     buffer.second.attributes.erase("normal");
    // }
    // Render tree for holding object position
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        default_prog.Link();
    };
    LoadProgram();

    //press 4 for starting make a roll
    pangolin::RegisterKeyPressCallback('3', [&]() { track = !track; });
    pangolin::RegisterKeyPressCallback('4', [&]() { finishSlamming = true; });
    pangolin::RegisterKeyPressCallback('5', [&]() { startRoll = !startRoll; });
    pangolin::RegisterKeyPressCallback('6', [&]() {
        int x;
        cout << "Please choose a room to start mapping: \n" << "1. Lab \n" << "2. Meeting room \n" << "3. Working room \n" << endl;
        cin >> x;
        cout << "choosen x: " << x << endl;
        if (x == 1 || x == 2 || x == 3){
            switch(x){
                case 1: {alignCameraWithFloor(s_cam, 4, -0.17, -8);} break;
                case 2: {alignCameraWithFloor(s_cam, 4.58513,  0, 3);} break;
                case 3: {alignCameraWithFloor(s_cam, -4, 0.174696, -9.30959);} break;
                default: {"you entered a wrong value, please choose a number between 1-3";} break;
            }
        }else {
            cout << "wrong numbers \n";
        }

    });
    pangolin::RegisterKeyPressCallback('7', [&]() { applyYawRotationToModelCam(s_cam, 180); });
    pangolin::RegisterKeyPressCallback('8', [&]() { printCurrentLocation(s_cam); });


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

    std::vector<cv::Point3d> seenPoints{};

    double timestamp = 0.0; // Initialize timestamp
    char userChoice;
    std::cout << "Starting Loop" << std::endl;

    auto point_arr_length = 0;
    
    bool down_up_flag = true;    
    int i = 0;
    bool tmp_flag = 0;
    double aggregator_step = 0;
    double max_dist_temp = 0;
    int index = 0;
    bool finishedLoop = false;
    vector<double> dists(360/0.5);

    pangolin::OpenGlMatrix max_s_cam;
    double max_ratio = 0;

    float rotation_angle = 0.0f;
    const float rotation_speed = 1.0f;

    bool temp_boolean = false;

    std::vector<cv::Mat> listOfDescriptors;
    std::vector< std::vector<cv::KeyPoint> > listOfKeyPoints;
    do {
        *ready = true;
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                    << std::endl;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        // pangolin::glDrawAxis(1.0);

        // Load any pending geometry to the GPU.
        if (d_cam.IsShown()) {
            

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            
            //adding 3d-axis of the drone perspective and the object perspective.
            //pangolin::glDrawAxis(5.0);

            //pangolin::glSetFrameOfReference(s_cam->GetProjectionMatrix());
            //pangolin::glDrawAxis(5.0);
            //pangolin::glUnsetFrameOfReference();

            // Apply rotation transformation to the cube
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glRotatef(rotation_angle, 0.0f, 1.0f, 0.0f); // Rotate around Y-axis

            default_prog.Bind();
            default_prog.SetUniform("KT_cw",  s_cam->GetProjectionMatrix() *  s_cam->GetModelViewMatrix());
            pangolin::GlDraw(default_prog, geomToRender, nullptr);
            default_prog.Unbind();

            glPopMatrix();

            // Increment the rotation angle
            rotation_angle += rotation_speed;

            if (rotation_angle >= 360.0f) {
                rotation_angle -= 360.0f;
            }

            // default_prog.Bind();
            // default_prog.SetUniform("KT_cw",  s_cam->GetProjectionMatrix() *  s_cam->GetModelViewMatrix());
            // pangolin::GlDraw(default_prog, geomToRender, nullptr);
            // default_prog.Unbind();


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

            //for_init == false
            // int row, col;
            // row = img.rows;
            // col = img.cols;
            // cout << "cols: " << col << endl;
            // //temp_boolean == true
            // if (temp_boolean == true){
            //     img = img(cv::Range(0, row-1), cv::Range(int(col/2) - 30, int(col/2) + 30));
            //     // imgBuffer = imgBuffer(cv::Range(0, row-1), cv::Range(int(col/2) - 30, int(col/2) + 30));
            // }

            writer.write(imgBuffer);

            s_cam->Apply();

            glDisable(GL_CULL_FACE);

            d_cam.Activate();

        }
        // std::cout << "Trying to SLAM!" << std::endl;
        // SLAM.TrackMonocular(img, timestamp);
        
        if (startRoll){
            // if (tmp_flag == 0){
            //     // alignCameraWithFloor(s_cam, 4.7, 0.5, 2.521);
            //     tmp_flag = 1;
            // }
            // std::vector<cv::KeyPoint> pts;
            // cv::Mat mDescriptors;
            // orbExtractor->operator()(img, cv::Mat(), pts, mDescriptors);
//            auto slamOutput = SLAM.TrackMonocular(img, timestamp);
//            // auto slamOutput = SLAM.TrackMonocular(mDescriptors, pts, timestamp);
//            auto desc = SLAM.GetTracker()->mCurrentFrame.mDescriptors;
//            auto keys = SLAM.GetTracker()->mCurrentFrame.mvKeys;
//
//            listOfDescriptors.push_back(desc);
//            listOfKeyPoints.push_back(keys);
//
//            std::cout << keys.size() << std::endl;
            // std::cout << desc << std::endl;
            // std::cout << mDescriptors << std::endl;
            auto empty_slam = rotateAndSlam(max_ratio, max_s_cam, dists, img, down_up_flag, timestamp, i, SLAM, s_cam, 0.5, aggregator_step);
             cout << "aggregator: " << aggregator_step << endl;
            if (empty_slam == true){continue;}
        }
         if (finishedLoop == false & SLAM.GetLoopClosing()->isFinished() || aggregator_step == 380){

             SLAM.ActivateLocalizationMode();

             startRoll = false; finishedLoop = true;
             // *stopFlag = true;
            
             for(int j = 0 ; j < dists.size(); j++){
                 if (dists[j] > max_dist_temp){
                     max_dist_temp = dists[j];
                     index = j;
                 }
             }
             cout << "index: " << index << "max ratio on that index: " << max_dist_temp << endl;
            
         }
        if (finishedLoop == true & aggregator_step > 300){
            // s_cam->SetModelViewMatrix(max_s_cam);
            // applyForwardToModelCam(s_cam, 1);

            // string input;
            // cout << "wanna set state to not-init? ";
            // cin >> input;

            // if (input == "y"){
            //     // SLAM.GetTracker()->setORBState();
            //     temp_boolean = true;
            // }

            
            // SLAM.Shutdown();
            // SLAM.TrackMonocular(img, timestamp);
            finishedLoop = false;
            aggregator_step = 0;
            startRoll = true;
            timestamp += 0.1;
            *stopFlag = true;
            pangolin::FinishFrame();
            continue;
        }
        // SLAM.TrackMonocular(img, timestamp);
        timestamp += 0.1;
        pangolin::FinishFrame();
        if (finishSlamming){
            break;
        }
    } while (!pangolin::ShouldQuit() && !*stopFlag);
    writer.release();
    saveMap(0, simulatorOutputDir, &SLAM);

    saveKeyPointsAndDescriptors(listOfDescriptors, listOfKeyPoints, &SLAM);
    SLAM.SaveMap(simulatorOutputDir + "/simulatorCloudPoint.bin");
    SLAM.Shutdown();
}


void saveKeyPointsAndDescriptors(std::vector<cv::Mat> listOfDescriptors, std::vector<std::vector<cv::KeyPoint>> listOfKeyPoints, ORB_SLAM2::System *SLAM){   
    std::ofstream outputFile("keyPoints.csv");
    int i = 0;
    for(const std::vector<cv::KeyPoint> &listOfPoints : listOfKeyPoints){
        
        outputFile << "X,Y" << std::endl;
        outputFile << "Frame" + std::to_string(i) << std::endl;
        for(const cv::KeyPoint &point : listOfPoints){
            //std::cout << point.pt.x << ' ' << point.pt.y << std::endl;
            outputFile << point.pt.x << ", " << point.pt.y << std::endl;   
        }
        i++;
    }
    outputFile.close();
}



//4, -0.17, -8
void alignCameraWithFloor(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, int x, int y, int z){
    auto model_view_at = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    // cout << model_view_at << endl;
    model_view_at(0, 3) = x;
    model_view_at(1, 3) = y;
    model_view_at(2, 3) = z;

    model_view_at(0, 0) = -1;
    model_view_at(0, 1) = 0;
    model_view_at(0, 2) = 0;

    model_view_at(1, 0) = 0;
    model_view_at(1, 1) = 1;
    model_view_at(1, 2) = 0;

    model_view_at(2, 0) = 0;
    model_view_at(2, 1) = 0;
    model_view_at(2, 2) = -1;


    s_cam->SetModelViewMatrix(model_view_at);
};


cv::Mat last_mat;
bool rotateAndSlam(double &max_ratio, pangolin::OpenGlMatrix& max_s_cam, vector<double> &dists, cv::Mat& img, bool &down_up_flag, double& timestamp, int &i, ORB_SLAM2::System &SLAM, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double step, double& aggregator){
    cv::Mat val = SLAM.TrackMonocular(img, timestamp);
    int started_tracking_already = SLAM.GetTracker()->mState;
//
//    vector<ORB_SLAM2::MapPoint*> vpCurrentMPs = SLAM.GetMap()->GetCurrentMapPoints();
//
//    set<ORB_SLAM2::MapPoint*> currentFrameKP = getCurrentFrameKeyPoints(SLAM);

//    pangolin::OpenGlMatrix M;
//    SLAM.GetMapDrawer()->GetCurrentOpenGLCameraMatrix(M);


    // double x, y, z;
    // auto model_view_at = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    // x = model_view_at(0, 3);
    // y = model_view_at(1, 3);
    // z = model_view_at(2, 3);

    // cout << x << ' ' << y << " " << z << " " << endl;


//    vector<double> avg_and_max_dist = getAvgDistance3D(vpCurrentMPs, M);
//
//    double current_ratio = avg_and_max_dist[0] / avg_and_max_dist[1];
//
//    if (current_ratio > max_ratio){
//        max_ratio = current_ratio;
//        max_s_cam = s_cam->GetModelViewMatrix();
//
//        cout << "max ratio: " << max_ratio << "max dist: " << avg_and_max_dist[0] << "avg dist: " << avg_and_max_dist[1] << endl;
//    }

    //checking section
    // cout << "max distance in the current frame from the drone is: " << avg_and_max_dist[0] << " and the avg dist is: " << avg_and_max_dist[1] << endl;

     if (last_mat.empty() != true){
         if (0){
             applyYawRotationToModelCam(s_cam, 3*step);
             if (started_tracking_already == 2) {
                 aggregator = aggregator - 3*step;
             }
         }
     }
    if (1){
        if (down_up_flag == true){
            applyUpModelCam(s_cam, -0.05);
            if(i == 10){
                i = 0;
                down_up_flag = false;
            }
            // applyRightToModelCam(s_cam, -0.1);
        }else {
            applyUpModelCam(s_cam, 0.05);
            if(i == 10){
                i = 0;
                down_up_flag = true;
            }
            // applyRightToModelCam(s_cam, 0.1);
        }
    }
    //int(timestamp) % 10 != 0
    if (1){
        if (started_tracking_already == 2) {
            applyYawRotationToModelCam(s_cam, -step);
            aggregator = aggregator + step;
        }
    }
    if (val.empty() == 1 || started_tracking_already == 3){
        //started_tracking_already == 2
        if (1) {
            applyYawRotationToModelCam(s_cam, 1*step);
            aggregator = aggregator - 1*step;
        }
        // applyRightToModelCam(s_cam, -0.01);
        // timestamp += 0.1;
        // pangolin::FinishFrame();
        i++;
        last_mat = val;
        return true;
    }
    i++;
    last_mat = val;
    return false;
}


void printCurrentLocation(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam){
    double x, y, z;
    auto model_view_at = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    x = model_view_at(0, 3);
    y = model_view_at(1, 3);
    z = model_view_at(2, 3);

    cout << x << ' ' << y << " " << z << " " << endl;

}


//vector<double> getAvgDistance3D(vector<ORB_SLAM2::MapPoint*>& currentFrameKP, pangolin::OpenGlMatrix& M){
//    vector<double> ret_values(2);
//
//    double x, y, z, max_dist, avg_dist, count_3d_p, t_x, t_y, t_z;
//    vector<ORB_SLAM2::MapPoint*>::iterator begin, end;
//
//    max_dist = 0;
//    avg_dist = 0;
//    count_3d_p = 0;
//    begin = currentFrameKP.begin();
//    end = currentFrameKP.end();
//
//    x = M(0,3);
//    y = M(1,3);
//    z = M(2,3);
//
//    // cout << M << endl;
//
//    for(vector<ORB_SLAM2::MapPoint*>::iterator t_begin = begin ; t_begin != end ; t_begin++ ){
//        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d((*t_begin)->GetWorldPos());
//        // cout << v.x() << " tx " << v.y() << " ty " << v.z() << " tz " << x << " x " << y << " y " << z << " z " << endl;
//
//        //((v.x() <= x + 0.5 && v.x() >= x - 0.5) && (v.y() <= y + 0.5 && v.y() >= y - 0.5) && v.z() >= z
//        if(v.z() <= z + 0.5 && v.z() >= z - 0.5){
//            double distance = hypot(hypot(x-v.x(),y-v.y()),z-v.z());
//
//            avg_dist += distance;
//            count_3d_p++;
//
//            if (distance > max_dist){
//                max_dist = distance;
//            }
//        }
//
//    }
//
//    // cout << max_dist << "-  avg: " << avg_dist/currentFrameKP.size() << endl;
//    ret_values[0] = max_dist;
//    if (count_3d_p != 0){
//        ret_values[1] = avg_dist/count_3d_p;
//    } else {ret_values[1] = 0;}
//
//
//    return ret_values;
//}


// double getAvgDistanceInAngle2D(vector<cv::KeyPoint> &currentFrameKP, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam){
//     auto model_view_at = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
//     double x, y, max_dist;

//     x = model_view_at(0, 3);
//     y = model_view_at(2, 3);

//     // cv::Size canvasSize = cv::Size(640, 480);
//     // visualizeKeypoints(currentFrameKP, canvasSize);
    
//     max_dist = 0;
//     for(int t = 0; t < currentFrameKP.size(); t++){
//         double point_x = currentFrameKP[t].pt.x;
//         double point_y = currentFrameKP[t].pt.y;
//         double curr_dist = std::hypot(x-point_x, y-point_y);
//         if (curr_dist > max_dist){
//             max_dist = curr_dist;
//         }
//     }

//     return max_dist;
// }
//
//Idea: check for the avg distance and then the maximum dist, if the maximum is alot bigger than the avg then thats the exit angle.
//

// void visualizeKeypoints(const std::vector<cv::KeyPoint>& keypoints, const cv::Size& canvasSize) {
//     cv::Mat canvas(canvasSize, CV_8UC3, cv::Scalar(255, 255, 255)); // Create a white canvas

//     // Draw keypoints on the canvas
//     for (const cv::KeyPoint& kp : keypoints) {
//         cv::circle(canvas, kp.pt, 5, cv::Scalar(0, 0, 255), -1); // Red circles for keypoints
//     }

//     cv::imshow("Keypoints Visualization", canvas);
//     cv::waitKey(10);
//     cv::destroyAllWindows();
// }


//set<ORB_SLAM2::MapPoint*> getCurrentFrameKeyPoints(ORB_SLAM2::System &SLAM){
//    set<ORB_SLAM2::MapPoint*> temp;
//    if (SLAM.GetTracker()->mState == 2){
//        temp = SLAM.GetMap()->mspMapPoints;
//        return temp;
//    }
//    return temp;
//}


int main(int argc, char **argv) {

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    bool stopFlag = false;
    bool ready = false;
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam = std::make_shared<pangolin::OpenGlRenderState>();
    std::thread t(makeARound, std::ref(settingPath), &stopFlag, std::ref(s_cam), &ready);
//    int startSleepTime = 3;
//    std::cout << "wating " << startSleepTime << " to init commands " << std::endl;
    while (!ready) {
        usleep(500);
    }

    t.join();

    return 0;
}
