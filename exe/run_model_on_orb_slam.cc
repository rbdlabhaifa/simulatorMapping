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

//saves the map (map points, key points&descriptors)
void saveMap(int mapNumber, std::string &simulatorOutputDir, ORB_SLAM2::System *SLAM) {
    std::ofstream pointData; // the data of the points will be saved to this object

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
                }
            }
            pointData << std::endl;
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;

}

//run model on orbslam
void runModelAndOrbSlam(std::string &settingPath, bool *stopFlag, std::shared_ptr<pangolin::OpenGlRenderState> &s_cam,
                        bool *ready) {
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data; //put the general settings path in json variable named data
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"]; //the settings of the camera
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ); //FSettings contains these details

    //details about the camera
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //the position and resolution of the camera
    Eigen::Matrix3d K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    Eigen::Vector2i viewport_desired_size(640, 480);

    cv::Mat img;

    //ORB Parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    //extracts the keypoints from the frame (Orb_Slam functionality)
    ORB_SLAM2::ORBextractor *orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST,
                                                                        fMinThFAST);

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;


    //save the current time in order to save it in directory name
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));  //convert time to string and save in time_buf
    std::string currentTime(time_buf);


    std::string vocPath = data["VocabularyPath"]; 
    std::string droneYamlPathSlam = data["DroneYamlPathSlam"]; //the settings of the camera
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"]; //The surface that the camera needs to be parallel to it
    std::string videoPath = data["offlineVideoTestPath"]; //path to the video 
    bool loadMap = data["loadMap"]; //whether load a map or start from the beginning
    double movementFactor = data["movementFactor"]; //movementFactor. larger movementFactor -> move faster
    bool isSavingMap = data["saveMap"]; //if we need to save the map
    std::string loadMapPath = data["loadMapPath"]; //the path to the map (if we want to load it)
    std::string simulatorOutputDirPath = data["simulatorOutputDir"]; //path of the directory to put the output in
    std::string simulatorOutputDir = simulatorOutputDirPath + currentTime + "/"; //providing a specific path to the directory with the currnt time
    std::filesystem::create_directory(simulatorOutputDir); //create a new directory where we will put the output of the simulator
                                                           //the path includes the current time
    ORB_SLAM2::System SLAM = ORB_SLAM2::System(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, false, loadMap,
                                               loadMapPath,
                                               true); //instance of System variable, allows access to ORBSLAM functionality

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

    //opens the video
    cv::VideoWriter writer;
    writer.open(simulatorOutputDir + "/scan.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(viewport_desired_size[0], viewport_desired_size[1]), true);

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
           (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);  //extracts the key points and descriptors
            SLAM.TrackMonocular(descriptors, keypoints, timestamp); //finds the map points from descriptors, keypoints and timestamp

            s_cam->Apply();

            glDisable(GL_CULL_FACE);
        }

        pangolin::FinishFrame();
    }
    writer.release();
    saveMap(0, simulatorOutputDir, &SLAM);
    SLAM.SaveMap(simulatorOutputDir + "/simulatorCloudPoint.bin"); //save the map
    SLAM.Shutdown(); //turn off the SLAM variable
}

//move the camera up by value
void applyUpModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(1, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

//move the camera forward by value
void applyForwardToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(2, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

//move the camera right by value
void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(0, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}

//perform yaw rotate by value (y axis)
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

    s_cam->SetModelViewMatrix(newModelView); //after the rotation, the camera view is different
}

//perform pitch rotate by value (x axis)
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

    s_cam->SetModelViewMatrix(newModelView); //after the rotation, the camera view is different
}


int main(int argc, char **argv) {

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    bool stopFlag = false;
    bool ready = false;
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam = std::make_shared<pangolin::OpenGlRenderState>();
    std::thread t(runModelAndOrbSlam, std::ref(settingPath), &stopFlag, std::ref(s_cam), &ready); //create a thread that will perform the runModelAndOrbSlam
    while (!ready) {
        usleep(500);
    }
    t.join(); //wait for the thread
    return 0;
}