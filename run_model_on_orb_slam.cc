/*********** add-comments , task1 ***********/
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

//these functions apply transformations to the camera view in a 3D scene using the Pangolin library.
void applyForwardToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);
void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);
void applyYawRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);
void applyUpModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);
void applyPitchRotationToModelCam(std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value);

//this function is responsible for rendering points in 3D using OpenGL.
void drawPoints(std::vector<cv::Point3d> &seen_points, std::vector<cv::Point3d> &new_points_seen) {
     //reading settings (from JSON file) and save them 
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];   //setting point size 

    //Rendering Previously Seen Points in black 
    glPointSize(point_size);
    glBegin(GL_POINTS);     //rendering points as OpenGL points
    glColor3f(0.0, 0.0, 0.0);   //set the color black for this points

    //iterates through the `seen_points` vector and renders each 3D point  
    for (auto &point: seen_points) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();    //end rendering

    //Rendering newly Seen Points in red 
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    //iterates through the `new_points_seen` vector and renders each 3D point  
    for (auto &point: new_points_seen) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();    //end rendering
}

//this function convers the point from 2D (x,y) to 3D (X,Y,Z),
cv::Point3f convert2Dto3D(cv::Point2f &keypoint, const cv::Mat &K, const cv::Mat &depth,
                          const pangolin::OpenGlRenderState &cam_state) {

    GLint viewport[4]; //[x,y,width,height]
    GLdouble modelview[16];     //the camera's position and orientation in the world
    GLdouble projection[16];        //the camera's intrinsic parameters and projection

    //extract 2D keypoints Position
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

    //This function maps the 2D point in the image plane to a 3D point in the camera's coordinate system (world coordinates)
    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);

    //return 3D point
    return cv::Point3f((float) worldX, (float) worldY, (float) worldZ);
}

// this function save the entire map
//this function take 3 paramater:
//mapNumber: An integer representing the map number. This is used to generate the filename for the output CSV file.
//simulatorOutputDir: holds the directory path where the output CSV file will be saved.
//SLAM: contains the map and keyframe information.
void saveMap(int mapNumber, std::string &simulatorOutputDir, ORB_SLAM2::System *SLAM) {
    //// Open a CSV file for writing the map points and their observations.
    std::ofstream pointData;
    std::unordered_set<int> seen_frames;

    //Generate the filename for the output CSV file based on the map number.
    pointData.open(simulatorOutputDir + "cloud" + std::to_string(mapNumber) + ".csv");
    //Iterate over all the map points in the ORB-SLAM2 system's map
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        //check if the current point is good or bad (we want only good points)
        if (p != nullptr && !p->isBad()) {
            //if it's not null, and a good point so:

            //extract the 3D world position of the map point
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth();   //update the normal and depth info for the point
            cv::Mat Pn = p->GetNormal();
            Pn.convertTo(Pn, CV_64F);
            
            // Write the map point information (world position, and distance invariance) to the CSV file.
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","
                      << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
             
            //take the keyframe and the observatiom of the current map point
            std::map<ORB_SLAM2::KeyFrame *, size_t> observations = p->GetObservations();
             //go through all the observations and save the keypoint data and desciptor
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

//this function is responsible for saving a list of 3D keypoints
void saveKeypointsToCSV(const std::vector<cv::Point3d> &keypoints, const std::string &filename) {
    //create an 'ofstream' object and open the file 'filename' for writing
    std::ofstream csv_file(filename);

    //write all the keypoints to the CSV file. (each line in the CSV file represent one 3D keypoint)
    for (const auto &keypoint: keypoints) {
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl;
    }

    //close the CSV file
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

    //reading and saving all the camera details and parameters
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    Eigen::Matrix3d K;  //used for camera projection and transforming 3D to 2D 
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);  //related to camera calibration and pose estimation.
    Eigen::Vector2i viewport_desired_size(640, 480);    //representing the desired size (width and height) of the visualization window

    cv::Mat img;

    //parameters used to initialize the ORB feature extractor.
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    //create an instance of the 'ORB_SLAM2'
    ORB_SLAM2::ORBextractor *orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST,
                                                                        fMinThFAST);

    // Options - we can delete this (task 2)
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
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string videoPath = data["offlineVideoTestPath"];
    bool loadMap = data["loadMap"];
    double movementFactor = data["movementFactor"];
    bool isSavingMap = data["saveMap"];
    std::string loadMapPath = data["loadMapPath"];
    std::string simulatorOutputDirPath = data["simulatorOutputDir"];
    std::string simulatorOutputDir = simulatorOutputDirPath + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
    ORB_SLAM2::System SLAM = ORB_SLAM2::System(vocPath, droneYamlPathSlam, ORB_SLAM2::System::MONOCULAR, true, false, loadMap,
                                               loadMapPath,
                                               true);

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
           (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);
            SLAM.TrackMonocular(descriptors, keypoints, timestamp);

            s_cam->Apply();

            glDisable(GL_CULL_FACE);
        }

        pangolin::FinishFrame();
    }
    writer.release();
    saveMap(0, simulatorOutputDir, &SLAM);
    SLAM.SaveMap(simulatorOutputDir + "/simulatorCloudPoint.bin");
    SLAM.Shutdown();
}

//this function is responsible for applying an "upward" translation to the camera's position in the 3D scen
void applyUpModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(1, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}
//this function is responsible for applying an "Forward" translation to the camera's position in the 3D scen
void applyForwardToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(2, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}
//this function is responsible for applying an "Right" translation to the camera's position in the 3D scen
void applyRightToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(s_cam->GetModelViewMatrix());
    camMatrix(0, 3) += value;
    s_cam->SetModelViewMatrix(camMatrix);
}
//this  function is responsible for applying a yaw rotation to the camera's orientation in the 3D scene
void applyYawRotationToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
     
     // Convert the rotation angle from degrees to radians
    double rand = double(value) * (M_PI / 180);

    // Calculate the cosine and sine of the rotation angle
    double c = std::cos(rand);
    double s = std::sin(rand);

    // Create a 3x3 rotation matrix around the Y-axis
    Eigen::Matrix3d R;
    R << c, 0, s,
            0, 1, 0,
            -s, 0, c;

    //create a 4x4 identity matrix with the top-left 3x3 block as the rotation matrix
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    //get the current model-view matrix of the camera as an Eigen matrix
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

    // Set the updated model-view matrix back to the camera's OpenGL render state
    s_cam->SetModelViewMatrix(newModelView);
}
//this  function is responsible for applying a Pitch rotation to the camera's orientation in the 3D scene
void applyPitchRotationToModelCam(shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value) {
    // Calculate the cosine and sine of the rotation angle
    double rand = double(value) * (M_PI / 180);
   
    // Calculate the cosine and sine of the rotation angle
    double c = std::cos(rand);
    double s = std::sin(rand);
    
    // Create a 3x3 rotation matrix around the Y-axis
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, c, -s,
            0, s, c;

    //create a 4x4 identity matrix with the top-left 3x3 block as the rotation matrix
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();;
    pangolinR.block<3, 3>(0, 0) = R;

  //get the current model-view matrix of the camera as an Eigen matrix
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

    // Set the updated model-view matrix back to the camera's OpenGL render state
    s_cam->SetModelViewMatrix(newModelView);
}

//this function used to apply a transformation to the camera's OpenGL render state, over a specified time duration with a specific frame rate
void intervalOverCommand(const std::function<void(std::shared_ptr<pangolin::OpenGlRenderState> &, double &)> &func,
                         std::shared_ptr<pangolin::OpenGlRenderState> &s_cam, double value, int intervalUsleep,
                         double fps,
                         int totalCommandTimeInSeconds) {
    
    //calculate the value to increment at each interval based on the totalCommandTimeInSeconds and fps
    double intervalValue = value / fps * totalCommandTimeInSeconds;
    // Initialize the interval index
    int intervalIndex = 0;

    //loop for the specified duration and apply the transformation function at each interval
    while (intervalIndex <= fps * totalCommandTimeInSeconds) {
        usleep(intervalUsleep); // Wait for the specified interval
        func(s_cam, intervalValue); // Apply the transformation function with the calculated value
        intervalIndex += 1; // Increment the interval index for the next iteration
    }
}

//this function takes a command and applies the corresponding transformation to the camera in the 3D scene rendered using Pangolin
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
     //reading settings (from JSON file) and save them 
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    bool stopFlag = false;
    bool ready = false;
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam = std::make_shared<pangolin::OpenGlRenderState>();
    std::thread t(runModelAndOrbSlam, std::ref(settingPath), &stopFlag, std::ref(s_cam), &ready);

//    int startSleepTime = 3;
//    std::cout << "wating " << startSleepTime << " to init commands " << std::endl;
    
    while (!ready) {
        usleep(500);    //suspend 500 microseconds 
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
    
    t.join();   //wait for threads to finish their work

    //    if (isSavingMap) {
    //        SLAM->SaveMap(simulatorOutputDir + "simulatorMap.bin");
    //    }
    //
    //    SLAM->Shutdown();

    return 0;
}
