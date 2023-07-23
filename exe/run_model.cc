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

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

//draws the map points. the seen points are in black, the new points are in red.
//now, the seen points are unused
void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen) {
    //put the general settings path in json variable named data
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];  // the size of the points


    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (auto point: seen_points) { //draw all the seen points in black
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (auto point: new_points_seen) { //draw all the new points in red
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    std::cout << new_points_seen.size() << std::endl; //print the number of new points

    glEnd();
}

//converts the key point from 2D to 3D regard to camera position
//K is not used
cv::Point3f convert2Dto3D(cv::Point2f keypoint, const cv::Mat& K, const cv::Mat& depth, const pangolin::OpenGlRenderState& cam_state) {
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
    y = (float)viewport[3] - keypoint.y; // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);

    return cv::Point3f((float)worldX, (float)worldY, (float)worldZ);
}

//saving all the keypoints to CSV file (keypoints are 3D points)
void saveKeypointsToCSV(const std::vector<cv::Point3d>& keypoints, const std::string& filename) {
    std::ofstream csv_file(filename);

    for (const auto& keypoint : keypoints) {
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl; //save the coordinates of the points
    }

    csv_file.close();
}

int main(int argc, char **argv) {
    //put the general settings path in json variable named data
    std::string settingPath = Auxiliary::GetGeneralSettingsPath(); 
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"]; //path to camera and ORB settings
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ); //FSettings contains these settings

    //cameta settings
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float viewpointX = fSettings["RunModel.ViewpointX"];
    float viewpointY = fSettings["RunModel.ViewpointY"];
    float viewpointZ = fSettings["RunModel.ViewpointZ"];

    //the position and resolution of the camera
    Eigen::Matrix3d K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    Eigen::Vector2i viewport_desired_size(640, 480); //image resolution

    cv::Mat img;

    //ORB Parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"]; //max number of key points
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    //extracts the keypoints from the frame (Orb_Slam functionality)
    ORB_SLAM2::ORBextractor* orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewport_desired_size[0], viewport_desired_size[1]); //the main window
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)-viewport_desired_size[0] / (float)viewport_desired_size[1]))
            .SetHandler(&handler);

    // Load Geometry asynchronously
    std::string model_path = data["modelPath"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path);
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);
    const auto mvm = pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE);
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
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
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });

    Eigen::Vector3d Pick_w = handler.Selected_P_w();
    std::vector<Eigen::Vector3d> Picks_w;

    while (!pangolin::ShouldQuit()) {
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
            default_prog.SetUniform("KT_cw",  s_cam.GetProjectionMatrix() *  s_cam.GetModelViewMatrix());
            pangolin::GlDraw( default_prog, geomToRender, nullptr);
            default_prog.Unbind();

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size);

            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp/8 );
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

            cv::Mat  imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            cv::cvtColor(imgBuffer, img,  cv::COLOR_RGBA2GRAY);
            img.convertTo(img, CV_8UC1);
            cv::flip(img, img, 0);

            cv::imshow("image", img);
            cv::waitKey(2); // You can replace 2 with 0 if you want the window to wait indefinitely for a key press

            // Detect keypoints
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            (*orbExtractor)(img, cv::Mat(), keypoints, descriptors); //gets an image and save the keypoints&descriptors

            // Draw keypoints on the image
            cv::Mat image_keypoints;
            cv::drawKeypoints(img, keypoints, image_keypoints); //gets img&keypoints and dyes the keypoints

            cv::imshow("image_keypoints", image_keypoints);
            cv::waitKey(2); // You can replace 2 with 0 if you want the window to wait indefinitely for a key press

            // Save the x and y values of the keypoints to a vector
            std::vector<cv::Point2f> keypoint_positions;
            for (const auto& keypoint : keypoints)
            {
                cv::Point2f position = cv::Point2f((float)keypoint.pt.x, (float)(keypoint.pt.y));
                keypoint_positions.push_back(position);
            }

            cv::Mat depth(viewport_size[3], viewport_size[2], CV_32FC1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_DEPTH_COMPONENT, GL_FLOAT, depth.data);
            
            // Convert keypoints pixels to keypoints 3d points
            std::vector<cv::Point3d> keypoint_points;
            for (const auto& keypoint : keypoint_positions)
            {
                cv::Point3f point_float = convert2Dto3D(keypoint, K_cv, depth, s_cam);
                cv::Point3d point = cv::Point3d(point_float.x, point_float.y, point_float.z);
                keypoint_points.push_back(point);
            }

            int frame_to_check = data["frameNumber"];
            std::string keypoints_csv_path = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_orbs.csv";

            saveKeypointsToCSV(keypoint_points, keypoints_csv_path); //save the keypoints we have found

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), keypoint_points);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
