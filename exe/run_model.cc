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

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (auto point: seen_points) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    glEnd();

    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (auto point: new_points_seen) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    std::cout << new_points_seen.size() << std::endl;

    glEnd();
}

cv::Point3f convert2Dto3D(cv::Point2f keypoint, const cv::Mat& K, const cv::Mat& depth, float near_plane, float far_plane) {
    cv::Point3f point3D;

    float depth_normalized = depth.at<float>(static_cast<int>(keypoint.y), static_cast<int>(keypoint.x));
    if (depth_normalized > 0) {
        float z = 2.0 * near_plane * far_plane / (far_plane + near_plane - (far_plane - near_plane) * (2.0 * depth_normalized - 1.0));
        float x = ((keypoint.x - K.at<float>(0, 2)) * z / K.at<float>(0, 0));
        float y = ((keypoint.y - K.at<float>(1, 2)) * z / K.at<float>(1, 1));
        point3D = cv::Point3d(x, y, z);
    }

    return point3D;
}

int main(int argc, char **argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
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

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewport_desired_size[0], viewport_desired_size[1]);
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
            cv::cvtColor(imgBuffer, img,  cv::COLOR_RGBA2BGR);
            cv::flip(img, img, 0);

            cv::imshow("image", img);
            cv::waitKey(2); // You can replace 2 with 0 if you want the window to wait indefinitely for a key press

            // Create an ORB detector
            cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

            // Detect keypoints
            std::vector<cv::KeyPoint> keypoints;
            detector->detect(img, keypoints);

            // Draw keypoints on the image
            cv::Mat image_keypoints;
            cv::drawKeypoints(img, keypoints, image_keypoints);

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
            cv::flip(depth, depth, 0);

            // Normalize depth values to range 0-1
            cv::normalize(depth, depth, 0.0, 1.0, cv::NORM_MINMAX);

            // Convert depth to CV_8UC1 for better visualization
            cv::Mat depth_visualization;
            depth.convertTo(depth_visualization, CV_8UC1, 255.0);

            cv::imshow("Depth", depth_visualization);
            cv::waitKey(2); // You can replace 2 with 0 if you want the window to wait indefinitely for a key press

            // Convert keypoints pixels to keypoints 3d points
            std::vector<cv::Point3d> keypoint_points;
            for (const auto& keypoint : keypoint_positions)
            {
                cv::Point3f point_float = convert2Dto3D(keypoint, K_cv, depth, NEAR_PLANE, FAR_PLANE);
                cv::Point3d point = cv::Point3d(point_float.x, point_float.y, point_float.z);
                keypoint_points.push_back(point);
            }

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), keypoint_points);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
