#include <thread>
#include <future>
#include <queue>

#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>

#include <pangolin/utils/file_utils.h>

#include <pangolin/geometry/geometry_ply.h>
#include <pangolin/geometry/glgeometry.h>

#include "include/run_model/TextureShader.h"
#include "include/Auxiliary.h"

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define POINT_SIZE 2

#define X_OFFSET -5.7
#define Y_OFFSET -0.8
#define Z_OFFSET -1.04
#define YAW_OFFSET -2.56
#define PITCH_OFFSET 1.12
#define ROLL_OFFSET 0.95
#define SCALE_FACTOR 1

void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen) {
    glPointSize(POINT_SIZE);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (auto point: seen_points) {
        glVertex3f((float) point.x * SCALE_FACTOR + X_OFFSET, (float) point.y * SCALE_FACTOR + Y_OFFSET,
                   (float) point.z * SCALE_FACTOR + Z_OFFSET);
    }
    glEnd();

    glPointSize(POINT_SIZE);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (auto point: new_points_seen) {
        glVertex3f((float) point.x * SCALE_FACTOR + X_OFFSET, (float) point.y * SCALE_FACTOR + Y_OFFSET,
                   (float) point.z * SCALE_FACTOR + Z_OFFSET);

    }

    glEnd();
}

int main(int argc, char **argv) {
    const float w = 640.0f;
    const float h = 480.0f;
    const float f = 300.0f;

    using namespace pangolin;

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Options
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;
    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", w, h);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(w, h, f, f, w / 2.0, h / 2.0, 0.1, 1000),
            pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -w / h)
            .SetHandler(&handler);

    // Load Geometry asynchronously
    std::string model_path = data["model_path"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path);
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);
    const Eigen::Vector3f center = total_aabb.center();
    const Eigen::Vector3f view = center + Eigen::Vector3f(1.2, 0.8, 1.2) *
                                          std::max((total_aabb.max() - center).norm(),
                                                   (center - total_aabb.min()).norm());
    const auto mvm = pangolin::ModelViewLookAt(view[0], view[1], view[2], center[0], center[1], center[2],
                                               pangolin::AxisY);
    const double far = 100.0 * (total_aabb.max() - total_aabb.min()).norm();
    const double near = far / 1e6;
    const auto proj = pangolin::ProjectionMatrix(w, h, f, f, w / 2.0, h / 2.0, near, far);
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
    // Render tree for holding object position
    pangolin::AxisDirection spin_other = pangolin::AxisNone;
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, shader);
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
    cv::Mat Twc;
    default_prog.Bind();

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
            default_prog.SetUniform("KT_cw",  s_cam.GetProjectionMatrix() *  s_cam.GetModelViewMatrix());
            pangolin::GlDraw( default_prog, geomToRender );
            // std::cout << "Projection Matrix: " << s_cam.GetProjectionMatrix() << std::endl;
            // std::cout << "Model View Matrix: " << s_cam.GetModelViewMatrix() << std::endl;
            Eigen::Matrix4d mv_mat = s_cam.GetModelViewMatrix();

            // Compute the camera position by inverting the model-view matrix and extracting the translation component
            Eigen::Matrix4d inv_mv_mat = mv_mat.inverse();
            Eigen::Vector3d cam_pos = inv_mv_mat.block<3, 1>(0, 3);

            // Compute the yaw, pitch, and roll angles from the rotation component of the model-view matrix
            Eigen::Matrix3d rot_mat = mv_mat.block<3, 3>(0, 0);
            double yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
            double pitch = asin(-rot_mat(2, 0));
            double roll = atan2(rot_mat(2, 1), rot_mat(2, 2));

            std::cout << "Camera position: " << cam_pos << ", yaw: " << yaw << ", pitch: " << pitch << ", roll: "
                      << roll << std::endl;

            s_cam.Apply();
            if (show_x0) pangolin::glDraw_x0(10.0, 10);
            if (show_y0) pangolin::glDraw_y0(10.0, 10);
            if (show_z0) pangolin::glDraw_z0(10.0, 10);
            if (show_axis) pangolin::glDrawAxis(10.0);
            if (show_bounds) pangolin::glDrawAlignedBox(total_aabb);

            glDisable(GL_CULL_FACE);

            std::string map_input_dir = data["mapInputDir"];
            const std::string cloud_points = map_input_dir + "cloud1.csv";
            std::vector<cv::Point3d> seen_points = Auxiliary::getPointsFromPos(cloud_points,
                   cv::Point3d(cam_pos[0] * SCALE_FACTOR + X_OFFSET, cam_pos[1] * SCALE_FACTOR + Y_OFFSET,
                               cam_pos[2] * SCALE_FACTOR + Z_OFFSET),
                               yaw + YAW_OFFSET, pitch + PITCH_OFFSET, roll + ROLL_OFFSET, Twc);
            drawPoints(std::vector<cv::Point3d>(), seen_points);
        }

        pangolin::FinishFrame();
    }
    default_prog.Unbind();

    return 0;
}
