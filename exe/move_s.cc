#include <thread>
#include <future>
#include <queue>
#include <iostream>
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
void StraightNavigation(pangolin::OpenGlRenderState& cam, Eigen::Vector3f target, double navigation_speed, pangolin::GlSlProgram& default_prog, const pangolin::GlGeometry& geomToRender)
{
    Eigen::Vector3f current_position(
        cam.GetModelViewMatrix()(0, 3),
        cam.GetModelViewMatrix()(1, 3),
        cam.GetModelViewMatrix()(2, 3)
    );

    Eigen::Vector3f direction = (target - current_position).normalized();

    float distance_to_target = (target - current_position).norm();

    while (distance_to_target > navigation_speed) {
        // Update the camera position
        default_prog.Bind();
        default_prog.SetUniform("KT_cw", cam.GetProjectionMatrix() * cam.GetModelViewMatrix());
        pangolin::GlDraw(default_prog, geomToRender, nullptr);
        default_prog.Unbind();

        current_position += direction * navigation_speed;
        cam.SetModelViewMatrix(
            Eigen::Affine3f(Eigen::Translation3f(current_position))
        );

        // Update the distance to the target and render the scene
        distance_to_target = (target - current_position).norm();

        //cam.Apply(); 
        pangolin::FinishFrame();
    }

    // Calculate the final translation step to precisely reach the target
    Eigen::Vector3f final_translation = direction * distance_to_target;
    current_position += final_translation;
    cam.SetModelViewMatrix(
        Eigen::Affine3f(Eigen::Translation3f(current_position))
    );

    // Render the final scene
    pangolin::FinishFrame();
}


void TargetPointStraight(pangolin::OpenGlRenderState& cam, Eigen::Vector3f target, double navigation_speed, pangolin::GlSlProgram& default_prog, const pangolin::GlGeometry& geomToRender)
{
    Eigen::Vector3f current_position(
        cam.GetModelViewMatrix()(0, 3),
        cam.GetModelViewMatrix()(1, 3),
        cam.GetModelViewMatrix()(2, 3)
    );

    Eigen::Vector3f target_direction = (target - current_position).normalized();
    Eigen::Vector3f up(0.0f, 0.0f, 1.0f);  // Assuming up direction is along z-axis
    Eigen::Vector3f right = target_direction.cross(up).normalized();
    Eigen::Vector3f new_up = right.cross(target_direction).normalized();

    Eigen::Matrix4f new_view_matrix;
    new_view_matrix << right.x(), right.y(), right.z(), 0,
        new_up.x(), new_up.y(), new_up.z(), 0,
        -target_direction.x(), -target_direction.y(), -target_direction.z(), 0,
        0, 0, 0, 1;

    // Update the camera view matrix to point towards the target direction
    cam.SetModelViewMatrix(new_view_matrix);

    // Call the function to navigate towards the target
    StraightNavigation(cam, target, navigation_speed, default_prog, geomToRender);
}


int main()
{

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath); // inpute file for reading the data from JSON file 
    nlohmann::json data;
    programData >> data;
    programData.close();   // Closing file read 


    double navigation_speed = 0.1;
    std::string configPath = data["DroneYamlPathSlam"];//retrieves the path to another configuration file .
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);//opens the YAML configuration file for reading.

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];  ///put the values from the DroneYamlPathSlam by using the read file
    float cy = fSettings["Camera.cy"];
    float viewpointX = fSettings["RunModel.ViewpointX"];
    float viewpointY = fSettings["RunModel.ViewpointY"];
    float viewpointZ = fSettings["RunModel.ViewpointZ"];

    Eigen::Matrix3d K; // declares an Eigen matrix (K) to store the camera intrinsic matrix.

    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;  //t initializes the intrinsic matrix K using the extracted fx, fy, cx, and cy values.
    cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0); //This line creates a corresponding OpenCV matrix (K_cv) using the same camera intrinsic parameters.



    bool go_toTarget = false;


    Eigen::Vector3f wanted_point(5.0, 2.0, -1.0);

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);



    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState cam(
        pangolin::ProjectionMatrix(640, 480, K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
        pangolin::ModelViewLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0) // Updated camera parameters
    );




    pangolin::Handler3D handler(cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)640 / (float)480))
        .SetHandler(&handler);

    std::string model_path = data["modelPath"];  // load the model path from the data  
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path); // creat object that represent 3D geometry data by using LoadGeometry which load 3D geometry model from the file path 
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);     // load the computed axis-aligned bounding box of the 3D geometry model to aabb
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);


    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
    // Render tree for holding object position
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        default_prog.Link();
    };
    LoadProgram();

    pangolin::RegisterKeyPressCallback('p', [&]() {go_toTarget = !go_toTarget; });


    Eigen::Vector3d Pick_w = handler.Selected_P_w();//: This line initializes the Eigen::Vector3d variable Pick_w with the current 3D position that is selected using the mouse in the visualization window.
    std::vector<Eigen::Vector3d> Picks_w;// creates an empty vector named Picks_w to store selected 3D positions.

    while (!pangolin::ShouldQuit()) {

        // Clear the screen and activate the camera view
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                << std::endl;
        }


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (d_cam.IsShown()) {
            d_cam.Activate();


            default_prog.Bind();
            default_prog.SetUniform("KT_cw", cam.GetProjectionMatrix() * cam.GetModelViewMatrix());
            pangolin::GlDraw(default_prog, geomToRender, nullptr);
            default_prog.Unbind();




            if (go_toTarget) {

                TargetPointStraight(cam, wanted_point, navigation_speed, default_prog, geomToRender);
                go_toTarget = false; // Reset the flag    
            }


            cam.Apply(); //applies transformations to the camera

            glDisable(GL_CULL_FACE);
        }
        // Update the Pangolin display
        pangolin::FinishFrame();
        Eigen::Vector3d camera_position(cam.GetModelViewMatrix()(0, 3),
            cam.GetModelViewMatrix()(1, 3),
            cam.GetModelViewMatrix()(2, 3));
        std::cout << "Camera Position: (" << camera_position[0] << ", " << camera_position[1] << ", " << camera_position[2] << ")\n";
    }

}
