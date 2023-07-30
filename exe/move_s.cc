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




void top_rot_navigation_f(pangolin::OpenGlRenderState& cam, double value) { //up
    double rand = -double(value) * (M_PI / 180); // Negate the angle to apply negative pitch rotation
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, c, -s,
        0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);
}

void right_rot_navigation_f(pangolin::OpenGlRenderState& cam, double value) { //right
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);
}


void  left_navigation_f(pangolin::OpenGlRenderState& cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(0, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}



void  straight_navigation_f(pangolin::OpenGlRenderState& cam, double value) {// 
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}





int main()
{
    std::cout << "hello\n";
    std::cout << "Key board dictionary:\n";
    std::cout << "go to the RIGHT click   ;\n";
    std::cout << "go to the LEFT click    k\n";
    std::cout << "go  UP click            o\n";
    std::cout << "go  DOWN  click         l\n";
    std::cout << "UP_ROTATION click       w\n";
    std::cout << "DOWN_ROTATION click     s\n";
    std::cout << "RIGHT ROTATION click    d\n";
    std::cout << "LEFT ROTATION click     a\n\n";
    std::cout << "MAKE SURE TO USE JUST SMALL LETTERS press S to start ";
    char x;
    while ((x != 's') && (x != 'S')) {
        std::cin >> x;
    }





    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath); // inpute file for reading the data from JSON file 
    nlohmann::json data;
    programData >> data;
    programData.close();   // Closing file read 


    double navigation_speed = 0.4;
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


    bool straight_navigation = false;
    bool left_navigation = false;
    bool right_navigation = false;
    bool back_navigation = false;
    bool top_rot_navigation = false;
    bool down_rot_navigation = false;
    bool right_rot_navigation = false;
    bool left_rot_navigation = false;
    bool go_toTarget = false;

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);



    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState cam(
        pangolin::ProjectionMatrix(640, 480, K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
        pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)
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
    pangolin::RegisterKeyPressCallback('o', [&]() {straight_navigation = !straight_navigation; });
    pangolin::RegisterKeyPressCallback(';', [&]() {right_navigation = !right_navigation; });
    pangolin::RegisterKeyPressCallback('k', [&]() {left_navigation = !left_navigation; });
    pangolin::RegisterKeyPressCallback('l', [&]() {back_navigation = !back_navigation; });
    pangolin::RegisterKeyPressCallback('w', [&]() {top_rot_navigation = !top_rot_navigation; });
    pangolin::RegisterKeyPressCallback('d', [&]() {right_rot_navigation = !right_rot_navigation; });
    pangolin::RegisterKeyPressCallback('a', [&]() {left_rot_navigation = !left_rot_navigation; });
    pangolin::RegisterKeyPressCallback('s', [&]() {down_rot_navigation = !down_rot_navigation; });
    pangolin::RegisterKeyPressCallback('t', [&]() {go_toTarget = !go_toTarget; });


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




            if (straight_navigation) {

                straight_navigation_f(cam, navigation_speed * 3);
                straight_navigation = false;
            }
            else if (right_navigation) {

                left_navigation_f(cam, -navigation_speed * 3);
                right_navigation = false;
            }
            else if (left_navigation) {

                left_navigation_f(cam, navigation_speed * 3);
                left_navigation = false;
            }
            else if (back_navigation) {

                straight_navigation_f(cam, -navigation_speed * 3);
                back_navigation = false;
            }



            else if (top_rot_navigation) {

                top_rot_navigation_f(cam, navigation_speed * 3);
                top_rot_navigation = false;
            }
            else if (down_rot_navigation) {

                top_rot_navigation_f(cam, -navigation_speed * 3);
                down_rot_navigation = false;
            }
            else if (left_rot_navigation) {

                right_rot_navigation_f(cam, -navigation_speed * 3);
                left_rot_navigation = false;
            }
            else if (right_rot_navigation) {

                right_rot_navigation_f(cam, navigation_speed * 3);
                right_rot_navigation = false;
            }

            cam.Apply(); //applies transformations to the camera

            glDisable(GL_CULL_FACE);
        }
        // Update the Pangolin display
        pangolin::FinishFrame();
    }
}
