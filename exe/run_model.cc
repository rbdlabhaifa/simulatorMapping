/*********** add-comments , task1 ***********/
#include <thread>   //This library provides functionality for creating and managing threads
#include <future>   //This library provides mechanisms to represent and manipulate asynchronous operations in C++, such as obtaining the results from asynchronous computations in other threads.
#include <queue>    //This library provides a standard implementation of a queue data structure

//Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input. It offers functionality for user interfaces, 3D visualization, and managing sensor input.
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>

#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "include/run_model/TextureShader.h"
#include "include/Auxiliary.h"

#include "ORBextractor.h"

//Eigen is a C++ library for linear algebra, matrix and vector operations, numerical solvers and related algorithms.
#include <Eigen/SVD>
#include <Eigen/Geometry>

//offers a wide range of functionalities for processing and analyzing images and videos.
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define NEAR_PLANE 0.1
#define FAR_PLANE 20


//this function is responsible for rendering points in 3D using OpenGL.
//this function takes 2 parameters: vector of 3D points that have seen before and vector of new 3D points that have not seen before
void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen) {
    
    //reading settings/data (from JSON file) and save them 
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    const int point_size = data["pointSize"];   //setting point size 

    //Rendering Previously Seen Points in black   `
    glPointSize(point_size);
    glBegin(GL_POINTS); //rendering points as OpenGL points
    glColor3f(0.0, 0.0, 0.0);   //set the color black for this points

    //iterates through the `seen_points` vector and renders each 3D point  
    for (auto point: seen_points) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));   
    }
    glEnd();    //end rendering

    //Rendering newly Seen Points in red   `
    glPointSize(point_size);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);   //(red, green, blue)

    //iterates through the `new_points_seen` vector and renders each 3D point  
    for (auto point: new_points_seen) {
        glVertex3f((float) (point.x), (float) (point.y), (float) (point.z));
    }
    std::cout << new_points_seen.size() << std::endl;   //print num of new points

    glEnd();    //end rendering
}

//this function convers the point from 2D (x,y) to 3D (X,Y,Z).
cv::Point3f convert2Dto3D(cv::Point2f keypoint, const cv::Mat& K, const cv::Mat& depth, const pangolin::OpenGlRenderState& cam_state) {
    
    /*[0]-> x: The x-coordinate of the lower-left corner of the viewport.
    [1]-> y: The y-coordinate of the lower-left corner of the viewport.
    [2]-> width: The width of the viewport.
    [3]-> height: The height of the viewport.*/
    GLint viewport[4];
    GLdouble modelview[16]; //the camera's position and orientation in the world
    GLdouble projection[16];    //the camera's intrinsic parameters and projection

     /// Get the current viewport settings
    glGetIntegerv(GL_VIEWPORT, viewport);
    
    // Copy the modelview and projection matrices 
    for (int i = 0; i < 16; ++i) {
        modelview[i] = cam_state.GetModelViewMatrix().m[i];
        projection[i] = cam_state.GetProjectionMatrix().m[i];
    }

    GLdouble x, y, z;
    GLdouble worldX, worldY, worldZ;

    x = keypoint.x;
    y = (float)viewport[3] - keypoint.y; // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    //This function maps the 2D point in the image plane to a 3D point in the camera's coordinate system (world coordinates)
    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);

    //return 3D point
    return cv::Point3f((float)worldX, (float)worldY, (float)worldZ);
}
//this function is responsible for saving a list of 3D keypoints. and takes 2 parameters:
//1. A constant reference to a vector of 3D keypoints
//2. A constant reference to a string representing the name of the CSV file to be created
void saveKeypointsToCSV(const std::vector<cv::Point3d>& keypoints, const std::string& filename) {
    //create an 'ofstream' object and open the file 'filename' for writing
    std::ofstream csv_file(filename);   //csv file is a simple file format used to store tabular data

    //write all the keypoints to the CSV file. (each line in the CSV file represent one 3D keypoint)
    for (const auto& keypoint : keypoints) {
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl;
    }

    //close the CSV file
    csv_file.close();
}

int main(int argc, char **argv) {
    //reading settings (from JSON file) and save them 
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);   //Create a FileStorage object 'fSettings' that opens the YAML file at the path 'configPath' in read mode.

    //reading and saving all the camera details and parameters
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float viewpointX = fSettings["RunModel.ViewpointX"];
    float viewpointY = fSettings["RunModel.ViewpointY"];
    float viewpointZ = fSettings["RunModel.ViewpointZ"];

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
    ORB_SLAM2::ORBextractor* orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // Options - we can delete this (task 2)
    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    bool cull_backfaces = false;

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewport_desired_size[0], viewport_desired_size[1]);
    glEnable(GL_DEPTH_TEST);

    // Define Projection (the process of transforming 3D points to 2D points) and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
            pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)
    );

    // Create Interactive View in window (cration of an interactive 3D view within the window generated by the Pangolin library)
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)-viewport_desired_size[0] / (float)viewport_desired_size[1]))
            .SetHandler(&handler);

    // Load Geometry asynchronously (loading 3D geometry data into the program's memory and preparing it for rendering)
    std::string model_path = data["modelPath"]; //Retrieve the path to the 3D model from the JSON data object. The path is associated with the key "modelPath".
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path); // Load the 3D model geometry from the model path.
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load); // Get an axis-aligned bounding box (AABB) that encompasses the geometry of the loaded 3D model. 
    Eigen::AlignedBox3f total_aabb; // Declare an Eigen::AlignedBox3f object, which represents a 3D box that can grow to include additional points
    total_aabb.extend(aabb);    // Extend total_aabb to include the AABB of the loaded 3D model
    const auto mvm = pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE);
    // Set the ModelView and Projection matrices 
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
    
    // Render tree for holding object position
    pangolin::GlSlProgram default_prog; //create an instance of pangolin::GlSlProgram called default_prog
    // Define a lambda function (function without a name) that will load an OpenGL shader program (type of program that is used to run on the GPU)
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();    // Remove all previously attached shader objects.
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);    // Add a shader to the program from a shader source code string.
        default_prog.Link();    // Link all added shaders together.
    };
    LoadProgram();  // Call the lambda function to load the shader program.
    
    // Register key press callbacks that toggle boolean flags when certain keys are pressed.
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });

    // Show axis and axis planes
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });

    Eigen::Vector3d Pick_w = handler.Selected_P_w();
    std::vector<Eigen::Vector3d> Picks_w;

    //this loop runs until the user closes the visualization window
    while (!pangolin::ShouldQuit()) {
        // Check if the selected point in world coordinates has changed significantly. If it has, print its coordinates.
        // 1E-6 is a small threshold to ignore insignificant changes.
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {  
            // If the selected point has changed, update our record of it and add it to the list of selected points
            Pick_w = handler.Selected_P_w();    
            Picks_w.push_back(Pick_w);
            // Print the coordinates of the selected point
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                      << std::endl;
        }

        // Clear the color and depth buffer bits
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

            // Get the current OpenGL viewport size
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
            (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);

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

            saveKeypointsToCSV(keypoint_points, keypoints_csv_path);

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), keypoint_points);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
