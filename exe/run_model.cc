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

void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath(); /*This line gets the file path of a jason settings file  The settings file contains various parameters related to
                                                                   the visualization, including the size of the points to be rendered.*/
    std::ifstream programData(settingPath); // opens the  settings file for reading.

    nlohmann::json data; // create a json object named data.
    programData >> data; //read the JSON file into the data JSON object.
    programData.close();// Closes the file stream

    const int point_size = data["pointSize"];

    glPointSize(point_size); // Sets the size of the points in OpenGL based on the point_size variable.
    glBegin(GL_POINTS);  // definition of a set of points to be rendered.
    glColor3f(0.0, 0.0, 0.0); //Sets the color for the points to be rendered. In this case, it sets the color to black .

    for (auto point : seen_points) {
        glVertex3f((float)(point.x), (float)(point.y), (float)(point.z));  //This loop iterates over the points in the seen_points vector and renders each point using glVertex3f, which specifies the 3D coordinates of the point to be drawn.
    }
    glEnd(); //Ends the definition of the points

    glPointSize(point_size);  //Sets the point size  to be rendered.
    glBegin(GL_POINTS); //begins the definition of a new set of points to be rendered.
    glColor3f(1.0, 0.0, 0.0);// sets the color for the new set of points to be rendered. In this case,  the color red 

    for (auto point : new_points_seen) {
        glVertex3f((float)(point.x), (float)(point.y), (float)(point.z)); /* This loop iterates over the points in the new_points_seen vector and renders each point using glVertex3f
                                                                             , specifying the 3D coordinates of the point to be drawn.*/
    }
    std::cout << new_points_seen.size() << std::endl;

    glEnd();
}

cv::Point3f convert2Dto3D(cv::Point2f keypoint, const cv::Mat& K, const cv::Mat& depth, const pangolin::OpenGlRenderState& cam_state) {
    GLint viewport[4];//
    GLdouble modelview[16];//These are arrays to put the OpenGL viewport parameters, modelview matrix, and projection matrix.
    GLdouble projection[16];//

    glGetIntegerv(GL_VIEWPORT, viewport);  //This OpenGL function retrieves the current viewport parameters (x, y, width, height) and stores them in the viewport array.
    for (int i = 0; i < 16; ++i) {
        modelview[i] = cam_state.GetModelViewMatrix().m[i]; //This loop copies the elements of the OpenGL camera's modelview matrix and projection matrix into the respective arrays modelview and projection.
        projection[i] = cam_state.GetProjectionMatrix().m[i];
    }

    GLdouble x, y, z;
    GLdouble worldX, worldY, worldZ;

    x = keypoint.x;
    y = (float)viewport[3] - keypoint.y; // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);//This OpenGL function gluUnProject takes the 2D (x, y, z) point in the viewport and the modelview and projection matrices to calculate the corresponding 3D (world) coordinates (worldX, worldY, worldZ) in the camera's coordinate system.

    return cv::Point3f((float)worldX, (float)worldY, (float)worldZ);//The  3D point  are returned as a cv::Point3f object.


}

void saveKeypointsToCSV(const std::vector<cv::Point3d>& keypoints, const std::string& filename) {
    std::ofstream csv_file(filename); //This line creates an output file stream (csv_file) with the provided filename. The std::ofstream class is used to write data to files.

    for (const auto& keypoint : keypoints) { // loop iterates over each 3D point in the keypoints vector.
        csv_file << keypoint.x << "," << keypoint.y << "," << keypoint.z << std::endl;// writes the X, Y, and Z coordinates of the current 3D point (keypoint) to the CSV file.
    }

    csv_file.close();//
}

int main(int argc, char** argv) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath(); //This line gets the file path of a json settings file .
    std::ifstream programData(settingPath); //  open the jason settings file for reading.
    nlohmann::json data;// create a json object named data.
    programData >> data;//  read the JSON file into the data JSON object.
    programData.close();// Closes the file stream


   // Reading Camera Settings :

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
    Eigen::Vector2i viewport_desired_size(640, 480); //This line declares a 2D vector (viewport_desired_size) representing the desired size of the OpenGL rendering window.

    cv::Mat img; //declares an OpenCV matrix (img) to store images captured from the OpenGL buffer.

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    ORB_SLAM2::ORBextractor* orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST); //creates a new instance of the ORB_SLAM2::ORBextractor class , 

    // Options
    bool show_bounds = false; // show bounding boxes around the rendered geometry.
    bool show_axis = false;// show the axis coordinate system (X, Y, Z axes) on the 
    bool show_x0 = false;//  show the planes representing the X=0
    bool show_y0 = false;// show the planes representing the y=0
    bool show_z0 = false;//  show the planes representing the z=0
    bool cull_backfaces = false;// enable backface culling. Backface culling is a technique used in 3D rendering to improve performance by not rendering triangles facing away from the camera.

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
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)-viewport_desired_size[0] / (float)viewport_desired_size[1]))
        .SetHandler(&handler);

    // Load Geometry asynchronously
    std::string model_path = data["modelPath"];
    const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(model_path); // loads the 3D geometry from the file specified 
    auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);   //The code retrieves the axis-aligned bounding box (AABB) of the loaded 3D geometry 

    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);//: new Eigen::AlignedBox3f object named total_aabb is created, and the bounding box obtained in the previous step (aabb) is extended to include it. 

    const auto mvm = pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY); //This line sets up the initial model view matrix (mvm) based on the specified viewpoint coordinates (viewpointX, viewpointY, viewpointZ).
    const auto proj = pangolin::ProjectionMatrix(viewport_desired_size(0), viewport_desired_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE); //this line sets up the initial projection matrix (proj) using the camera's intrinsic parameters (K) and viewport size (viewport_desired_size). 
    s_cam.SetModelViewMatrix(mvm); //  This sets the initial camera position and orientation in the 3D scene.
    s_cam.SetProjectionMatrix(proj);// The initial projection matrix (proj) is assigned to the OpenGL render state (s_cam). This sets the camera's perspective and field of view.
    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);//
    // Render tree for holding object position
    pangolin::GlSlProgram default_prog;

    auto LoadProgram = [&]() { //A lambda function LoadProgram is defined to set up the shader program (default_prog). 
        default_prog.ClearShaders();  //his line clears any existing shaders attached to the shader program default_prog. 
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);//This line adds a new shader to the program default_prog. The shader being added is an annotated shader provided by Pangolin (referred to as 
        default_prog.Link();//his line links the shader program after adding the new shader
    };
    LoadProgram(); //This line calls the LoadProgram lambda function, setting up the shader program for rendering.
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });//This line registers a keypress callback for the key 'b'. When the user presses the 'b' key, the callback function (a lambda function) is triggered. 
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });//This line registers a keypress callback for the key '0'. When the user presses the '0' key, the callback function (a lambda function) is triggered. 

    // Show axis and axis planes
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });

    Eigen::Vector3d Pick_w = handler.Selected_P_w();//: This line initializes the Eigen::Vector3d variable Pick_w with the current 3D position that is selected using the mouse in the visualization window.
    std::vector<Eigen::Vector3d> Picks_w;// creates an empty vector named Picks_w to store selected 3D positions.

    while (!pangolin::ShouldQuit()) { //continues as long as the Pangolin window is open and the user has not requested to quit the application.
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) { //checks if the selected 3D position (handler.Selected_P_w()) has changed since the last frame
            Pick_w = handler.Selected_P_w(); //This line updates the variable Pick_w with the currently selected 3D point in the 3D scene. 
            Picks_w.push_back(Pick_w);//  adds the selected 3D point to the vector Picks_w. 
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2]) //This line prints the selected 3D point's coordinates to the console 
                << std::endl;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// This line clears the color and depth buffers of the OpenGL rendering context.

        // Load any pending geometry to the GPU.
        if (d_cam.IsShown()) {///checks if the display d_cam is shown or not
            d_cam.Activate();///This line activates the camera to render the 3D scene. It sets up the OpenGL state for rendering the scene from the specified camera's viewpoint.

            if (cull_backfaces) { // conditionally enables backface culling. 
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }

            default_prog.Bind();//This line binds the shader program default_prog to the rendering pipeline.
            default_prog.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix()); //This line sets the uniform variable "KT_cw" in the shader program with the result of multiplying the camera's projection matrix and model-view matrix. 
            pangolin::GlDraw(default_prog, geomToRender, nullptr); //This line renders the geometry geomToRender using the shader program default_prog. The rendered image is then stored in the OpenGL buffer.
            default_prog.Unbind(); //

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size); // retrieves the viewport size, which represents the dimensions of the OpenGL rendering window, and stores them in the viewport_size array. 

            pangolin::Image<unsigned char> buffer;  // declares an image buffer named buffer. 
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");//  sets the pixel format for the image buffer to "RGBA32", which means it will store 4 channels (red, green, blue, alpha) of 8-bit unsigned integers per pixel.
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);//Tallocates memory for the image buffer with the dimensions obtained from the viewport size.
            glReadBuffer(GL_BACK);//  This line sets the OpenGL read buffer to GL_BACK, which indicates that the image will be read from the back buffer.
            glPixelStorei(GL_PACK_ALIGNMENT, 1);// sets the pixel storage mode to GL_PACK_ALIGNMENT, which affects the packing of pixel data in memory.
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);// This line reads the pixel data from the OpenGL buffer and stores it in the previously allocated buffer.

                                //This line creates an OpenCV matrix imgBuffer using the data from the buffer
            cv::Mat  imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);

            //This line converts the RGBA image in imgBuffer to a grayscale image and put it in the img matrix. The
            cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY);
            img.convertTo(img, CV_8UC1);// convert the grayscale image to a single-channel 8-bit unsigned integer image
            cv::flip(img, img, 0);//  flip the img matrix vertically.

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

            //retrieves the value of "frameNumber" and put it in frame_to_check
            int frame_to_check = data["frameNumber"];

            // constructs the file path where the keypoints will be saved as a CSV file
            std::string keypoints_csv_path = std::string(data["framesOutput"]) + "frame_" + std::to_string(frame_to_check) + "_orbs.csv";

            saveKeypointsToCSV(keypoint_points, keypoints_csv_path);

            s_cam.Apply(); //applies the camera parameters (model view and projection matrices) stored in the s_cam variable to the OpenGL rendering context

            glDisable(GL_CULL_FACE); //disables face culling for rendering. 

            drawPoints(std::vector<cv::Point3d>(), keypoint_points);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
