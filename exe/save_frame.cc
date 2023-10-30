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

class KeyPoint3D {
public:
    KeyPoint3D(cv::KeyPoint keypoint=cv::KeyPoint(), cv::Mat descriptor=cv::Mat(), cv::Point3d point=cv::Point3d()) {
        this->keypoint = keypoint;
        this->descriptor = descriptor;
        this->point = point;
    };
    cv::KeyPoint keypoint;
    cv::Mat descriptor;
    cv::Point3d point;
};

void drawPoints(std::vector<cv::Point3d> seen_points, std::vector<KeyPoint3D> keypoints3D) {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::vector<cv::Point3d> new_points_seen;
    for(auto& point : keypoints3D) {
        new_points_seen.push_back(cv::Point3d(point.point.x, point.point.y, point.point.z));
    }

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

cv::Point3f convert2Dto3D(KeyPoint3D keypoint3D, const cv::Mat& K, const cv::Mat& depth, const pangolin::OpenGlRenderState& cam_state) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];

    glGetIntegerv(GL_VIEWPORT, viewport);
    for (int i = 0; i < 16; ++i) {
        modelview[i] = cam_state.GetModelViewMatrix().m[i];
        projection[i] = cam_state.GetProjectionMatrix().m[i];
    }

    cv::Point2f keypoint = cv::Point2f(keypoint3D.keypoint.pt.x, keypoint3D.keypoint.pt.y);

    GLdouble x, y, z;
    GLdouble worldX, worldY, worldZ;

    x = keypoint.x;
    y = (float)viewport[3] - keypoint.y; // OpenGL has the origin in the lower-left corner, so we need to flip the y-coordinate
    z = depth.at<float>(static_cast<int>(y), static_cast<int>(x)); // Get depth value at the keypoint position

    gluUnProject(x, y, z, modelview, projection, viewport, &worldX, &worldY, &worldZ);

    return cv::Point3f((float)worldX, (float)worldY, (float)worldZ);
}

void saveKeypoints3DToCSV(const std::vector<KeyPoint3D> keypoints3D, const std::string& filename) {
    std::ofstream csv_file(filename);

    for(int i = 0; i < keypoints3D.size(); i++) {
        cv::KeyPoint keypoint = keypoints3D[i].keypoint;
        cv::Mat descriptor = keypoints3D[i].descriptor;
        cv::Point3d point = keypoints3D[i].point;
        // Save keypoints values
        csv_file << keypoint.pt.x << "," << keypoint.pt.y << "," << keypoint.size << "," << keypoint.angle << "," <<
                              keypoint.response << "," << keypoint.octave << "," << keypoint.class_id << ",";
        // Save descriptor
        for (int k=0; k < descriptor.cols; k++) {
            csv_file << static_cast<int>(descriptor.at<uchar>(0, k)) << ",";
        }
        // Save 3d point
        csv_file << point.x << "," << point.y << "," << point.z << std::endl;
    }

    csv_file.close();
}

int main(int argc, char **argv) {
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./exe/save_frame frame_name" << std::endl;
        return 1;
    }
    std::string frame_name(argv[1]);

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

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    bool orbSlamExtractor = data["orbSlamExtractor"];

    ORB_SLAM2::ORBextractor* orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    cv::Ptr<cv::ORB> cvExtractor = cv::ORB::create(nFeatures, fScaleFactor, nLevels);

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
            cv::cvtColor(imgBuffer, img,  cv::COLOR_RGBA2GRAY);
            img.convertTo(img, CV_8UC1);
            cv::flip(img, img, 0);

            cv::imshow("image", img);
            cv::waitKey(2); // You can replace 2 with 0 if you want the window to wait indefinitely for a key press

            // Detect keypoints
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            if (orbSlamExtractor) {
                (*orbExtractor)(img, cv::Mat(), keypoints, descriptors);
            }
            else {
                cvExtractor->detectAndCompute(img, cv::Mat(), keypoints, descriptors);
            }
            std::cout << "Keypoints size: " << keypoints.size() << ", Descriptor size: " << descriptors.rows << std::endl;

            // Draw keypoints on the image
            cv::Mat image_keypoints;
            cv::drawKeypoints(img, keypoints, image_keypoints);

            cv::imshow("image_keypoints", image_keypoints);
            cv::waitKey(2);

            // Save the x and y values of the keypoints to a vector
            std::vector<KeyPoint3D> keypoints3D;
            for (int i = 0; i < keypoints.size(); i++)
            {
                keypoints3D.push_back(KeyPoint3D(keypoints[i], descriptors.row(i)));
            }

            cv::Mat depth(viewport_size[3], viewport_size[2], CV_32FC1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_DEPTH_COMPONENT, GL_FLOAT, depth.data);
            
            // Convert keypoints pixels to keypoints 3d points
            for (int i = 0; i < keypoints.size(); i++)
            {
                cv::Point3f point_float = convert2Dto3D(keypoints3D[i], K_cv, depth, s_cam);
                cv::Point3d point = cv::Point3d(point_float.x, point_float.y, point_float.z);
                keypoints3D[i].point = point;
            }

            std::string keypoints_csv_path = frame_name + "_orbs_with_3d.csv";
            cv::flip(imgBuffer, imgBuffer, 0);
            cv::imwrite(frame_name + ".png", imgBuffer);

            saveKeypoints3DToCSV(keypoints3D, keypoints_csv_path);

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), keypoints3D);
        }

        pangolin::FinishFrame();
    }

    return 0;
}