#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>

#include "include/run_model/TextureShader.h"
#include "include/Auxiliary.h"

#define NEAR_PLANE 0.1
#define FAR_PLANE 20

#define ANIMATION_DURATION 5.0
#define ANIMATION_STEPS 100.0

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

Eigen::Matrix4f loadMatrixFromFile(const std::string &filename) {
    Eigen::Matrix4f matrix;
    std::ifstream infile(filename);

    if (infile.is_open()) {
        int row = 0;
        std::string line;
        while (std::getline(infile, line)) {
            std::istringstream ss(line);
            std::string value;
            int col = 0;
            while (std::getline(ss, value, ',')) {
                matrix(row, col) = std::stof(value);
                col++;
            }
            row++;
        }
        infile.close();
    } else {
        std::cerr << "Cannot open file: " << filename << std::endl;
    }

    return matrix;
}

cv::Point3d transformPoint(const cv::Point3d &point, const Eigen::Matrix4f &transformation) {
    Eigen::Vector4f eigenPoint = Eigen::Vector4f((float)point.x, (float)point.y, (float)point.z, 1.0f);
    Eigen::Vector4f transformedPoint = transformation * eigenPoint;
    return cv::Point3d((double)transformedPoint(0), (double)transformedPoint(1), (double)transformedPoint(2));
}

std::vector<cv::Point3d> loadPoints() {
    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string cloud_points = std::string(data["mapInputDir"]) + "cloud1.csv";

    std::vector<cv::Point3d> points;

    pointData.open(cloud_points, std::ios::in);

    while (!pointData.eof()) {
        row.clear();

        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            try
            {
                std::stod(word);
            }
            catch(std::out_of_range)
            {
                word = "0";
            }
            row.push_back(word);
        }
        points.push_back(cv::Point3d(std::stod(row[0]), std::stod(row[1]), std::stod(row[2])));
    }
    pointData.close();

    return points;
}

void applyYawRotationToModelCam(pangolin::OpenGlRenderState *s_cam, double value) {
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

    s_cam->SetModelViewMatrix(newModelView);
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

    cv::Mat Twc;
    bool use_lab_icp = bool(data["useLabICP"]);
    std::cout << use_lab_icp << std::endl;

    std::string transformation_matrix_csv_path;
    if (use_lab_icp)
    {
        transformation_matrix_csv_path = std::string(data["framesOutput"]) + "frames_lab_transformation_matrix.csv";
    }
    else
    {
        transformation_matrix_csv_path = std::string(data["framesOutput"]) + "frames_transformation_matrix.csv";
    }
    Eigen::Matrix4f transformation = loadMatrixFromFile(transformation_matrix_csv_path);
    std::cout << transformation << std::endl;

    double targetPointX = data["targetPointX"];
    double targetPointY = data["targetPointY"];
    double targetPointZ = data["targetPointZ"];

    std::vector<cv::Point3d> points_to_draw;
    std::vector<cv::Point3d> points = loadPoints();

    for (auto point : points) {
        points_to_draw.push_back(transformPoint(point, transformation));
    }

    const Eigen::Vector3d start_point(
        s_cam.GetModelViewMatrix().m[12],
        s_cam.GetModelViewMatrix().m[13],
        s_cam.GetModelViewMatrix().m[14]
    );
    cv::Point3d target_point_transform(targetPointX, targetPointY, targetPointZ);
    target_point_transform = transformPoint(target_point_transform, transformation);
    const Eigen::Vector3d target_point(target_point_transform.x, start_point[1] - 1, target_point_transform.z + 2);

    Eigen::Vector3d Pick_w = handler.Selected_P_w();
    std::vector<Eigen::Vector3d> Picks_w;

    const Eigen::Vector3d step = (target_point - start_point) / ANIMATION_STEPS;
    const float yaw_movement = data["yawMovement"];

    for (int i = 1; i <= ANIMATION_STEPS; ++i) {
        // Update the camera's model view matrix        
        applyYawRotationToModelCam(&s_cam, yaw_movement/ANIMATION_STEPS);

        // Render the scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                      << std::endl;
        }
        if (d_cam.IsShown()) {
            d_cam.Activate();

            // Load any pending geometry to the GPU.
            if (d_cam.IsShown()) {
                d_cam.Activate();
            }

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            default_prog.Bind();
            default_prog.SetUniform("KT_cw",  s_cam.GetProjectionMatrix() *  s_cam.GetModelViewMatrix());
            pangolin::GlDraw( default_prog, geomToRender, nullptr);
            default_prog.Unbind();

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), points_to_draw);

            pangolin::FinishFrame();
        }

        // Delay to control the animation speed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (int i = 1; i <= ANIMATION_STEPS; ++i) {
        const Eigen::Vector3d current_point = start_point + step * i;

        // Update the camera's model view matrix
        pangolin::OpenGlMatrix& mv_matrix = s_cam.GetModelViewMatrix();
        mv_matrix.m[12] = current_point[0];
        mv_matrix.m[13] = current_point[1];
        mv_matrix.m[14] = current_point[2];

        // Render the scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                      << std::endl;
        }
        if (d_cam.IsShown()) {
            d_cam.Activate();

            // Load any pending geometry to the GPU.
            if (d_cam.IsShown()) {
                d_cam.Activate();
            }

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            default_prog.Bind();
            default_prog.SetUniform("KT_cw",  s_cam.GetProjectionMatrix() *  s_cam.GetModelViewMatrix());
            pangolin::GlDraw( default_prog, geomToRender, nullptr);
            default_prog.Unbind();

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            drawPoints(std::vector<cv::Point3d>(), points_to_draw);

            pangolin::FinishFrame();
        }

        // Delay to control the animation speed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

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

            s_cam.Apply();

            glDisable(GL_CULL_FACE);
        }
    
        drawPoints(std::vector<cv::Point3d>(), points_to_draw);

        pangolin::FinishFrame();
    }

    return 0;
}
