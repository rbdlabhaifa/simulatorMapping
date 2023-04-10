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

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "include/run_model/shader.h"
#include "include/run_model/util.h"
#include "include/Auxiliary.h"

#define POINT_SIZE 2

#define X_OFFSET -5.7
#define Y_OFFSET -0.8
#define Z_OFFSET -1.04
#define YAW_OFFSET -2.56
#define PITCH_OFFSET 1.12
#define ROLL_OFFSET 0.95
#define SCALE_FACTOR 1

#define POINT_SIZE 20

#define X_OFFSET 0.1885
#define Y_OFFSET -0.03
#define Z_OFFSET -0.135
#define YAW_OFFSET 0.2
#define PITCH_OFFSET 0.5
#define ROLL_OFFSET 0.3
#define SCALE_FACTOR 0.0305

std::vector<cv::Point3d> getPointsFromPos(const std::string cloud_points, const cv::Point3d camera_position, double yaw, double pitch, double roll, cv::Mat &Twc)
{
    double fx = 620;
    double fy = 620;
    double cx = 320;
    double cy = 244;
    int width = 640;
    int height = 480;

    double minX = 3.7;
    double maxX = width;
    double minY = 3.7;
    double maxY = height;

    Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
    Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(-roll, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Tcw_eigen.block<3, 1>(0, 3) << -camera_position.x, camera_position.y, -camera_position.z;

    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw.at<double>(i,j) = Tcw_eigen(i,j);
        }
    }

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat mOw = -Rcw.t()*tcw;

    /* Create Matrix for s_cam */
    Eigen::Matrix4d tmp_Tcw_eigen = Eigen::Matrix4d::Identity();
    tmp_Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    tmp_Tcw_eigen.block<3, 1>(0, 3) << camera_position.x, camera_position.y, camera_position.z;

    cv::Mat tmpTcw = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            tmpTcw.at<double>(i,j) = tmp_Tcw_eigen(i,j);
        }
    }

    cv::Mat tmpRcw = tmpTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tmpRwc = tmpRcw.t();
    cv::Mat tmptcw = tmpTcw.rowRange(0,3).col(3);
    cv::Mat tmpMOw = -tmpRcw.t()*tmptcw;

    Twc = cv::Mat::eye(4,4,tmpTcw.type());
    tmpRwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Twc.at<double>(12) = mOw.at<double>(0);
    Twc.at<double>(13) = mOw.at<double>(1);
    Twc.at<double>(14) = mOw.at<double>(2);

    /* End create Matrix for s_cam */

    std::vector<cv::Vec<double, 8>> points;

    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;

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
        points.push_back(cv::Vec<double, 8>(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7])));
    }
    pointData.close();

    std::vector<cv::Point3d> seen_points;

    for(cv::Vec<double, 8>  point : points)
    {
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point[0];
        worldPos.at<double>(1) = point[1];
        worldPos.at<double>(2) = point[2];

        const cv::Mat Pc = Rcw*worldPos+tcw;
        const double &PcX = Pc.at<double>(0);
        const double &PcY= Pc.at<double>(1);
        const double &PcZ = Pc.at<double>(2);

        // Check positive depth
        if(PcZ<0.0f)
            continue;

        // Project in image and check it is not outside
        const double invz = 1.0f/PcZ;
        const double u=fx*PcX*invz+cx;
        const double v=fy*PcY*invz+cy;

        if(u<minX || u>maxX)
            continue;
        if(v<minY || v>maxY)
            continue;

        // Check distance is in the scale invariance region of the MapPoint
        const double minDistance = point[3];
        const double maxDistance = point[4];
        const cv::Mat PO = worldPos-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point[5];
        Pn.at<double>(1) = point[6];
        Pn.at<double>(2) = point[7];

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5)
            continue;

        seen_points.push_back(cv::Point3d(worldPos.at<double>(0), worldPos.at<double>(1), worldPos.at<double>(2)));
    }

    return seen_points;
}

void drawMapPoints(std::vector<cv::Point3d> seen_points, std::vector<cv::Point3d> new_points_seen)
{
    glPointSize(POINT_SIZE);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : seen_points)
    {
        glVertex3f((float)((point.x - X_OFFSET) / SCALE_FACTOR), (float)((point.y - Y_OFFSET) / SCALE_FACTOR), (float)((point.z - Z_OFFSET) / SCALE_FACTOR));
    }
    glEnd();

    glPointSize(POINT_SIZE);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(auto point : new_points_seen)
    {
        glVertex3f((float)((point.x - X_OFFSET) / SCALE_FACTOR), (float)((point.y - Y_OFFSET) / SCALE_FACTOR), (float)((point.z - Z_OFFSET) / SCALE_FACTOR));
    }
    std::cout << new_points_seen.size();

    glEnd();
}

int main( int argc, char** argv )
{
    const float w = 640.0f;
    const float h = 480.0f;
    const float f = 300.0f;

    using namespace pangolin;

    argagg::parser argparser {{
        { "help", {"-h", "--help"}, "Print usage information and exit.", 0},
        { "model", {"-m","--model","--mesh"}, "3D Model to load (obj or ply)", 1},
        { "matcap", {"--matcap"}, "Matcap (material capture) images to load for shading", 1},
        { "envmap", {"--envmap","-e"}, "Equirect environment map for skybox", 1},
        { "mode", {"--mode"}, "Render mode to use {show_uv, show_texture, show_color, show_normal, show_matcap, show_vertex}", 1},
        { "bounds", {"--aabb"}, "Show axis-aligned bounding-box", 0},
        { "show_axis", {"--axis"}, "Show axis coordinates for Origin", 0},
        { "show_x0", {"--x0"}, "Show X=0 Plane", 0},
        { "show_y0", {"--y0"}, "Show Y=0 Plane", 0},
        { "show_z0", {"--z0"}, "Show Z=0 Plane", 0},
        { "cull_backfaces", {"--cull"}, "Enable backface culling", 0},
        { "spin", {"--spin"}, "Spin models around an axis {none, negx, x, negy, y, negz, z}", 1},
    }};

    argagg::parser_results args = argparser.parse(argc, argv);
    if ( (bool)args["help"] || !args.has_option("model")) {
        std::cerr << "usage: ModelViewer [options]" << std::endl
                  << argparser << std::endl;
        return 0;
    }

    // Options
    enum class RenderMode { uv=0, tex, color, normal, matcap, vertex, num_modes };
    const std::string mode_names[] = {"SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP", "SHOW_VERTEX"};
    const std::string spin_names[] = {"NONE", "NEGX", "X", "NEGY", "Y", "NEGZ", "Z"};
    const char mode_key[] = {'u','t','c','n','m','v'};
    RenderMode current_mode = RenderMode::normal;
    for(int i=0; i < (int)RenderMode::num_modes; ++i)
        if(pangolin::ToUpperCopy(args["mode"].as<std::string>("SHOW_NORMAL")) == mode_names[i])
            current_mode = RenderMode(i);
    pangolin::AxisDirection spin_direction = pangolin::AxisNone;
    for(int i=0; i <= (int)pangolin::AxisZ; ++i)
        if(pangolin::ToUpperCopy(args["spin"].as<std::string>("none")) == spin_names[i])
            spin_direction = pangolin::AxisDirection(i);

    bool show_bounds = args.has_option("bounds");
    bool show_axis = args.has_option("show_axis");
    bool show_x0 = args.has_option("show_x0");
    bool show_y0 = args.has_option("show_y0");
    bool show_z0 = args.has_option("show_z0");
    bool cull_backfaces = args.has_option("cull_backfaces");
    int mesh_to_show = -1;

    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", w, h);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w, h, f, f, w/2.0, h/2.0, 0.1, 1000),
        pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
            .SetHandler(&handler);

    // Load Geometry asynchronously
    std::vector<std::future<pangolin::Geometry>> geom_to_load;
    for(const auto& filename : ExpandGlobOption(args["model"]))
    {
        geom_to_load.emplace_back( std::async(std::launch::async,[filename](){
            return pangolin::LoadGeometry(filename);
        }) );
    }

    // Render tree for holding object position
    RenderNode root;
    std::vector<std::shared_ptr<GlGeomRenderable>> renderables;
    pangolin::AxisDirection spin_other = pangolin::AxisNone;
    auto spin_transform = std::make_shared<SpinTransform>(spin_direction);
    auto show_renderable = [&](int index){
        mesh_to_show = index;
        for(size_t i=0; i < renderables.size(); ++i) {
            if(renderables[i]) renderables[i]->show = (index == -1 || index == (int)i) ? true : false;
        }
    };

    // Pull one piece of loaded geometry onto the GPU if ready
    Eigen::AlignedBox3f total_aabb;
    auto LoadGeometryToGpu = [&]()
    {
        for(auto& future_geom : geom_to_load) {
            if( future_geom.valid() && is_ready(future_geom) ) {
                auto geom = future_geom.get();
                auto aabb = pangolin::GetAxisAlignedBox(geom);
                total_aabb.extend(aabb);
                const Eigen::Vector3f center = total_aabb.center();
                const Eigen::Vector3f view = center + Eigen::Vector3f(1.2, 0.8,1.2) * std::max( (total_aabb.max() - center).norm(), (center - total_aabb.min()).norm());
                const auto mvm = pangolin::ModelViewLookAt(view[0], view[1], view[2], center[0], center[1], center[2], pangolin::AxisY);
                const double far = 100.0*(total_aabb.max() - total_aabb.min()).norm();
                const double near = far / 1e6;
                const auto proj = pangolin::ProjectionMatrix(w, h, f, f, w/2.0, h/2.0, near, far );
                s_cam.SetModelViewMatrix(mvm);
                s_cam.SetProjectionMatrix(proj);

                auto renderable = std::make_shared<GlGeomRenderable>(pangolin::ToGlGeometry(geom), aabb);
                renderables.push_back(renderable);
                RenderNode::Edge edge = { spin_transform, { renderable, {} } };
                root.edges.emplace_back(std::move(edge));
                break;
            }
        }
    };

    // Load Any matcap materials
    size_t matcap_index = 0;
    std::vector<pangolin::GlTexture> matcaps = TryLoad<pangolin::GlTexture>(ExpandGlobOption(args["matcap"]), [](const std::string& f){
        return pangolin::GlTexture(pangolin::LoadImage(f));
    });

    // Load Any Environment maps
    size_t envmap_index = 0;
    std::vector<pangolin::GlTexture> envmaps = TryLoad<pangolin::GlTexture>(ExpandGlobOption(args["envmap"]), [](const std::string& f){
        return pangolin::GlTexture(pangolin::LoadImage(f));
    });

    // GlSl Graphics shader program for display
    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&](const RenderMode mode){
        current_mode = mode;
        default_prog.ClearShaders();
        std::map<std::string,std::string> prog_defines;
        for(int i=0; i < (int)RenderMode::num_modes-1; ++i) {
            prog_defines[mode_names[i]] = std::to_string((int)mode == i);
        }
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, default_model_shader, prog_defines);
        default_prog.Link();
    };
    LoadProgram(current_mode);

    pangolin::GlSlProgram env_prog;
    if(envmaps.size()) {
        env_prog.AddShader(pangolin::GlSlAnnotatedShader, equi_env_shader);
        env_prog.Link();
    }

    // Setup keyboard shortcuts.
    for(int i=0; i < (int)RenderMode::num_modes; ++i)
        pangolin::RegisterKeyPressCallback(mode_key[i], [&,i](){LoadProgram((RenderMode)i);});

    pangolin::RegisterKeyPressCallback(PANGO_SPECIAL + PANGO_KEY_RIGHT, [&](){if(renderables.size()) show_renderable((mesh_to_show + 1) % renderables.size());});
    pangolin::RegisterKeyPressCallback(PANGO_SPECIAL + PANGO_KEY_LEFT, [&](){if(renderables.size()) show_renderable((mesh_to_show + renderables.size()-1) % renderables.size());});
    pangolin::RegisterKeyPressCallback(' ', [&](){show_renderable(-1);});

    if(matcaps.size()) {
        pangolin::RegisterKeyPressCallback('=', [&](){matcap_index = (matcap_index+1)%matcaps.size();});
        pangolin::RegisterKeyPressCallback('-', [&](){matcap_index = (matcap_index+matcaps.size()-1)%matcaps.size();});
    }
    if(envmaps.size()) {
        pangolin::RegisterKeyPressCallback(']', [&](){envmap_index = ((envmap_index + 1) % envmaps.size());});
        pangolin::RegisterKeyPressCallback('[', [&](){envmap_index = ((envmap_index + envmaps.size()-1) % envmaps.size());});
    }
    pangolin::RegisterKeyPressCallback('s', [&](){std::swap(spin_transform->dir, spin_other);});
    pangolin::RegisterKeyPressCallback('b', [&](){show_bounds = !show_bounds;});
    pangolin::RegisterKeyPressCallback('0', [&](){cull_backfaces = !cull_backfaces;});

    // Show axis and axis planes
    pangolin::RegisterKeyPressCallback('a', [&](){show_axis = !show_axis;});
    pangolin::RegisterKeyPressCallback('x', [&](){show_x0 = !show_x0;});
    pangolin::RegisterKeyPressCallback('y', [&](){show_y0 = !show_y0;});
    pangolin::RegisterKeyPressCallback('z', [&](){show_z0 = !show_z0;});

    Eigen::Vector3d Pick_w = handler.Selected_P_w();
    std::vector<Eigen::Vector3d> Picks_w;
    cv::Mat Twc;

    while( !pangolin::ShouldQuit() )
    {
        if( (handler.Selected_P_w() - Pick_w).norm() > 1E-6)
        {
            Pick_w = handler.Selected_P_w();
            Picks_w.push_back(Pick_w);
            std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                      << std::endl;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Load any pending geometry to the GPU.
        LoadGeometryToGpu();



        if(d_cam.IsShown()) {
            d_cam.Activate();

            if(cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }

            if(env_prog.Valid()) {
                glDisable(GL_DEPTH_TEST);
                env_prog.Bind();
                const Eigen::Matrix4d mvmat = s_cam.GetModelViewMatrix();
                const Eigen::Matrix3f R_env_cam = mvmat.block<3,3>(0,0).cast<float>().transpose();
                Eigen::Matrix3f Kinv;
                Kinv << 1.0/f, 0.0, -(w/2.0)/f,
                        0.0, 1.0/f, -(h/2.0)/f,
                        0.0, 0.0, 1.0;

                env_prog.SetUniform("R_env_camKinv", (R_env_cam*Kinv).eval());

                const GLint vertex_handle = env_prog.GetAttributeHandle("vertex");
                const GLint xy_handle = env_prog.GetAttributeHandle("xy");

                if(vertex_handle >= 0 && xy_handle >= 0 ) {
                    glActiveTexture(GL_TEXTURE0);
                    envmaps[envmap_index].Bind();
                    const GLfloat ndc[] = { 1.0f,1.0f,  -1.0f,1.0f,  -1.0f,-1.0f,  1.0f,-1.0f };
                    const GLfloat pix[] = { 0.0f,0.0f,  w,0.0f,  w,h,  0.0f,h };

                    glEnableVertexAttribArray(vertex_handle);
                    glVertexAttribPointer(vertex_handle, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), ndc );
                    glEnableVertexAttribArray(xy_handle);
                    glVertexAttribPointer(xy_handle, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), pix );
                    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
                    glDisableVertexAttribArray(vertex_handle);
                    glDisableVertexAttribArray(xy_handle);

                    env_prog.Unbind();
                    glEnable(GL_DEPTH_TEST);
                }
            }

            default_prog.Bind();
            render_tree(
                default_prog, root, s_cam.GetProjectionMatrix(), s_cam.GetModelViewMatrix(),
                matcaps.size() ? &matcaps[matcap_index] : nullptr
            );
            // std::cout << "Projection Matrix: " << s_cam.GetProjectionMatrix() << std::endl;
            // std::cout << "Model View Matrix: " << s_cam.GetModelViewMatrix() << std::endl;
            Eigen::Matrix4d mv_mat = s_cam.GetModelViewMatrix();

            // Compute the camera position by inverting the model-view matrix and extracting the translation component
            Eigen::Matrix4d inv_mv_mat = mv_mat.inverse();
            Eigen::Vector3d cam_pos = inv_mv_mat.block<3,1>(0,3);

            // Compute the yaw, pitch, and roll angles from the rotation component of the model-view matrix
            Eigen::Matrix3d rot_mat = mv_mat.block<3,3>(0,0);
            double yaw = atan2(rot_mat(1,0), rot_mat(0,0));
            double pitch = asin(-rot_mat(2,0));
            double roll = atan2(rot_mat(2,1), rot_mat(2,2));

            std::cout << "Camera position: " << cam_pos << ", yaw: " << yaw << ", pitch: " << pitch << ", roll: " << roll << std::endl;
            default_prog.Unbind();

            s_cam.Apply();
            if(show_x0) pangolin::glDraw_x0(10.0, 10);
            if(show_y0) pangolin::glDraw_y0(10.0, 10);
            if(show_z0) pangolin::glDraw_z0(10.0, 10);
            if(show_axis) pangolin::glDrawAxis(10.0);
            if(show_bounds) pangolin::glDrawAlignedBox(total_aabb);

            glDisable(GL_CULL_FACE);

            std::vector<cv::Point3d> seen_points = getPointsFromPos("/home/liam/dev/rbd/slamMaps/example_mapping11/cloud1.csv", cv::Point3d(cam_pos[0]*SCALE_FACTOR + X_OFFSET, cam_pos[1]*SCALE_FACTOR + Y_OFFSET, cam_pos[2]*SCALE_FACTOR + Z_OFFSET), yaw + YAW_OFFSET, pitch + PITCH_OFFSET, roll + ROLL_OFFSET, Twc);
            drawMapPoints(std::vector<cv::Point3d>(), seen_points);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
