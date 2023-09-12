//
// Created by tzuk on 6/4/23.
//

#include "simulator.h"

cv::Mat Simulator::getCurrentLocation() {
    locationLock.lock();
    cv::Mat locationCopy = Tcw.clone();
    locationLock.unlock();
    return locationCopy;
}

Simulator::Simulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo,
                     bool trackImages,
                     bool saveMap, std::string simulatorOutputDirPath, bool loadMap, std::string mapLoadPath,
                     double movementFactor,
                     std::string vocPath,
                     double speedFactor) : stopFlag(false), ready(false), saveMapSignal(false),
                                            track(false),
                                            movementFactor(movementFactor), modelPath(model_path), modelTextureNameToAlignTo(modelTextureNameToAlignTo),
                                            isSaveMap(saveMap),
                                            trackImages(trackImages), cull_backfaces(false),
                                            viewportDesiredSize(640, 480),
                                            speedFactor(speedFactor) {
    cv::FileStorage fSettings(ORBSLAMConfigFile, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int nLevels = fSettings["ORBextractor.nLevels"];

    SLAM = std::make_shared<ORB_SLAM2::System>(vocPath, ORBSLAMConfigFile, ORB_SLAM2::System::MONOCULAR, true, trackImages,
                                               loadMap,
                                               mapLoadPath,
                                               true);
    bool isLocalized = true;
//    cv::Mat lastLocalizedLocation = this->getCurrentLocation();
//    auto currentLocation = ORB_SLAM2::Converter::toVector3d(simulator.getCurrentLocation().rowRange(0, 2).col(3));
//
//    double currentAngle = std::atan2(currentLocation.z(),currentLocation.x());
//    double targetAngle = std::atan2(exitPoints.front().second.z(),exitPoints.front().second.x());
//    int angle_difference = targetAngle;

    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    simulatorOutputDir = simulatorOutputDirPath + "/" + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);

}

Simulator::~Simulator() {
    delete(this->orbExtractor);
}

void Simulator::command(std::string &command, int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    std::istringstream iss(command);
    std::string c;
    double value;
    iss >> c;
    if (commandMap.count(c) && commandMap[c]) {

        std::string stringValue;
        iss >> stringValue;
        value = std::stod(stringValue);
        std::cout << command << std::endl;
        applyCommand(c, value, intervalUsleep, fps, totalCommandTimeInSeconds);
        lastCommand = command;
    } else {
        std::cout << "the command " << c << " is not supported and will be skipped" << std::endl;
    }
}

void Simulator::simulatorRunThread() {
    pangolin::CreateWindowAndBind("Main", viewportDesiredSize[0], viewportDesiredSize[1]);
    glEnable(GL_DEPTH_TEST);
    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2),
                                       K(1, 2), 0.1, 20),
            pangolin::ModelViewLookAt(0.1, -0.1, 0.3, 0, 0, 0, 0.0, -1.0,
                                      pangolin::AxisY)); // the first 3 value are meaningless because we change them later

    bool show_bounds = false;
    bool show_axis = false;
    bool show_x0 = false;
    bool show_y0 = false;
    bool show_z0 = false;
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('k', [&]() { stopFlag = !stopFlag; });
    pangolin::RegisterKeyPressCallback('t', [&]() { track = !track; });
    pangolin::RegisterKeyPressCallback('m', [&]() { saveMapSignal = !saveMapSignal; });
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });
    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 1 * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -1 * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('r', [&]() { applyUpModelCam(s_cam, -movementFactor * this->speedFactor);});// ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor * this->speedFactor); });
    pangolin::RegisterKeyPressCallback('1', [&]() { slower(); });
    pangolin::RegisterKeyPressCallback('2', [&]() { faster(); });

    const pangolin::Geometry modelGeometry = pangolin::LoadGeometry(modelPath);
    alignModelViewPointToSurface(modelGeometry, modelTextureNameToAlignTo);

    auto mvm = pangolin::ModelViewLookAt(-3.8966, 0.974479 ,4.55261 ,-3.566 ,0.98659, 1.03574, 0, 1.0, 0);
    s_cam.SetModelViewMatrix(mvm);

    geomToRender = pangolin::ToGlGeometry(modelGeometry);
    for (auto &buffer: geomToRender.buffers) {
        buffer.second.attributes.erase("normal");
    }
    cv::Mat img;

    auto LoadProgram = [&]() {
        program.ClearShaders();
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        program.Link();
    };
    LoadProgram();
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, ((float) -viewportDesiredSize[0] / (float) viewportDesiredSize[1]))
            .SetHandler(&handler);
    int numberOfFramesForOrbslam = 0;
    while (!pangolin::ShouldQuit() && !stopFlag) {
        ready = true;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (d_cam.IsShown()) {
            d_cam.Activate();

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            program.Bind();
            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
            pangolin::GlDraw(program, geomToRender, nullptr);
            program.Unbind();

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size);

            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY);
            img.convertTo(img, CV_8UC1);
            cv::flip(img, img, 0);

            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
            double timestamp = value.count() / 1000.0;
            if (saveMapSignal) {
                saveMapSignal = false;
                char time_buf[21];
                time_t now_t;
                std::time(&now_t);
                std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now_t));
                std::string currentTime(time_buf);
                saveMap(currentTime);
                SLAM->SaveMap(simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin");
                std::cout << "new map saved to " << simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin"
                          << std::endl;
            }

            if (track) {
                locationLock.lock();
                if (trackImages){
                    Tcw = SLAM->TrackMonocular(img, timestamp);
                }else{
                    std::vector<cv::KeyPoint> pts;
                    cv::Mat mDescriptors;
                    orbExtractor->operator()(img, cv::Mat(), pts, mDescriptors);
                    Tcw = SLAM->TrackMonocular(mDescriptors, pts, timestamp);
                }
//                auto current_pos = getCurrentLocation();
//                if (!current_pos.empty())
//                {
//                    auto currentLocation = ORB_SLAM2::Converter::toVector3d(getCurrentLocation().rowRange(0, 2).col(3));
//                    std::cout << "x = " << currentLocation.x() << " y = " << currentLocation.y() << " z = " << currentLocation.z();
//                }

                locationLock.unlock();
            }

            //std::cout << s_cam.GetModelViewMatrix() << std::endl;//TODO: remove this before commit

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

//             drawPoints(seenPoints, keypoint_points);
        }

        pangolin::FinishFrame();
    }
    if (isSaveMap) {
        SaveMap();
    }
    SLAM->Shutdown();
}

void Simulator::SaveMap(){
    saveMap("final");
    SLAM->SaveMap(simulatorOutputDir + "/finalSimulatorCloudPoint.bin");
    std::cout << "new map saved to " << simulatorOutputDir + "/finalSimulatorCloudPoint.bin" << std::endl;
}

std::thread Simulator::run() {
    std::thread thread(&Simulator::simulatorRunThread, this);
    return thread;
}

void Simulator::saveMap(std::string prefix) {
    std::ofstream pointData;

    pointData.open(simulatorOutputDir + "/cloud" + prefix + ".csv");
    for (auto &p: SLAM->GetMap()->GetAllMapPoints()) {
        if (p != nullptr && !p->isBad()) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();
            p->UpdateNormalAndDepth();
            cv::Mat Pn = p->GetNormal();
            Pn.convertTo(Pn, CV_64F);
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","
                      << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
            std::map<ORB_SLAM2::KeyFrame *, size_t> observations = p->GetObservations();
            for (auto obs: observations) {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first;
                if (!currentFrame->image.empty()) {
                    size_t pointIndex = obs.second;
                    cv::KeyPoint keyPoint = currentFrame->mvKeysUn[pointIndex];
                    cv::Point2f featurePoint = keyPoint.pt;
                    pointData << "," << currentFrame->mnId << "," << featurePoint.x << "," << featurePoint.y;
                }
            }
            pointData << std::endl;
        }
    }
    pointData.close();

}

void Simulator::extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                               Eigen::MatrixXf &surface) {
    std::vector<Eigen::Vector3<unsigned int>> surfaceIndices;
    for (auto &o: modelGeometry.objects) {
        if (o.first == modelTextureNameToAlignTo) {
            const auto &it_vert = o.second.attributes.find("vertex_indices");
            if (it_vert != o.second.attributes.end()) {
                const auto &vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);
                for (size_t i = 0; i < vs.h; ++i) {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i));
                    surfaceIndices.emplace_back(v);
                }
            }
        }
    }
    surface = Eigen::MatrixXf(surfaceIndices.size() * 3, 3);
    int currentIndex = 0;
    for (const auto &b: modelGeometry.buffers) {
        const auto &it_vert = b.second.attributes.find("vertex");
        if (it_vert != b.second.attributes.end()) {
            const auto &vs = std::get<pangolin::Image<float>>(it_vert->second);
            for (auto &row: surfaceIndices) {
                for (auto &i: row) {
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i));
                    surface.row(currentIndex++) = v;
                }
            }
        }
    }
}

void Simulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    double rand = double(value) * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, c, -s,
            0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();;
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

void Simulator::intervalOverCommand(
        const std::function<void(pangolin::OpenGlRenderState &, double &)> &func, double value,
        int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    double intervalValue = this->speedFactor * value / (fps * totalCommandTimeInSeconds);
    int intervalIndex = 0;
    while (intervalIndex <= fps * totalCommandTimeInSeconds) {
        usleep(intervalUsleep);
        func(s_cam, intervalValue);
        intervalIndex += 1;
    }
}

void Simulator::applyCommand(std::string &command, double value, int intervalUsleep, double fps,
                             int totalCommandTimeInSeconds) {
    if (command == "cw") {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "ccw") {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, -1 * value,
                            intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "forward") {
        intervalOverCommand(Simulator::applyForwardToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "back") {
        intervalOverCommand(Simulator::applyForwardToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "right") {
        intervalOverCommand(Simulator::applyRightToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "left") {
        intervalOverCommand(Simulator::applyRightToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "up") {
        intervalOverCommand(Simulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "down") {
        intervalOverCommand(Simulator::applyUpModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    }
}

void Simulator::applyUpModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(1, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(0, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void Simulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
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

void
Simulator::alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry,
                                        std::string modelTextureNameToAlignTo) {
    Eigen::MatrixXf surface;
    extractSurface(modelGeometry, modelTextureNameToAlignTo, surface);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(surface, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.computeV();
    Eigen::Vector3f v = svd.matrixV().col(2);
    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,
                                               -1.0,
                                               pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), 0.1, 20);
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    applyPitchRotationToModelCam(s_cam, -90);
}

std::shared_ptr<ORB_SLAM2::System> Simulator::getSLAM() {
    return SLAM;
}

void Simulator::navigateToPoint(const Eigen::Vector3f& point)
{
    //TODO:Remember to add localization

    auto current_location = getCurrentLocation();
    while (current_location.empty())
        sleep(0.5);

    auto current_angle = ExtractYaw();

    Eigen::Vector3f currentTranslation = ExtractTranslation();

    cv::Point3f dest_direction_vector(point.x() - currentTranslation.x(),  point.y() - currentTranslation.y(), point.z() - currentTranslation.z());
//    auto currentLocation = ORB_SLAM2::Converter::toVector3d(getCurrentLocation().rowRange(0, 2).col(3));

//    double currentAngle = std::atan2(currentLocation.z(),currentLocation.x());

    std::cout << "Current Angle " << current_angle << std::endl;
    auto dest_angle = static_cast<float>(std::atan2(dest_direction_vector.y, dest_direction_vector.x) * 180 / M_PI);
    std::cout << "Dest Angle " << dest_angle << std::endl;
    float angle_difference = current_angle - dest_angle;

    std::cout << "Angle difference " << angle_difference << std::endl;

    std::string rotCommand;
    if (angle_difference<0){
        rotCommand = "ccw " + std::to_string(std::abs(angle_difference));

    }else{
        rotCommand = "cw " + std::to_string(angle_difference);
    }
    std::cout << rotCommand << std::endl;
    command(rotCommand);
//    double distanceToTarget = sqrt(std::pow(translation_vector.at<float>(0)-point.x(), 2) + std::pow(translation_vector.at<float>(1)-point.y(), 2) + std::pow(translation_vector.at<float>(2)-point.z(), 2));
    double distanceToTarget = (currentTranslation - point).norm();

    std::string forwardCommand = "forward " + std::to_string( 3*distanceToTarget);
    std::cout << forwardCommand << std::endl;
    command(forwardCommand);
}

void Simulator::setSpeed(double speed)
{
    this->speedFactor = speed;
}

double Simulator::getSpeed() const
{
    return this->speedFactor;
}

void Simulator::faster()
{
    if(this->speedFactor < 3.0){
        this->speedFactor += 0.1;
    }
}

void Simulator::slower()
{
    if(this->speedFactor > 0.5){
        this->speedFactor -= 0.1;
    }
}


cv::Point3f Simulator::rotation_matrix_to_euler_angles(const cv::Mat &R) {
    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

    bool singular = sy < 1e-6;  // If

    float x, y, z;
    if (!singular) {
        x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
        y = atan2(-R.at<float>(2, 0), sy);
        z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
    } else {
        x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
        y = atan2(-R.at<float>(2, 0), sy);
        z = 0;
    }
    return cv::Point3f(x * 180 / CV_PI, y * 180 / CV_PI, z * 180 / CV_PI);
}

cv::Mat Simulator::align(const cv::Mat& pose) const
{
    cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).clone().t();
    cv::Mat tcw = pose.rowRange(0, 3).col(3).clone();

    cv::Mat twc = -Rwc*tcw;

    cv::Mat aligned_Rwc = R_align * Rwc;
    cv::Mat aligned_twc = R_align * (twc - mu_align);

    cv::Mat align_pose;

    align_pose = cv::Mat::eye(4,4,CV_32F);

    cv::Mat aligned_Rcw = aligned_Rwc.t();
    cv::Mat aligned_tcw = -aligned_Rcw * aligned_twc;
    aligned_Rcw.copyTo(align_pose.rowRange(0,3).colRange(0,3));
    aligned_tcw.copyTo(align_pose.rowRange(0,3).col(3));

    return align_pose;
}

void Simulator::Initialization(){
    while (SLAM->GetTracker()->mState != ORB_SLAM2::Tracking::OK)
    {
        std::string c = "left 0.3";
        command(c);
        c = "right 0.3";
        command(c);
    }
    auto [R, mu] = SLAM->GetMap()->calculate_align_matrices();
    R_align = R;
    mu_align = mu;
}

cv::Mat Simulator::rotation_matrix_from_pose(const cv::Mat& pose){
    return pose.rowRange(0, 3).colRange(0, 3).clone().t();

}

float Simulator::ExtractYaw(){
    auto current_pose = getCurrentLocation();
    if (!current_pose.empty()) {
        auto aligned_pose = align(current_pose);
//        std::cout << "---------------------" << std::endl;
//        std::cout << "aligned pose function " << aligned_pose << std::endl;
//        std::cout << "---------------------" << std::endl;
        auto Rwc = rotation_matrix_from_pose(aligned_pose);
//        std::cout << "---------------------" << std::endl;
//        std::cout << "Rwc function" << Rwc << std::endl;
//        std::cout << "---------------------" << std::endl;
        auto angles = rotation_matrix_to_euler_angles(Rwc);
//        std::cout << "---------------------" << std::endl;
//        std::cout << "angles function" << angles << std::endl;
//        std::cout << "---------------------" << std::endl;
        auto currentAngle = angles.z + 90;
        return currentAngle;
    }
    return 100000;
}

Eigen::Vector3f Simulator::ExtractTranslation()
{
    Eigen::Vector3f translation;
    auto current_pose = getCurrentLocation();
    if (!current_pose.empty()) {
        auto aligned_pose = align(current_pose);
        translation = translation_vector_from_pose(current_pose);
    }
    return translation;
}

Eigen::Vector3f Simulator::translation_vector_from_pose(const cv::Mat& pose){
    auto tcw = pose.rowRange(0, 3).col(3);
    auto Rwc = pose.rowRange(0, 3).colRange(0, 3).clone().t();
    cv::Mat translation = -Rwc * tcw;
    cv::Mat translation_wc = R_align * (translation - mu_align);
    Eigen::Vector3f eigen_translation(translation_wc.at<float>(0), translation_wc.at<float>(1), translation_wc.at<float>(2));
    return eigen_translation;
}
