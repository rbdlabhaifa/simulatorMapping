//
// Created by Dean on 9/30/23.
//

#include "polySimulator.h"
#include "navigation/ExitRoomFiles/exit_room_algo.cpp"


// std::vector<std::thread>:: getSimulatorThreadsView();

cv::Mat PolySimulator::getCurrentLocation() {
    locationLock.lock();
    cv::Mat locationCopy = Tcw.clone();
    locationLock.unlock();
    return locationCopy;
}

PolySimulator::PolySimulator(bool _loadCustomMap) : stopFlag(false), ready(false), saveMapSignal(false),
                                            track(false),
                                            isSaveMap(0),
                                            cull_backfaces(false),
                                            viewportDesiredSize(640, 480)
                                            {

    // loading configuration file content
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string ORBSLAMConfigFile = data["DroneYamlPathSlam"];
    std::string vocPath = data["VocabularyPath"];
    this->modelPath = data["modelPath"];
    this->trackImages = data["trackImages"];
    this->movementFactor = data["movementFactor"];
    this->speedFactor = data["simulatorStartingSpeed"];
    std::string simulatorOutputDirPath = data["mapInputDir"];

    bool shouldLoadMap = false;
    // end configuration content loading

    cv::FileStorage fSettings(ORBSLAMConfigFile, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    this->viewpointX = fSettings["RunModel.ViewpointX"];
    this->viewpointY = fSettings["RunModel.ViewpointY"];
    this->viewpointZ = fSettings["RunModel.ViewpointZ"];

    this->loadCustomMap = _loadCustomMap;

    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int nLevels = fSettings["ORBextractor.nLevels"];

    SLAM = std::make_shared<ORB_SLAM2::System>(vocPath, ORBSLAMConfigFile, ORB_SLAM2::System::MONOCULAR, true, shouldLoadMap, "../slamMaps/example.bin", true);
    orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    char time_buf[21];
    time_t now;
    std::time(&now);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));
    std::string currentTime(time_buf);
    simulatorOutputDir = simulatorOutputDirPath + "/" + currentTime + "/";
    std::filesystem::create_directory(simulatorOutputDir);
}

PolySimulator::~PolySimulator() {
    delete(this->orbExtractor);
}

void PolySimulator::command(std::string &command, int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    std::istringstream iss(command);
    std::string c;
    double value;
    iss >> c;
    if (commandMap.count(c) && commandMap[c]) {

        std::string stringValue;
        iss >> stringValue;
        value = std::stod(stringValue);
        applyCommand(c, value, intervalUsleep, fps, totalCommandTimeInSeconds);
    } else {
        std::cout << "the command " << c << " is not supported and will be skipped" << std::endl;
    }
}

void PolySimulator::saveMap(std::string prefix) {
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

void PolySimulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
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

void PolySimulator::intervalOverCommand(
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

void PolySimulator::applyCommand(std::string &command, double value, int intervalUsleep, double fps,
                             int totalCommandTimeInSeconds) {
    if (command == "cw") {
        intervalOverCommand(PolySimulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "ccw") {
        intervalOverCommand(PolySimulator::applyYawRotationToModelCam, -1 * value,
                            intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "forward") {
        intervalOverCommand(PolySimulator::applyForwardToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "back") {
        intervalOverCommand(PolySimulator::applyForwardToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "right") {
        intervalOverCommand(PolySimulator::applyRightToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "left") {
        intervalOverCommand(PolySimulator::applyRightToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "up") {
        intervalOverCommand(PolySimulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "down") {
        intervalOverCommand(PolySimulator::applyUpModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    }
}

void PolySimulator::applyUpModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    camMatrix(1, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void PolySimulator::applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void PolySimulator::applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    camMatrix(0, 3) += value;

    cam.SetModelViewMatrix(camMatrix);
}

void PolySimulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
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

void PolySimulator::setSpeed(double speed)
{
    this->speedFactor = speed;
}

double PolySimulator::getSpeed() const
{
    return this->speedFactor;
}

void PolySimulator::faster()
{
    if(this->speedFactor < 3.0){ 
        this->speedFactor += 0.1;
    }
}

void PolySimulator::slower(){
    if(this->speedFactor > 0.5){
        this->speedFactor -= 0.1;
    }
}

//New Functionalities

//vector(1.0f, 0.0f, 0.0f)

bool PolySimulator::isSetInPlace(){
    return this->setInPlace;
}

void PolySimulator::setInPlaceFlag(bool val){
    this->setInPlace = val;
}

void PolySimulator::startRolling(){
    this->startRoll = true;
}

bool PolySimulator::isTracking(){
    return this->track;
}

//This method is rendering into the screen an OpenGL dot.
void PolySimulator::generateGLPoint(vector<GLfloat> colors, GLfloat size, vector<double> point){
    glColor3f(colors[0], colors[1], colors[2]);
    glPointSize(size); // Adjust the point size as needed
    glBegin(GL_POINTS);
    glVertex3d(point[0], point[1], point[2]); // Replace with the desired 3D coordinate
    glEnd();
}

//This method rendering into the screen an OpenGL line between the two points.
void PolySimulator::generateGLLine(vector<GLfloat> colors, vector<double> point1, vector<double> point2){
    glColor3f(colors[0], colors[1], colors[2]);
    glBegin(GL_LINES);
    glVertex3d(point1[0], point1[1], point1[2]);
    glVertex3d(point2[0], point2[1], point2[2]);
    glEnd();
}

//This method align the current drone position with the axis according to the given x,y,z coordinate.
void PolySimulator::alignCameraWithFloor(int x, int y, int z){
    locationLock.lock();
    auto model_view_at = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    locationLock.unlock();

    // cout << model_view_at << endl;
    model_view_at(0, 3) = x;
    model_view_at(1, 3) = y;
    model_view_at(2, 3) = z;

    model_view_at(0, 0) = -1;
    model_view_at(0, 1) = 0;
    model_view_at(0, 2) = 0;

    model_view_at(1, 0) = 0;
    model_view_at(1, 1) = 1;
    model_view_at(1, 2) = 0;

    model_view_at(2, 0) = 0;
    model_view_at(2, 1) = 0;
    model_view_at(2, 2) = -1;

    locationLock.lock();
    this->s_cam.SetModelViewMatrix(model_view_at);
    locationLock.unlock();
};

//This method align the current drone position with the axis.
void PolySimulator::alignCameraWithFloor(){
    locationLock.lock();
    auto model_view_at = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    locationLock.unlock();

    model_view_at(0, 0) = -1;
    model_view_at(0, 1) = 0;
    model_view_at(0, 2) = 0;

    model_view_at(1, 0) = 0;
    model_view_at(1, 1) = 1;
    model_view_at(1, 2) = 0;

    model_view_at(2, 0) = 0;
    model_view_at(2, 1) = 0;
    model_view_at(2, 2) = -1;

    this->locationLock.lock();
    this->s_cam.SetModelViewMatrix(model_view_at);
    this->locationLock.unlock();
}

bool PolySimulator::rotateAndSlam(cv::Mat& img, bool &down_up_flag, double& timestamp, int &i, double step, double& aggregator){
    cv::Mat val = this->SLAM->TrackMonocular(img, timestamp);

    int started_tracking_already = this->SLAM->GetTracker()->mState;

    if (1){
        if (down_up_flag == true){
            PolySimulator::applyUpModelCam(this->s_cam, -0.05);
            if(i == 10){
                i = 0;
                down_up_flag = false;
            }
        }else {
            PolySimulator::applyUpModelCam(this->s_cam, 0.05);
            if(i == 10){
                i = 0;
                down_up_flag = true;
            }
        }
    }
    if (1){
        if (started_tracking_already == 2) {
            PolySimulator::applyYawRotationToModelCam(this->s_cam, -step);
            aggregator = aggregator + step;
        }
    }
    if (val.empty() == 1 || started_tracking_already == 3){
        if (1) {
            PolySimulator::applyYawRotationToModelCam(this->s_cam, 1*step);
            aggregator = aggregator - 1*step;
        }
        i++;
        return true;
    }
    i++;
    return false;
}

//This method printing the current x,y,z location of the 'drone'.
void PolySimulator::printCurrentLocation(){
    double x, y, z;

    this->locationLock.lock();
    auto model_view_at = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    this->locationLock.unlock();

    x = model_view_at(0, 3);
    y = model_view_at(1, 3);
    z = model_view_at(2, 3);

    cout << x << ' ' << z << " " << y << " " << endl;
}

//This method return the current x,y,z location of the 'drone'.
std::vector<double> PolySimulator::getDroneLocation(){
    std::vector<double> position;

    this->locationLock.lock();
    auto model_view_at = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    this->locationLock.unlock();

    position.push_back(model_view_at(0, 3));
    position.push_back(model_view_at(1, 3));
    position.push_back(model_view_at(2, 3));

    return position;  //x,y,z -> x,z,y
}

//This method converting ORB_SLAM2::MapPoints into a vector<vector<double>> - vector of 3d points.
vector<vector<double>> PolySimulator::fromMapPointsToDoubleVec(vector<ORB_SLAM2::MapPoint*> points){
    vector<vector<double>> data_points;
    
    for (const auto m_point : points){
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(m_point->GetWorldPos());
        vector<double> point;
        point.push_back(v.x());
        point.push_back(v.y());
        point.push_back(v.z());

        data_points.push_back(point);
    }

    return data_points;
}

//This method hard moving the 'drone view' to look at the coordinate given aligned with the axis.
void PolySimulator::lookAtPoint(double x, double y, double z) {
    auto model_view_at = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    // cout << model_view_at << endl;
    model_view_at(0, 3) = x;
    model_view_at(1, 3) = y;
    model_view_at(2, 3) = z;

    model_view_at(0, 0) = -1;
    model_view_at(0, 1) = 0;
    model_view_at(0, 2) = 0;

    model_view_at(1, 0) = 0;
    model_view_at(1, 1) = 1;
    model_view_at(1, 2) = 0;

    model_view_at(2, 0) = 0;
    model_view_at(2, 1) = 0;
    model_view_at(2, 2) = -1;
    
    //pangolin::ModelViewLookAt(model_view_at(0, 3),model_view_at(1, 3),model_view_at(2, 3), -x, y, z, 0, 0, pangolin::AxisY)

    this->s_cam.SetModelViewMatrix(model_view_at);
}

//This method initiazlizing the view camera - which is the drone 'eyes' 
void PolySimulator::setCamera(){
    this->s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
        pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)); // the first 3 value are meaningless because we change them later
}

//function create a pangolin::View instance - D_CAM according to the screen size choosen - aspect ratio.
pangolin::View& PolySimulator::setDCAM(pangolin::Handler3D *handler){
    return pangolin::CreateDisplay().SetBounds(0.0, 1, 0, 1, (float) (-viewportDesiredSize[0] / viewportDesiredSize[1])).SetHandler(handler);
}

bool PolySimulator::SlamCurrentView(cv::Mat img, double timestamp){
    this->Tcw = this->SLAM->TrackMonocular(img, timestamp);
    if (this->Tcw.empty() == 1){
        return true;
    }
    return false;
}

void PolySimulator::moveCameraByStep(double step_x, double step_y, double step_z, double step_size_x1, double step_size_y1, double step_size_z1, double step_size_x2, double step_size_y2, double step_size_z2, double step_size_x3, double step_size_y3, double step_size_z3){
    auto s_cam_mat = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    s_cam_mat(0, 3) += step_x;
    s_cam_mat(1, 3) += step_z;
    s_cam_mat(2, 3) += step_y;

    s_cam_mat(0, 2) += step_size_x1;
    s_cam_mat(1, 2) += step_size_y1;
    s_cam_mat(2, 2) += step_size_z1;

    s_cam_mat(0, 1) += step_size_x2;
    s_cam_mat(1, 1) += step_size_y2;
    s_cam_mat(2, 1) += step_size_z3;

    s_cam_mat(0, 0) += step_size_x3;
    s_cam_mat(1, 0) += step_size_y3;
    s_cam_mat(2, 0) += step_size_z3;

    this->s_cam.SetModelViewMatrix(s_cam_mat);
}

void PolySimulator::moveToXYZ(std::vector<double> dest, int steps){
    this->alignCameraWithFloor();
    // std::vector<double> currentLocation = this->getDroneLocation();
    double x, y, z, dest_x, dest_y, dest_z, step_size_x, step_size_y, step_size_z, x1,y1,z1, x2,y2,z2, x3,y3,z3;
    double step_size_x1, step_size_y1, step_size_z1;
    double step_size_x2, step_size_y2, step_size_z2;
    double step_size_x3, step_size_y3, step_size_z3;

    this->locationLock.lock();
    auto s_cam_mat = pangolin::ToEigen<double>(this->s_cam.GetModelViewMatrix());
    this->locationLock.unlock();
    x = s_cam_mat(0, 3);
    y = s_cam_mat(2, 3);
    z = s_cam_mat(1, 3);

    x1 = s_cam_mat(0, 0);
    y1 = s_cam_mat(0, 1);
    z1 = s_cam_mat(0, 2);

    x2 = s_cam_mat(1, 0);
    y2 = s_cam_mat(1, 1);
    z2 = s_cam_mat(1, 2);

    x3 = s_cam_mat(2, 0);
    y3 = s_cam_mat(2, 1);
    z3 = s_cam_mat(2, 2);
    
    dest_x = dest[0];
    dest_y = dest[1];
    dest_z = dest[2];

    step_size_x = (dest_x - x) / steps;
    step_size_y = (dest_y - y) / steps;
    step_size_z = (dest_z - z) / steps;

    

    step_size_x1 = 0;
    step_size_y1 = 0;
    step_size_z1 = 0;


    step_size_x2 = 0;
    step_size_y2 = 0;
    step_size_z2 = 0;


    step_size_x3 = 0;
    step_size_y3 = 0;
    step_size_z3 = 0;

    for(int i = 0 ; i < steps ; i++){
        this->locationLock.lock();
        this->moveCameraByStep(step_size_x, step_size_y, step_size_z, step_size_x1, step_size_y1, step_size_z1, step_size_x2, step_size_y2, step_size_z2, step_size_x3, step_size_y3, step_size_z3);
        this->locationLock.unlock();
        usleep(50);
    }

    curr_cam_pose = {dest_x, dest_z, dest_y};
}

void PolySimulator::renderPoints(bool showDrone){
    if(showDrone){
        generateGLPoint(vector<GLfloat>{1.0f, 1.0f, 0.0f}, 20.0f, curr_cam_pose); //x,z,y
    }
    for(int i = 0 ; i < pointsToRednder.size() ; i++){
        generateGLPoint(vector<GLfloat>{1.0f, 0.0f, 0.0f}, 20.0f, vector<double>{pointsToRednder[i][0], 0 , pointsToRednder[i][1]});
        generateGLLine(vector<GLfloat>{1.0f, 0.0f, 0.0f}, curr_cam_pose, vector<double>{pointsToRednder[i][0], 0 , pointsToRednder[i][1]});
    }
}

void PolySimulator::addDroneLocation(std::vector<double> currentLocation){
    auto it = std::find(dronePath.begin(), dronePath.end(), currentLocation);

    if (it != dronePath.end()) {
        std::cout << "Location already exist, not adding into array" << std::endl;
    } else {
        dronePath.push_back(currentLocation);
    }
}

void PolySimulator::renderDroneLocations(){
    for(int i = 0 ; i < dronePath.size() ; i++){
        generateGLPoint(vector<GLfloat>{1.0f, 0.0f, 0.0f}, 20.0f, vector<double>{dronePath[i][0], dronePath[i][1], dronePath[i][2]});
    }
}

std::vector<vector<double>> PolySimulator::getCurrentMapPoint(){
    std::vector<vector<double>> vectorOfDoublePoints;
    if(slamFinishedLoopCloser() == true){
        try{
        vectorOfDoublePoints = fromMapPointsToDoubleVec(this->SLAM->GetMap()->GetAllMapPoints());
        }
        catch(exception e){
        std::cout << "Error occured while trying to get the current ORB-SLAM2's Map." << std::endl;
        }
    } else {
        std::cout << "Slam did not finished it's mapping process." << std::endl;
    }
    return vectorOfDoublePoints;
}

cv::Mat PolySimulator::CaptureFrame(pangolin::View& camera_view) {
    int viewport_size[4];
    glGetIntegerv(GL_VIEWPORT, viewport_size);

    pangolin::Image<unsigned char> buffer;
    pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
    buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);

    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

    cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);

    cv::Mat img;
    cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY);
    img.convertTo(img, CV_8UC1);
    cv::flip(img, img, 0);

    return img.clone();
}

//This function defines the Pangolin interactive window for the user.
//By choosing a 'char' value to attach the pangolin interactive window you can modify different actions.
void PolySimulator::definePangolinRegKeys(){

    pangolin::RegisterKeyPressCallback('9', [&]() { this->reflectViewBool(); });
    // pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });

    // Show axis and axis planes
//     pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; });
//    pangolin::RegisterKeyPressCallback('k', [&]() { stopFlag = !stopFlag; });
    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 1); });
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -1); });
    pangolin::RegisterKeyPressCallback('r', [&]() { applyUpModelCam(s_cam, -movementFactor); });    // ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('1', [&]() { slower(); });
    pangolin::RegisterKeyPressCallback('2', [&]() { faster(); });
}


//This is the main function that rendering using 'Pangolin' the main view of the simulator.
void PolySimulator::RunPolyModel(){
    cv::Mat img;

    //Creating Pangolin window and loading the model.
    // Create Window for rendering
    pangolin::CreateWindowAndBind("Main", viewportDesiredSize[0], viewportDesiredSize[1]);
    //needed for object rendering
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    setCamera();

    //Create Interactive View in window
    // pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = setDCAM(new pangolin::Handler3D(s_cam));
    this->simD_CAM = d_cam;

    //allow the user to decide which map he wants to load
    pangolin::Geometry temp_geom_load;
    if (this->loadCustomMap){
        std::cout << "Please insert a path to the desired .obj model" << std::endl;
        std::string customPath;
        std::cin >> customPath;
        temp_geom_load = pangolin::LoadGeometry(customPath);
    }else {
        temp_geom_load = pangolin::LoadGeometry(modelPath);
    }

    auto aabb = pangolin::GetAxisAlignedBox(temp_geom_load);
    Eigen::AlignedBox3f total_aabb;
    total_aabb.extend(aabb);

    const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(temp_geom_load);

    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        default_prog.ClearShaders();
        default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        default_prog.Link();
    };
    LoadProgram();

    //Define the pangolin interactive window registeration keys
    definePangolinRegKeys();

    //align the UAV's camera with the floor.
    alignCameraWithFloor();

    double timestamp = 0.0; // Initialize timestamp

    do {
        ready = true;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        printCurrentSCam();

        if (d_cam.IsShown()) {

            d_cam.Activate();

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }

            default_prog.Bind();
            default_prog.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
            pangolin::GlDraw(default_prog, geomToRender, nullptr);
            //adding 3d-axis of the drone perspective and the object perspective.
            // pangolin::glDrawAxis(5.0);

            pangolin::glSetFrameOfReference(s_cam.GetProjectionMatrix());
            pangolin::glDrawAxis(5.0);
            pangolin::glUnsetFrameOfReference();
            default_prog.Unbind();

            img = CaptureFrame(d_cam);

            this->lastImageCap = img.clone();

            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
            timestamp = value.count() / 1000.0;

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

        }
        if(track){
            this->simulatorLock.lock();
            auto empty_slam = SlamCurrentView(img, timestamp);
//            auto empty_slam = rotateAndSlam(img, down_up_flag,timestamp,i,0.3,aggregator);
            this->simulatorLock.unlock();

            if (empty_slam == true){
                pangolin::FinishFrame();
                continue;
            }
        }
        if(foundExitPoints){
            //vector<double>{4, -0.17, -8}
            renderPoints(true);
        }
        if(addPointsToRender){
            // Retrieve the camera's current position (x, y, z)
            pangolin::OpenGlMatrix mv = this->s_cam.GetModelViewMatrix();
            Eigen::Matrix4d mvMatrix = mv;

            // Extract the translation part (camera position)
            Eigen::Vector3d cameraPosition = mvMatrix.block<3, 1>(0, 3);

            // Print the camera position
            std::cout << "Camera Position: x = " << cameraPosition[0] << ", y = " << cameraPosition[1] << ", z = " << cameraPosition[2] << std::endl;

            addDroneLocation({cameraPosition[0], 0, cameraPosition[2]});
        }
        if(renderDronePathPoints){
            this->renderDroneLocations();
        }
        timestamp += 0.1;
        pangolin::FinishFrame();
    } while (!pangolin::ShouldQuit() && !stopFlag);

    if (isSaveMap) {
        saveMap("final");
        SLAM->SaveMap(simulatorOutputDir + "/finalSimulatorCloudPoint.bin");
        std::cout << "new map saved to " << simulatorOutputDir + "/finalSimulatorCloudPoint.bin" << std::endl;
    }
    SLAM->Shutdown();
}

//this is the main function that activates the simulator
std::thread PolySimulator::run() {
    std::thread t(&PolySimulator::RunPolyModel, this);
    return t;
}
