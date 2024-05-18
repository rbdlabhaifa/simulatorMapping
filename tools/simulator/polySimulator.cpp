//
// Created by Dean on 9/30/23.
//

#include "polySimulator.h"
#include "navigation/ExitRoomFiles/exit_room_algo.cpp"

PolySimulator::PolySimulator(bool _loadCustomMap){
    // loading configuration file content
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    this->ORBSLAMConfigFile = data["DroneYamlPathSlam"];
    this->vocPath = data["VocabularyPath"];
    this->modelPath = data["modelPath"];
    this->trackImages = data["trackImages"];
    this->movementFactor = data["movementFactor"];
    this->speedFactor = data["simulatorStartingSpeed"];
    this->simulatorOutputDirPath = data["simulatorOutputDir"];

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
//    orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

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

bool PolySimulator::isSetInPlace() const{
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

//This method align the current drone position with the axis.
void PolySimulator::alignCameraWithFloor(bool withAnimation, PolySimulator::axis axis){
    int countZeros = 0;

    locationLock.lock();
    auto currentModelViewMat = this->s_cam.GetModelViewMatrix().Inverse();
    auto currentModelViewMatNotINV = this->s_cam.GetModelViewMatrix();
    locationLock.unlock();

    pangolin::OpenGlMatrix destView, stepMat;

    switch(axis){
        case PolySimulator::posX: {
            destView = pangolin::ModelViewLookAt(currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3), currentModelViewMat(0,3) + 1, currentModelViewMat(1,3), currentModelViewMat(2,3), 0, 1, 0);
        } break;
        case PolySimulator::negX: {
            destView = pangolin::ModelViewLookAt(currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3), currentModelViewMat(0,3) - 1, currentModelViewMat(1,3), currentModelViewMat(2,3), 0, 1, 0);
        } break;
        case PolySimulator::posZ:{
            destView = pangolin::ModelViewLookAt(currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3), currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3) + 1, 0, 1, 0);
        } break;
        case PolySimulator::negZ:{
            destView = pangolin::ModelViewLookAt(currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3), currentModelViewMat(0,3), currentModelViewMat(1,3), currentModelViewMat(2,3) - 1, 0, 1, 0);
        } break;
        default: {
            std::cout << "Wrong command, returned without doing nothing" << std::endl;
            return;
        }
    }

//    currentModelViewMat = currentModelViewMat.Inverse();

    for(int i = 0 ; i < 4; i++){
        for(int j = 0 ; j < 4 ; j++){
            stepMat(i,j) = (destView(i,j) - currentModelViewMatNotINV(i,j)) / ALIGN_INTERPOLATION_STEP;
            if(stepMat(i, j) == 0){
                countZeros++;
            }
        }
    }

    if(countZeros == 16){
        std::cout << "No action needed because the Drone is already in that direction\n" << std::endl;
        return;
    }

    for (int k = 0; k < ALIGN_INTERPOLATION_STEP; k++){
        for(int i = 0 ; i < 4; i++){
            for(int j = 0 ; j < 4 ; j++){
                currentModelViewMatNotINV(i,j) += stepMat(i,j);
            }
        }
        this->locationLock.lock();
        this->s_cam.SetModelViewMatrix(currentModelViewMatNotINV);
        this->locationLock.unlock();
        if(withAnimation){
            usleep(50);
        }
    }
    this->s_cam.SetModelViewMatrix(destView);

//    std::cout << stepMat << std::endl;
//    std::cout << currentModelViewMatNotINV << std::endl;
//    std::cout << destView << std::endl;
}

//this camera rotating the drone while slamming.
[[maybe_unused]] bool PolySimulator::rotateAndSlam(bool &down_up_flag, double& timestamp, double step){
    cv::Mat val = this->SLAM->TrackMonocular(*this->getCurrentViewPointerBW(), timestamp);
    static int i = 0;
//    int started_tracking_already = this->getSlamState();
    int started_tracking_already = this->getSlamState();
//    std::cout << started_tracking_already << std::endl;
    if (down_up_flag) {
        PolySimulator::applyUpModelCam(this->s_cam, -0.05);
        if (i == 10) {
            i = 0;
            down_up_flag = !down_up_flag;
        }
    } else {
        PolySimulator::applyUpModelCam(this->s_cam, 0.05);
        if (i == 10) {
            i = 0;
            down_up_flag = !down_up_flag;
        }
    }
    //if ORB-SLAM2 system is Slamming.
    if (started_tracking_already == 2) {
        PolySimulator::applyYawRotationToModelCam(this->s_cam, -step);
    }
    //if ORB-SLAM2 system is not 'OK' and created a map already
    if (started_tracking_already != 2 && started_tracking_already != 1){
        PolySimulator::applyYawRotationToModelCam(this->s_cam,  10 * step);
        i++;
        return false;
    }
    if (started_tracking_already == 1) { //in case ORB-SLAM2 didn't manage to create a map
        PolySimulator::applyYawRotationToModelCam(this->s_cam, step);
        i++;
        return false;
    }
    i++;
    return true;
}

//This method printing the current x,y,z location of the 'drone'.
void PolySimulator::printCurrentLocation(){
    this->locationLock.lock();
    const auto T_world_camera = this->s_cam.GetModelViewMatrix().Inverse();
    this->locationLock.unlock();
    const double x = T_world_camera(0,3);
    const double y = T_world_camera(1,3);
    const double z = T_world_camera(2,3);

    std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
}

//This method return the current x,y,z location of the 'drone'.
std::vector<double> PolySimulator::getDroneLocation(){
    std::vector<double> position;
    this->locationLock.lock();
    const auto T_world_camera = this->s_cam.GetModelViewMatrix().Inverse();
    this->locationLock.unlock();
    const double x = T_world_camera(0,3); //4
    const double y = T_world_camera(1,3); //-0.17
    const double z = T_world_camera(2,3); //-8

    position.push_back(x);
    position.push_back(y);
    position.push_back(z);

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
void PolySimulator::lookAtPoint(vector<double> whereToLook) {
    pangolin::OpenGlMatrix stepMat;
    this->locationLock.lock();
    //get the current camera view
    pangolin::OpenGlMatrix currentView = this->s_cam.GetModelViewMatrix().Inverse();
    this->locationLock.unlock();
    //get the camera view after poiting it to the desired location
    pangolin::OpenGlMatrix DestView = pangolin::ModelViewLookAt(currentView(0,3), currentView(1,3), currentView(2,3), whereToLook[0], whereToLook[2], whereToLook[1], 0, 1, 0);

    currentView = currentView.Inverse();

    for(int i = 0 ; i < 4; i++){
        for(int j = 0 ; j < 4 ; j++){
            stepMat(i,j) = (DestView(i,j) - currentView(i,j)) / LOOK_AT_INTERPOLATE_STEP;
        }
    }
    //interpolate the values
    for (int k = 0; k < LOOK_AT_INTERPOLATE_STEP; k++){
        for(int i = 0 ; i < 4; i++){
            for(int j = 0 ; j < 4 ; j++){
                currentView(i,j) += stepMat(i,j);
            }
        }
        this->locationLock.lock();
        this->s_cam.SetModelViewMatrix(currentView);
        this->locationLock.unlock();
        usleep(50);
    }
}

//This method initiazlizing the view camera - which is the drone 'eyes' 
void PolySimulator::setCamera(){
    this->s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), NEAR_PLANE, FAR_PLANE),
        pangolin::ModelViewLookAt(viewpointX, viewpointY, viewpointZ, 0, 0, 0, 0.0, -1.0, pangolin::AxisY)); // the first 3 value are meaningless because we change them later
}

//function create a pangolin::View instance - D_CAM according to the screen size choosen - aspect ratio.
pangolin::View& PolySimulator::setDCAM(pangolin::Handler3D *handler){
    return pangolin::CreateDisplay().SetBounds(0.0, 1, 0, 1,  ((float) -viewportDesiredSize[0] / (float) viewportDesiredSize[1])).SetHandler(handler);
}

//This camera moving the drone to XYZ position.
void PolySimulator::moveToXYZ(vector<double> whereToMove){
    lookAtPoint(whereToMove);

    pangolin::OpenGlMatrix stepMat;

    this->locationLock.lock();
    //get the current camera view
    pangolin::OpenGlMatrix currentView = this->s_cam.GetModelViewMatrix().Inverse();
    this->locationLock.unlock();
    //get the camera view after poiting it to the desired location
    pangolin::OpenGlMatrix DestView = currentView;
    DestView(0,3) = whereToMove[0]; //x
    DestView(1,3) = whereToMove[2]; //y which is z
    DestView(2,3) = whereToMove[1]; // z which is y

    currentView = currentView.Inverse();
    DestView = DestView.Inverse();

    for(int i = 0 ; i < 4; i++){
        for(int j = 0 ; j < 4 ; j++){
            stepMat(i,j) = (DestView(i,j) - currentView(i,j)) / INTERPOLATE_STEP;
        }
    }
    //interpolate the values
    for (int k = 0; k < INTERPOLATE_STEP; k++){
        for(int i = 0 ; i < 4; i++){
            for(int j = 0 ; j < 4 ; j++){
                currentView(i,j) += stepMat(i,j);
            }
        }
        this->locationLock.lock();
        this->s_cam.SetModelViewMatrix(currentView);
        this->locationLock.unlock();
        usleep(50);
    }

    curr_cam_pose = {whereToMove[0],whereToMove[2], whereToMove[1]};
}

//Rendering the exit-room algorithm output points.
void PolySimulator::renderPoints(bool showDrone){
    if(showDrone){
        generateGLPoint(vector<GLfloat>{1.0f, 1.0f, 0.0f}, 20.0f, curr_cam_pose); //x,z,y
    }
    for(auto & i : pointsToRender){
        generateGLPoint(vector<GLfloat>{1.0f, 0.0f, 0.0f}, 20.0f, vector<double>{i[0], 0.5, i[1]});
        generateGLLine(vector<GLfloat>{1.0f, 0.0f, 0.0f}, curr_cam_pose, vector<double>{i[0], 0.5, i[1]});
    }
}

//Allows the user adding points which the drone visited in
void PolySimulator::addDroneLocation(const std::vector<double>& currentLocation){
    auto it = std::find(dronePath.begin(), dronePath.end(), currentLocation);

    if (it != dronePath.end()) {
//        std::cout << "Location already exist, not adding into array" << std::endl;
    } else {
        dronePath.push_back(currentLocation);
    }
}

//Rendering the drone locations saved by the user
void PolySimulator::renderDroneLocations(){
    glClear(GL_POINTS | GL_LINES);
    for(auto & i : dronePath){
        generateGLPoint(vector<GLfloat>{1.0f, 0.0f, 0.0f}, 20.0f, vector<double>{i[0], i[1], i[2]});
    }
//    for(int i = 0 ; i < dronePath.size()-1 ; i++){
//        generateGLPoint(vector<GLfloat>{1.0f, 0.0f, 0.0f}, 10.0f, vector<double>{dronePath[i][0], dronePath[i][1], dronePath[i][2]});
//        generateGLLine(vector<GLfloat>{1.0f, 0.0f, 0.0f}, dronePath[i], dronePath[i+1]);
//    }
}

//Returning the map points of the ORB-SLAM2 system only after slam finished loop closer.
std::vector<vector<double>> PolySimulator::getCurrentMapPoint(){
    std::vector<vector<double>> vectorOfDoublePoints;
    if(slamFinishedLoopCloser()){
        try{
            vectorOfDoublePoints = fromMapPointsToDoubleVec(this->SLAM->GetMap()->GetAllMapPoints());
        }
        catch(exception &e){
        std::cout << "Error occured while trying to get the current ORB-SLAM2's Map." << std::endl;
        }
    } else {
        std::cout << "Slam did not finished it's mapping process." << std::endl;
    }
    return vectorOfDoublePoints;
}

//Creating the drone view and saving it
void PolySimulator::CaptureFrame(pangolin::View& camera_view) {
    this->simulatorLock.lock();
    int viewport_size[4];
    glGetIntegerv(GL_VIEWPORT, viewport_size);

    pangolin::Image<unsigned char> buffer;
    //pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
    pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGBA32");
    buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);

    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

    this->currentImageColored = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
    cv::flip( this->currentImageColored,  this->currentImageColored, 0);

    cv::cvtColor(this->currentImageColored, this->currentImageBW, cv::COLOR_RGBA2GRAY);
    currentImageBW.convertTo(currentImageBW, CV_8UC1);

    this->simulatorLock.unlock();
}

//Get the current frame produced by the drone's cam.
const cv::Mat& PolySimulator::getCurrentViewFromUAV(){
    this->simulatorLock.lock();
    cv::Mat &refToImage = this->currentImageColored;
    this->simulatorLock.unlock();
    return refToImage;
}

//This function defines the Pangolin interactive window for the user.
//By choosing a 'char' value to attach the pangolin interactive window you can modify different actions.
void PolySimulator::definePangolinRegKeys(){

    pangolin::RegisterKeyPressCallback('9', [&]() { this->reflectViewBool(); });
    // pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });

    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 0.01); });
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -0.01); });
    pangolin::RegisterKeyPressCallback('z', [&]() { applyRollRotationToModelCam(s_cam, 0.01); });
    pangolin::RegisterKeyPressCallback('c', [&]() { applyRollRotationToModelCam(s_cam, -0.01); });
    pangolin::RegisterKeyPressCallback('t', [&]() { applyPitchRotationToModelCam(s_cam, 0.01); });
    pangolin::RegisterKeyPressCallback('y', [&]() { applyPitchRotationToModelCam(s_cam, -0.01); });
    pangolin::RegisterKeyPressCallback('r', [&]() { applyUpModelCam(s_cam, -movementFactor); });    // ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('1', [&]() { slower(); });
    pangolin::RegisterKeyPressCallback('2', [&]() { faster(); });
}

//This is the main function that rendering using 'Pangolin' the main view of the simulator.
void PolySimulator::RunPolyModel(){
    //Creating Pangolin window and loading the model.
    // Create Window for rendering
    pangolin::CreateWindowAndBind("PolySimulator", viewportDesiredSize[0], viewportDesiredSize[1]);
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

    geomToRender = pangolin::ToGlGeometry(temp_geom_load);

    pangolin::GlSlProgram default_prog;
    auto LoadProgram = [&]() {
        program.ClearShaders();
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        program.Link();
    };
    LoadProgram();

    //Define the pangolin interactive window registeration keys
    definePangolinRegKeys();

    //align the UAV's camera with the floor.
    alignCameraWithFloor(false, PolySimulator::posZ);

    double timestamp = 0.0; // Initialize timestamp


    do {
        ready = true;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (d_cam.IsShown()) {

            if (cull_backfaces) {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }

            program.Bind();
            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
            pangolin::GlDraw(program, geomToRender, nullptr);
            program.Unbind();

            CaptureFrame(d_cam);

            this->lastImageCap = this->currentImageBW.clone();

            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
//            timestamp = value.count() / 1000.0;

            if(track){
                this->simulatorLock.lock();
                this->Tcw = this->SLAM->TrackMonocular(this->currentImageBW.clone(), timestamp);
                this->simulatorLock.unlock();
                if (this->Tcw.empty() == 1){
                    pangolin::FinishFrame();
                    continue;
                }
            }

            s_cam.Apply();

            glDisable(GL_CULL_FACE);

            d_cam.Activate();
        }
        if(foundExitPoints){
            //vector<double>{4, -0.17, -8}
            renderPoints(true);
        }
        if(addPointsToRender){
            vector<double> currentDronePos = this->getDroneLocation();

            addDroneLocation({currentDronePos[0], currentDronePos[1], currentDronePos[2]});
        }
        if(renderDronePathPoints){
            this->renderDroneLocations();
        }
        timestamp += 0.1;
        pangolin::FinishFrame();
    } while (!pangolin::ShouldQuit() && !stopFlag);

    if (isSaveMap) {
        saveMap("final");
        this->SLAM->SaveMap(simulatorOutputDir + "/finalSimulatorCloudPoint.bin");
        std::cout << "new map saved to " << simulatorOutputDir + "/finalSimulatorCloudPoint.bin" << std::endl;
    }
    this->SLAM->Shutdown();
}

//this is the main function that activates the simulator
std::thread PolySimulator::run() {
    std::thread t(&PolySimulator::RunPolyModel, this);
    return t;
}