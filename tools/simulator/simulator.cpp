//
// Created by tzuk on 6/4/23.
//

#include "simulator.h"

cv::Mat Simulator::getCurrentLocation() {
    locationLock.lock();      //JFM   lock and return indpendt mat
    cv::Mat locationCopy = Tcw.clone();      //return the current location (this func lock our location w)
    locationLock.unlock();
    return locationCopy;
}
//constructor
Simulator::Simulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo,
    bool trackImages,
    bool saveMap, std::string simulatorOutputDirPath, bool loadMap, std::string mapLoadPath,
    double movementFactor,
    
   
    std::string vocPath) : stopFlag(false), ready(false), saveMapSignal(false),
    track(false),
    movementFactor(movementFactor)   //speed of the robot(high movementFactor->high speed)
    , modelPath(model_path), modelTextureNameToAlignTo(modelTextureNameToAlignTo),
    isSaveMap(saveMap),
    trackImages(trackImages), cull_backfaces(false),
    viewportDesiredSize(640, 480) {
    // Open the ORBSLAMConfigFile for reading only .
    cv::FileStorage fSettings(ORBSLAMConfigFile, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];                 //put the values from the ORBSLAMConfigFile by using the read file
    float cy = fSettings["Camera.cy"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int nLevels = fSettings["ORBextractor.nLevels"];


    SLAM = std::make_shared<ORB_SLAM2::System>(vocPath, ORBSLAMConfigFile, ORB_SLAM2::System::MONOCULAR, true, trackImages,  //construct  a new ORB_SLAM2::System and put it in SLAM
        loadMap,
        mapLoadPath,
        true);
    //parmeters  ----
   /*vocPath: A string representing the path to the ORBSLAM2 vocabulary file. Defaults to .
   * ORBSLAMConfigFile: A string representing the path to the ORBSLAM2 configuration file .
   * RB_SLAM2::System::MONOCULAR:  indicates the operation mode of the ORB-SLAM2 system being used in the simulator.
   /* Parameters in SLAM constructor:
    * vocPath: A string representing the path to the ORBSLAM2 vocabulary file.
    *  true: Enables the relocalization  of ORB-SLAM2.
     * trackImages: should the simulator track images during the simulation ? .
     * loadMap:should the SLAM system need to load an existed map from mapLoadPath.
     *  mapLoadPath: The file path that we can load an existed map from it
     * true: Enables the viewer for visualizing the SLAM process in real-time.
    */






    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    //This line constructs a camera intrinsic matrix K using the parameters read from the configuration file 
    //(fx, fy, cx, and cy). The matrix is used for camera calibration in the ORB-SLAM2 system.

    orbExtractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);//build an instance of ORB_SLAM2::ORBextractor
    /*
    * nFeatures:the number of features in one image
    * fScaleFactor :The scale factor for the image pyramid. control the image size
    * nLevels This parameter sets the number of levels in the image pyramid(each level have diffrent scale).
    *fIniThFAST:
    * fMinThFAST
    */
    char time_buf[21];                               //char array 
    time_t now;
    std::time(&now);                              //get the current time by the system clock
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now));    //write the current time in simple way (put it in time_buf)
    std::string currentTime(time_buf);
    simulatorOutputDir = simulatorOutputDirPath + "/" + currentTime + "/";  //create path to  file that save the current time
    std::filesystem::create_directory(simulatorOutputDir);

}
//command:refrence to string represnt  what the robot should do e.g ...   
void Simulator::command(std::string& command, int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
    std::istringstream iss(command);                  //this make us can treat the  string like a stream
    std::string c;
    double value;
    iss >> c;                                         //put the first word of iss in c
    if (commandMap.count(c) && commandMap[c]) {       //check if the specifc command is in the commandmap and if he  has an implemtion by cheching the boolean value of
                                                      //the command in the command map

        std::string stringValue;
        iss >> stringValue;                          //this three lines take the seconed section of the command and put it in value(its a number (e.g number of steps...) )
        value = std::stod(stringValue);


        applyCommand(c, value, intervalUsleep, fps, totalCommandTimeInSeconds);

    }
    else {
        std::cout << "the command " << c << " is not supported and will be skipped" << std::endl;   //if we didnt have the command or his emplmtion 
    }
}



void Simulator::simulatorRunThread() {
    pangolin::CreateWindowAndBind("Main", viewportDesiredSize[0], viewportDesiredSize[1]); //create a  graphical user interface
    /*
    * Main :the name of the interface
    * viewportDesiredSize[0]: width of the graphical user interface window
    *viewportDesiredSize[1]: height of the graphical user interface window
     */
    glEnable(GL_DEPTH_TEST);           // enables depth testing in the rendering pipeline


    //ProjectionMatrix: creates a "lens" for our virtual camera. It determines how the 3D scene will be projected onto the 2D screen.
    //   K(0, 0): Focal length along the x-axis. Higher value = zoomed-in view, lower value = wider field of view.
    //   K(1, 1) : Focal length along the y - axis.Affects vertical zoom level.
    //  K(0, 2) : X - coordinate of principal point, where optical axis intersects image plane.Corrects optical distortions.
    //   K(1, 2) : Y - coordinate of principal point.Corrects optical distortions.
    //   0.1 : Near clipping plane distance.Closest visible distance from the camera.
    //   20 : Far clipping plane distance.Farthest visible distance from the camera

    /*
    pangolin::ModelViewLookAt: This function positions and orients the camera in the 3D world. It tells the camera where to be .
        0.1: Initial x-coordinate of the camera position. This sets the camera's horizontal position in the scene.
       -0.1: Initial y - coordinate of the camera position.This sets the camera's vertical position in the scene.
        0.3 : Initial z - coordinate of the camera position.This sets the camera's depth position in the scene.
        0 : Initial x - coordinate of the camera's target point. This sets the point the camera is looking at along the x-axis.
        0 : Initial y - coordinate of the camera's target point. This sets the point the camera is looking at along the y-axis.
        0 : Initial z - coordinate of the camera's target point. This sets the point the camera is looking at along the z-axis.
        0.0 : Initial x - coordinate of the camera's up vector. This sets the camera's up direction along the x - axis.
        - 1.0 : Initial y - coordinate of the camera's up vector. This sets the camera's up direction along the y - axis.
        pangolin::AxisY : Initial z - coordinate of the camera's up vector. This sets the camera's up direction along the z - axis, specifically aligned with the y - axis of the camera's local coordinate system. This means that the camera's "top" points towards the positive y - direction.
    */

    // pangolin::OpenGlRenderState: state" for our virtual camera. It stores all the information ,   lens properties and the camera's position and orientation.

    s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1), K(0, 2),
            K(1, 2), 0.1, 20),
        pangolin::ModelViewLookAt(0.1, -0.1, 0.3, 0, 0, 0, 0.0, -1.0,//
            pangolin::AxisY)); // the first 3 value are meaningless because we change them later

    bool show_bounds = false; //dont show   boundaries of the rendered objects in the 3D scene
    bool show_axis = false; // dont show axis
    bool show_x0 = false;  //dont show show the x0-axis
    bool show_y0 = false; //dont show the y0-axis
    bool show_z0 = false; //dont show the z0-axis
    //these letters is a butons in the inter face so 
    pangolin::RegisterKeyPressCallback('b', [&]() { show_bounds = !show_bounds; });  // if the user press b->'b': Toggle showing bounds of the rendered objects in the 3D scene.
    pangolin::RegisterKeyPressCallback('0', [&]() { cull_backfaces = !cull_backfaces; });//if ->Toggle backface culling, which controls whether to hide objects when viewed from the backside.
    pangolin::RegisterKeyPressCallback('a', [&]() { show_axis = !show_axis; }); //Toggle showing the X, Y, and Z axes in the 3D scene.
    pangolin::RegisterKeyPressCallback('k', [&]() { stopFlag = !stopFlag; });// Toggle stopping or resuming the main loop in the simulation.
    pangolin::RegisterKeyPressCallback('t', [&]() { track = !track; }); //Toggle SLAM tracking (activating or deactivating the tracking process).
    pangolin::RegisterKeyPressCallback('m', [&]() { saveMapSignal = !saveMapSignal; }); //Toggle signaling to save the map and cloud point data.
    pangolin::RegisterKeyPressCallback('x', [&]() { show_x0 = !show_x0; }); // Toggle showing the X-axis of the local coordinate system for the virtual camera's position
    pangolin::RegisterKeyPressCallback('y', [&]() { show_y0 = !show_y0; }); //Toggle showing the Y-axis of the local coordinate system for the virtual camera's position
    pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; }); //Toggle showing the Z-axis of the local coordinate system for the virtual camera's position.
    pangolin::RegisterKeyPressCallback('w', [&]() { applyForwardToModelCam(s_cam, movementFactor); }); //Apply forward movement to the model's camera.
    pangolin::RegisterKeyPressCallback('a', [&]() { applyRightToModelCam(s_cam, movementFactor); }); //Apply backward movement to the model's camera.
    pangolin::RegisterKeyPressCallback('s', [&]() { applyForwardToModelCam(s_cam, -movementFactor); }); //Apply rightward movement to the model's camera.
    pangolin::RegisterKeyPressCallback('d', [&]() { applyRightToModelCam(s_cam, -movementFactor); });// Apply leftward movement to the model's camera.
    pangolin::RegisterKeyPressCallback('e', [&]() { applyYawRotationToModelCam(s_cam, 1); });// Apply yaw rotation to the model's camera (rotate horizontally).
    pangolin::RegisterKeyPressCallback('q', [&]() { applyYawRotationToModelCam(s_cam, -1); });//Apply yaw rotation to the model's camera in the opposite direction.
    pangolin::RegisterKeyPressCallback('r', [&]() {// Apply upward movement to the model's camera (move vertically).
        applyUpModelCam(s_cam, -movementFactor);
        });// ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]() { applyUpModelCam(s_cam, movementFactor); }); //Apply downward movement to the model's camera (move vertically).


    const pangolin::Geometry modelGeometry = pangolin::LoadGeometry(modelPath);//load the 3D  model from the file specified by modelPath and put it in modelGeometry.
    alignModelViewPointToSurface(modelGeometry, modelTextureNameToAlignTo); //  align the view point of the model to a specific surface.

    geomToRender = pangolin::ToGlGeometry(modelGeometry);//Convert the pangolin::Geometry object modelGeometry into a format suitable for rendering using OpenGL.and put it in geomToRender

    for (auto& buffer : geomToRender.buffers) {
        .
            buffer.second.attributes.erase("normal"); //remove the attribute named "normal" from the all the  buffers in geomToRender.

    }
    cv::Mat img;

    auto LoadProgram = [&]() {  //Declare a lambda function named LoadProgram.
        program.ClearShaders(); //It clears all the previous attached shaders from the program, preparing it for new shader attachments.

       //  add a shader to the shader program.
       //  pangolin::GlSlAnnotatedShader is the type of the shader
       //  pangolin::shader is the actual shader code or file containing the shader.
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);

        // link the shaders attached to the program into a complete shader program .
        program.Link();
    };

    LoadProgram();//  call the LoadProgram lambda function that was declared .

    pangolin::Handler3D handler(s_cam);   // creates a Handler3D object named handler.

    pangolin::View& d_cam = pangolin::CreateDisplay()   // create a new display window.

        .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)-viewportDesiredSize[0] / (float)viewportDesiredSize[1]))
        .SetHandler(&handler);

    int numberOfFramesForOrbslam = 0;

    while (!pangolin::ShouldQuit() && !stopFlag) { // continuously render frames and update the 3D scene until the user decides to quit (by closing the display window) or sets the stopFlag variable to  true
        ready = true;

        //T clears the color and depth buffers of the rendering pipeline.
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        if (d_cam.IsShown()) { // check if  d_cam is currently shown on the screen.

            d_cam.Activate(); // activates the display window d_cam


            if (cull_backfaces) { //  check the bool cull_backfaces to determine whether to enable backface culling or not.
                glEnable(GL_CULL_FACE); // enables backface culling in the OpenGL rendering pipeline.
                glCullFace(GL_BACK);//This line sets the culling mode to cull back faces of polygons 
            }
            program.Bind(); // binds the shader program  to the current OpenGL rendering context.


            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix()); // sets a  variable  "KT_cw" in the shader  program.

            pangolin::GlDraw(program, geomToRender, nullptr); //render the geometry contained in geomToRender.
            program.Unbind(); // unbinds the currently bound shader from the OpenGL .

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size); // put the  current OpenGL viewport parameters in the viewport_size array.

            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");//RGBA32" specifies that the image format is Red-Green-Blue-Alpha with 32 bits per pixel.
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8); //allocate memory to the buffer and put the image data.


            // sets the buffer from which pixels will be read during the next read operation.
            glReadBuffer(GL_BACK);

            //sets the pixel storage mode for subsequent glReadPixels calls.
            glPixelStorei(GL_PACK_ALIGNMENT, 1);

            //read pixel data from the current back buffer and stores it in the buffer object.
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

            // create an OpenCV cv::Mat object named imgBuffer.
            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            // convert the image imgBuffer from RGBA format to grayscale format.
            cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY);

            // convert the pixel data type of the image img from 8-bit unsigned integers with 4 channels to 8-bit unsigned integers with 1 channel .
            img.convertTo(img, CV_8UC1);

            // flips the image img vertically 
            cv::flip(img, img, 0);


            // captures the current time using the std::chrono::system_clock ,and put  it in " now".
            auto now = std::chrono::system_clock::now();// // clock that represents the current time of the system.


            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);// converts the now time point to a time point with a precision of milliseconds and put the result  in  now_ms.
            auto value = now_ms.time_since_epoch(); //  calculate the duration between the now_ms time point and the epoch time point  and put the result  at value.

            double timestamp = value.count() / 1000.0; // converts the duration value to a floating-point number timestamp, representing the number of seconds that have elapsed since the epoch.

            if (saveMapSignal) {
                saveMapSignal = false;
                char time_buf[21];
                time_t now_t;
                std::time(&now_t);  // gets the current time from the system and put it in the now_t .
                std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now_t)); // format the current time stored in now_t
                std::string currentTime(time_buf);
                saveMap(currentTime); //The function saveMap  saves the current state of the map with the provided timestamp.

                SLAM->SaveMap(simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin"); //This line saves the map associated with the current timestamp.


                // print  message to the console, indicating that a new map has been saved and specifying the file path where the map is saved.
                std::cout << "new map saved to " << simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin"
                    << std::endl;
            }

            if (track) {
                locationLock.lock(); //preventing other threads from accessing shared location data concurrently
                if (trackImages) {
                    Tcw = SLAM->TrackMonocular(img, timestamp); // the SLAM system will  (img) to track the camera motion.
                }
                else {
                    std::vector<cv::KeyPoint> pts;
                    cv::Mat mDescriptors;
                    orbExtractor->operator()(img, cv::Mat(), pts, mDescriptors);   // the SLAM system will first extract keypoints and descriptors from the input image (img) using the orbExtractor, and then it will use those keypoints and descriptors to track the camera motion.
                    Tcw = SLAM->TrackMonocular(mDescriptors, pts, timestamp);
                }

                locationLock.unlock();  //unlock the mutex
            }

            s_cam.Apply(); // apply the OpenGL camera state  to the rendering context. 

            glDisable(GL_CULL_FACE); // disable the face culling functionality in the OpenGL rendering pipeline. 

            drawPoints(seenPoints, keypoint_points); //draw points in the 3D space
        }

        pangolin::FinishFrame();  //finishes the current frame
    }
    if (isSaveMap) {

        saveMap("final");
        SLAM->SaveMap(simulatorOutputDir + "/finalSimulatorCloudPoint.bin"); //save the map 
        std::cout << "new map saved to " << simulatorOutputDir + "/finalSimulatorCloudPoint.bin" << std::endl;

    }
    SLAM->Shutdown();  //Shutdown  the SLAM system. 
}
//starting the simulator in a separate thread
std::thread Simulator::run() {
    std::thread thread(&Simulator::simulatorRunThread, this);  // creates a new std::thread object named thread.

    return thread;
}

void Simulator::saveMap(std::string prefix) {
    std::ofstream pointData;  //declare output stram class

    pointData.open(simulatorOutputDir + "/cloud" + prefix + ".csv");      //open file for writing

    //simulatorOutputDir->containing the directory path where the file should be created. 
    //to indicate that this file related to cloud(collection of 3D points in space)  (section from the file name)
    //perfix to the file name
    //file kind(Comma-Separated Values)
    for (auto& p : SLAM->GetMap()->GetAllMapPoints()) {   //loop run at all the map points in the SLAM system
        if (p != nullptr && !p->isBad()) { //check if the point good
            auto point = p->GetWorldPos();  //retrieves the world position of a map
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point);
            // convert the 3D world position point to an Eigen matrix.

            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);   //crate a openCV matrix intailezd with 0
            worldPos.at<double>(0) = vector.x(); //put the x of the world position in the first element of the opencv matrix.
            worldPos.at<double>(1) = vector.y(); //put the y  of the world position in the first element of the opencv matrix.
            worldPos.at<double>(2) = vector.z(); //put the z of the world position in the first element of the opencv matrix.
            p->UpdateNormalAndDepth();  //update the depth and the normal vecot of the map
            cv::Mat Pn = p->GetNormal();   // put normal vector of the map point in Pn.
            Pn.convertTo(Pn, CV_64F);      // Convert the normal vector to a double-precision floating-point OpenCV matrix.
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);  // write the x, y,z  of the map point world position to the pointData output stream in CSV format.
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","  //  writing the minimum and maximum distance invariance  of the map point, aand x, y, z of its normal vector.
                << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);
            std::map<ORB_SLAM2::KeyFrame*, size_t> observations = p->GetObservations();  // retrieves a map containing the observations of the map point p.
            for (auto obs : observations) {   //loop iterates at all  observation of the map point p
                ORB_SLAM2::KeyFrame* currentFrame = obs.first; //returnes the pointer to the camera frame that observes the map point p .
                if (!currentFrame->image.empty()) {   //check if the current frame has an image
                    size_t pointIndex = obs.second;    //return the index of the map point
                    cv::KeyPoint keyPoint = currentFrame->mvKeysUn[pointIndex];  // Get the keypoint
                    cv::Point2f featurePoint = keyPoint.pt;    //return the 2D  coordinates of the keypoint.
                    pointData << "," << currentFrame->mnId << "," << featurePoint.x << "," << featurePoint.y;  // Write the data to the file: keyframe ID, x, y.

                }
            }
            pointData << std::endl;  //make end to the line
        }
    }
    pointData.close();   //close the file

}
// extract surface information from a given 3D model geometry. 
//parmeters:
// modelGeometry: constant reference to a pangolin::Geometry object (the input 3D model geometry)
// modelTextureNameToAlignTo: which is a std::string representing the name of the texture to align the surface to
// surface :  reference to an Eigen::MatrixXf (a 2D matrix of floats) that will store the extracted surface information.

void Simulator::extractSurface(const pangolin::Geometry& modelGeometry, std::string modelTextureNameToAlignTo,
    Eigen::MatrixXf& surface) {
    std::vector<Eigen::Vector3<unsigned int>> surfaceIndices; // declares a vector surfaceIndices that  will store the indices of the vertices that make up the surface.

    for (auto& o : modelGeometry.objects) { //iterates over the  3D model in the modelGeometry.
        if (o.first == modelTextureNameToAlignTo) { // checks if the name of the texture associated with the current 3D model
            const auto& it_vert = o.second.attributes.find("vertex_indices"); // find the attribute named "vertex_indices" in the current object 
            if (it_vert != o.second.attributes.end()) {// checks if the "vertex_indices" attribute was found in the current  object
                const auto& vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);// retrieve the attribute data associated with the "vertex_indices" attribute and put it in the  vs. 
                for (size_t i = 0; i < vs.h; ++i) {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i)); //This line maps the vertex indices of the current triangle to an Eigen Vector

                    surfaceIndices.emplace_back(v); //This adds  v, representing the vertex indices of the current triangle, to the surfaceIndices vector
                }
            }
        }
    }
    surface = Eigen::MatrixXf(surfaceIndices.size() * 3, 3); //This line creates surface, which will store the extracted surface information. 
    int currentIndex = 0;
    for (const auto& b : modelGeometry.buffers) {  //loop over all the buffers of the modelGeometry.
        const auto& it_vert = b.second.attributes.find("vertex"); //finds the attribute named "vertex" in the current buffer  and stores it in it_vert.
        if (it_vert != b.second.attributes.end()) {// check if the "vertex" attribute was found in the current buffer. If it was, it means the buffer contains vertex positions.

            const auto& vs = std::get<pangolin::Image<float>>(it_vert->second); //etrieves the attribute data associated with the "vertex" attribute and stores it in a vs
            for (auto& row : surfaceIndices) {
                for (auto& i : row) {
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i)); //This line maps the 3D position (x, y, z) of the vertex with index i in the vs attribute data to  v.


                    surface.row(currentIndex++) = v;
                }
            }
        }
    }
}




//apply a pitch rotation to the camera view in the 3D model visualization.
//parms
//pangolin::OpenGlRenderState &cam: A reference to the camera's OpenGL render state, which contains the current model view matrix that represents the camera's position and orientation.
// value: The amount of pitch rotation to be applied, specified in degrees.
void Simulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState& cam, double value) {
    double rand = double(value) * (M_PI / 180); //Convert the value from degrees to radians 
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R; // Create a 3x3 rotation matrix R to represent the pitch rotation.
    R << 1, 0, 0,
        0, c, -s,
        0, s, c;

    //Create a 4x4 identity matrix pangolinR, which will be used to store the pitch rotation matrix in 4x4 form.
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();;
    pangolinR.block<3, 3>(0, 0) = R; //Assign the 3x3 rotation matrix R to the top-left 3x3 block of pangolinR.

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());//Extract the current model view matrix of the camera  and convert it to an Eigen matrix for manipulation.

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }
    //Set the updated model view matrix  back to the camera's OpenGL render state . 
    cam.SetModelViewMatrix(newModelView);
}
//func:  reference to a function that accepts an pangolin::OpenGlRenderState objectand and double  represents the command that excuted to the camera.

//value :  representing the  amount of change that will happen  by the command.

//intervalUsleep :representing the time  between each step of the command. controls how fast the command is executed.

//fps : representing the frames per second(FPS)

//totalCommandTimeInSeconds :  representing the time for  the command should be executed.
//
void Simulator::intervalOverCommand(
    const std::function<void(pangolin::OpenGlRenderState&, double&)>& func, double value,
    int intervalUsleep, double fps, int totalCommandTimeInSeconds) {

    double intervalValue = value / (fps * totalCommandTimeInSeconds);    // Calculate the intervalValue to apply to the command in every step
    int intervalIndex = 0;
    while (intervalIndex <= fps * totalCommandTimeInSeconds) {    //  runs 'fps * totalCommandTimeInSeconds' times representing total number of steps for the command

        usleep(intervalUsleep);   //stop   peroid bdfore excute the next step
        func(s_cam, intervalValue); // change the camera state based on the intervalValue.
        intervalIndex += 1;
    }
}
//parmetrs: command(witch command we want to excute)  |  value(number of steps (e.g number of steps to go back)  |  intervalUsleep(the time that the 
//robot need to rest between two opreations  |   fps(updat the image every  fps time)  |    totalCommandTimeInSeconds(some command work for alimited peroid (this parmeter
//is the limited peroid)))
//The function checks the value of the command parameter to identify which type of camera movement should be applied.

//Depending on the command, the function calls the appropriate intervalOverCommand function with the corresponding camera manipulation functionand the provided command 
//parameters.The intervalOverCommand function is used to gradually apply the command in smaller increments over a specified time duration.
void Simulator::applyCommand(std::string& command, double value, int intervalUsleep, double fps,
    int totalCommandTimeInSeconds) {

    if (command == "cw") {        //clock wise 
        intervalOverCommand(Simulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
            totalCommandTimeInSeconds);
    }
    else if (command == "ccw") {  //Counterclockwise
        intervalOverCommand(Simulator::applyYawRotationToModelCam, -1 * value,
            intervalUsleep, fps,
            totalCommandTimeInSeconds);
    }
    else if (command == "forward") {
        intervalOverCommand(Simulator::applyForwardToModelCam, value, intervalUsleep, fps,    //The function checks the value of the command parameter to identify which type of camera movement should be applied.

    }
    else if (command == "back") {
        intervalOverCommand(Simulator::applyForwardToModelCam, -1 * value, intervalUsleep,
            fps, totalCommandTimeInSeconds);
    }
    else if (command == "right") {
        intervalOverCommand(Simulator::applyRightToModelCam, -1 * value, intervalUsleep,
            fps, totalCommandTimeInSeconds);
    }
    else if (command == "left") {
        intervalOverCommand(Simulator::applyRightToModelCam, value, intervalUsleep, fps,
            totalCommandTimeInSeconds);
    }
    else if (command == "up") {
        intervalOverCommand(Simulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
            totalCommandTimeInSeconds);
    }
    else if (command == "down") {
        intervalOverCommand(Simulator::applyUpModelCam, value, intervalUsleep, fps,
            totalCommandTimeInSeconds);
    }
}
//  applyUpModelCam function adjusts the camera's position in the 3D environment by moving it upward by a specified distance value. 
// 
//cam:  reference to the camera's OpenGL render state
//value:The amount of upward translation to apply to the camera's position. 
void Simulator::applyUpModelCam(pangolin::OpenGlRenderState& cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(1, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

//applyForwardToModelCam function adjusts the camera's position in the 3D environment by moving it forward along its viewing direction by a specified distance value. 
//parameters:
//pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
// value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.
void Simulator::applyForwardToModelCam(pangolin::OpenGlRenderState& cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}
//adjusts the camera's position in the 3D environment by moving it rightward along its right direction by a specified distance value. 
//parms:
//pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
// value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.

//
void Simulator::applyRightToModelCam(pangolin::OpenGlRenderState& cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(0, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}


//parms:
//pangolin::OpenGlRenderState& cam : A reference to the camera's OpenGL render state, representing the current position and orientation of the camera in the 3D environment.
// value : The amount of forward translation to apply to the camera's position. It represents the distance the camera should move along its forward direction.


//adjusts the camera's orientation in the 3D environment by applying a yaw rotation around its vertical axis by a specified angle value. 
void Simulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState& cam, double value) {
    double rand = double(value) * (M_PI / 180); //    // Convert the rotation angle from degrees to radians

    double c = std::cos(rand);
    double s = std::sin(rand);
    //    // Convert the rotation angle from degrees to radians
    Eigen::Matrix3d R;
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;

    //    // Create a 4x4 transformation matrix pangolinR and set its top-left 3x3 block to the yaw rotation matrix R
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;    // Get the current model view matrix of the camera

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

void Simulator::alignModelViewPointToSurface(const pangolin::Geometry& modelGeometry,
    std::string modelTextureNameToAlignTo) {
    Eigen::MatrixXf surface;//: This line declares a 2D Eigen matrix surface, which will be used to store the extracted surface data from the model.
    extractSurface(modelGeometry, modelTextureNameToAlignTo, surface);//This line calls the extractSurface function to extract the surface vertices of the model with the given texture name (modelTextureNameToAlignTo) and stores the result in the surface matrix.
   //This line computes the Singular Value Decomposition (SVD) of the surface matrix. 
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(surface, Eigen::ComputeThinU | Eigen::ComputeThinV);

    svd.computeV();//This line computes the right singular vectors (V) of the SVD. These vectors represent the orientation of the surface in 3D space.

    Eigen::Vector3f v = svd.matrixV().col(2);//This line extracts the third column of the matrix V

    // This line constructs a new model-view matrix (mvm) using the normal vector v as the camera's look-at direction. The camera position is set at the origin (0, 0, 0), and the camera's up direction is along the negative Y-axis (pangolin::AxisY).
    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,//
        -1.0,
        pangolin::AxisY);
    const auto proj = pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1),
        K(0, 2), K(1, 2), 0.1, 20);
    s_cam.SetModelViewMatrix(mvm);
    s_cam.SetProjectionMatrix(proj);
    applyPitchRotationToModelCam(s_cam, -90);
}