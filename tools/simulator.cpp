//
// Created by tzuk on 6/4/23.
//

#include "simulator.h"

cv::Mat Simulator::getCurrentLocation()
{
    locationLock.lock(); // lock the lock so no one can change on it while we are doing Tcw.clone()
    cv::Mat locationCopy = Tcw.clone(); // save acopy from the location 
    locationLock.unlock(); 
    return locationCopy; //return A 4x4 location matrix 
}

Simulator::Simulator(std::string ORBSLAMConfigFile, std::string model_path, std::string modelTextureNameToAlignTo,
                     bool trackImages,
                     bool saveMap, std::string simulatorOutputDirPath, bool loadMap, std::string mapLoadPath,
                     double movementFactor,
                     std::string vocPath) : stopFlag(false), ready(false), saveMapSignal(false),
                                            track(false),
                                            movementFactor(movementFactor), modelPath(model_path), modelTextureNameToAlignTo(modelTextureNameToAlignTo),
                                            isSaveMap(saveMap),
                                            trackImages(trackImages), cull_backfaces(false),
                                            viewportDesiredSize(640, 480)
{
    cv::FileStorage fSettings(ORBSLAMConfigFile, cv::FileStorage::READ);    // it creates an instance of the FileStorage class and opens the file specified by configPath and set it to read mode 

    // //reading from fSettings the camera parameters 
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    // read the parameters related to the ORB feature extractor
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    SLAM = std::make_shared<ORB_SLAM2::System>(vocPath, ORBSLAMConfigFile, ORB_SLAM2::System::MONOCULAR, false, trackImages,
                                               loadMap,
                                               mapLoadPath,
                                               true);       // creating a shared pointer to a new instance of the ORB_SLAM2::System
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;   // filling the elemts of the matrix (3x3)
    orbExtractor = std::make_shared<ORB_SLAM2::ORBextractor>(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);  // initializing the shared pointer by creating a new instance of the ORB_SLAM2::ORBextractor 
                                                                                                                           // class and setting it to point to this new created object
}

void Simulator::command(std::string &command, int intervalUsleep, double fps, int totalCommandTimeInSeconds)
{
    std::istringstream iss(command); // convert the command string to input stream 
    std::string c;
    double value;
    iss >> c; // extract the first token from the command and save it in c 
    if (commandMap.count(c) && commandMap[c])   //check if c is in the command map and if it's exuctable or no  
    {
        std::string stringValue;
        iss >> stringValue; //extract the next token from the command and save it in stringValue
        value = std::stod(stringValue); // convert the string to number 
        applyCommand(c, value, intervalUsleep, fps, totalCommandTimeInSeconds); // applying the command 
    }
    else
    {
        std::cout << "the command " << c << " is not supported and will be skipped" << std::endl;
    }
}
void Simulator::simulatorRunThread()
{
    pangolin::CreateWindowAndBind("Model");     // creating new window and bind it for rendering the 3D model

    // we manually need to restore the properties of the context
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

    // handling user interaction 
    pangolin::RegisterKeyPressCallback('b', [&]()
                                       { show_bounds = !show_bounds; }); // when the user press b the lambda function will be executed and if show_bounds was false it will set it to true ... 
    pangolin::RegisterKeyPressCallback('0', [&]()
                                       { cull_backfaces = !cull_backfaces; });
    pangolin::RegisterKeyPressCallback('a', [&]()
                                       { show_axis = !show_axis; });
    pangolin::RegisterKeyPressCallback('k', [&]()
                                       { stopFlag = !stopFlag; });
    pangolin::RegisterKeyPressCallback('t', [&]()
                                       { track = !track; });
    pangolin::RegisterKeyPressCallback('m', [&]()
                                       { saveMapSignal = !saveMapSignal; });
    pangolin::RegisterKeyPressCallback('x', [&]()
                                       { show_x0 = !show_x0; });
    pangolin::RegisterKeyPressCallback('y', [&]()
                                       { show_y0 = !show_y0; });
    pangolin::RegisterKeyPressCallback('z', [&]()
                                       { show_z0 = !show_z0; });
    pangolin::RegisterKeyPressCallback('w', [&]()
                                       { applyForwardToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('a', [&]()
                                       { applyRightToModelCam(s_cam, movementFactor); });
    pangolin::RegisterKeyPressCallback('s', [&]()
                                       { applyForwardToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('d', [&]()
                                       { applyRightToModelCam(s_cam, -movementFactor); });
    pangolin::RegisterKeyPressCallback('e', [&]()
                                       { applyYawRotationToModelCam(s_cam, 1); });
    pangolin::RegisterKeyPressCallback('q', [&]()
                                       { applyYawRotationToModelCam(s_cam, -1); });
    pangolin::RegisterKeyPressCallback('r', [&]()
                                       { applyUpModelCam(s_cam, -movementFactor); }); // ORBSLAM y axis is reversed
    pangolin::RegisterKeyPressCallback('f', [&]()
                                       { applyUpModelCam(s_cam, movementFactor); });

    const pangolin::Geometry modelGeometry = pangolin::LoadGeometry(modelPath); // load the 3D model Geometry
    alignModelViewPointToSurface(modelGeometry, modelTextureNameToAlignTo);
    geomToRender = pangolin::ToGlGeometry(modelGeometry);
    for (auto &buffer : geomToRender.buffers)
    {
        buffer.second.attributes.erase("normal");
    }

    // load and apply the GLSL shaders for rendering.
    auto LoadProgram = [&]()
    {
        program.ClearShaders();
        program.AddShader(pangolin::GlSlAnnotatedShader, pangolin::shader);
        program.Link();
    };
    LoadProgram();
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, ((float)-viewportDesiredSize[0] / (float)viewportDesiredSize[1]))
                                .SetHandler(&handler);
    int numberOfFramesForOrbslam = 0;

    while (!pangolin::ShouldQuit() && !stopFlag)    //keep till pangolin::ShouldQuit or stop signal is received.
    {
        cv::Mat img; //object which represents an image in OpenCV
        ready = true;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clearing the color buffer and the depth buffer

        if (d_cam.IsShown())
        {
            d_cam.Activate(); // perform rendering and tracking of the camera pose

            if (cull_backfaces) // check if cull_backfaces
            {
                //enable backface culling
                glEnable(GL_CULL_FACE); 
                glCullFace(GL_BACK);
            }
            

            program.Bind(); //blind the slam program
            program.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix()); // set the uniform for the camera projection matrix.
            pangolin::GlDraw(program, geomToRender, nullptr); // draw the 3d model 
            program.Unbind();   //ublind the slam program

            int viewport_size[4];
            glGetIntegerv(GL_VIEWPORT, viewport_size); //retrieving the dimensions of the current viewport

           // read the pixel buffer from OpenGL to retrieve the rendered image for processing.
            pangolin::Image<unsigned char> buffer;
            pangolin::VideoPixelFormat fmt = pangolin::VideoFormatFromString("RGBA32");
            buffer.Alloc(viewport_size[2], viewport_size[3], viewport_size[2] * fmt.bpp / 8);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, viewport_size[2], viewport_size[3], GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr);

            cv::Mat imgBuffer = cv::Mat(viewport_size[3], viewport_size[2], CV_8UC4, buffer.ptr);
            if(!imgBuffer.empty()){ //Check if imgBuffer is not empty
                cv::cvtColor(imgBuffer, img, cv::COLOR_RGBA2GRAY); //converts the image from RGBA to grayscale
            img.convertTo(img, CV_8UC1); //converts the image data to 8-bit unsigned integer
            cv::flip(img, img, 0); // flip the image vertically 
            }
            s_cam.Apply(); // Apply the camera model view matrix for navigation.

            glDisable(GL_CULL_FACE);   // Disable backface culling for subsequent rendering 
             pangolin::FinishFrame(); // and here we finish the current frame from displaying

             // Geting the current timestamp for the frame
            auto now = std::chrono::system_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto value = now_ms.time_since_epoch();
            double timestamp = value.count() / 1000.0;
            if (saveMapSignal) //check if a signal to save the map is received
            {
                saveMapSignal = false; //reset
                char time_buf[21]; //character array to hold the timestamp string
                time_t now_t; //hold the current time 
                std::time(&now_t); //store the cuurent time in now_t
                std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&now_t)); //this will format the timestamp as a string and store it in the time_buf array
                std::string currentTime(time_buf); //convert the timestamp string in the time_buf to currentTime
                saveMap(currentTime); //saving the map 
                SLAM->SaveMap(simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin");
                std::cout << "new map saved to " << simulatorOutputDir + "/simulatorCloudPoint" + currentTime + ".bin"
                          << std::endl;
            }
            if(!img.empty()){ // check if the valid frame image is available
                locationLock.lock(); // lock the lock 
                if (trackImages)    // check if tracking using images -> call the TrackMonocular function with the grayscale image
                {
                    Tcw = SLAM->TrackMonocular(img, timestamp);
                }
                else // else we are tracking using ORB descriptors -> extract keypoints and descriptors before tracking.
                {
                    std::vector<cv::KeyPoint> pts;
                    cv::Mat mDescriptors;
                    orbExtractor->operator()(img, cv::Mat(), pts, mDescriptors);
                    Tcw = SLAM->TrackMonocular(mDescriptors, pts, timestamp);
                }

                locationLock.unlock(); // release the lock 
            }
            


            //             drawPoints(seenPoints, keypoint_points);
        }else{  // the camera view is not shown -> just apply the camera view without rendering
            s_cam.Apply();
        pangolin::FinishFrame();

        }

    }
    if (isSaveMap) // save the final map if isSaveMap is set to true
    {

        saveMap("final");
        SLAM->SaveMap(simulatorOutputDir + "/finalSimulatorCloudPoint.bin");
        std::cout << "new map saved to " << simulatorOutputDir + "/finalSimulatorCloudPoint.bin" << std::endl;
    }
    SLAM->Shutdown(); // shut down the SLAM system 
}

std::thread Simulator::run() //Starts the 3D model viewer (pangolin), and wait for the user or code signal to start sending the view to the ORBSLAM2 object
{
    std::thread thread(&Simulator::simulatorRunThread, this); //make the simulatorRunThread function runs concurrently in this thread.
    return thread;
}

void Simulator::saveMap(std::string prefix)
{
    // opening an output file for saving the Map data in the csv
    std::ofstream pointData;
    pointData.open(simulatorOutputDir + "/cloud" + prefix + ".csv");

    for (auto &p : SLAM->GetMap()->GetAllMapPoints())   // interate over all SLAM map points 
    {
        if (p != nullptr && !p->isBad()) // check if it's valid point and not bad 
        {
            // Extract the 3D world position of the map point.
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> vector = ORB_SLAM2::Converter::toVector3d(point); //converting the input point to a 3D point represented as an Eigen vector.
            cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F); //creating new cv::Mat object with size of 3x1 filled with zeros
            //saving the values of the Eigen 3D vector to an OpenCV object (worldPos)
            worldPos.at<double>(0) = vector.x();
            worldPos.at<double>(1) = vector.y();
            worldPos.at<double>(2) = vector.z();

            p->UpdateNormalAndDepth(); // Update Normal And Depth for the point 
            cv::Mat Pn = p->GetNormal(); // store the normal vector of map point 
            Pn.convertTo(Pn, CV_64F); //converting the data type of Pn to double .

            // Writing the map point data to the CSV file.
            pointData << worldPos.at<double>(0) << "," << worldPos.at<double>(1) << "," << worldPos.at<double>(2);
            pointData << "," << p->GetMinDistanceInvariance() << "," << p->GetMaxDistanceInvariance() << ","
                      << Pn.at<double>(0) << "," << Pn.at<double>(1) << "," << Pn.at<double>(2);

            std::map<ORB_SLAM2::KeyFrame *, size_t> observations = p->GetObservations(); // creating a map observations  with key of type  ORB_SLAM2::KeyFrame * and the values type is size_t 
                                                                                                // GetObservations return a map of observations for map point p
            for (auto &obs : observations)
            {
                ORB_SLAM2::KeyFrame *currentFrame = obs.first; // take the value of the keyframe pointer from the current obs and save it in currentFrame
                if (!currentFrame->image.empty()) // check if the currentFrame image is not empty 
                {
                    size_t pointIndex = obs.second;     // take the value from the current obs and save it in pointIndex
                    cv::KeyPoint keyPoint = currentFrame->mvKeysUn[pointIndex]; // save the keypoint associated with the observation of the map point and retrieves the 2D feature point
                    cv::Point2f featurePoint = keyPoint.pt;     //take the 2d feature coordinates from the keypoint and save it in feature Point 
                    pointData << "," << currentFrame->mnId << "," << featurePoint.x << "," << featurePoint.y; // write the info to the file 
                }
            }
            pointData << std::endl;
        }
    }
    pointData.close();  // Close the CSV file.
 
}

void Simulator::extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                               Eigen::MatrixXf &surface)
{
    std::vector<Eigen::Vector3<unsigned int>> surfaceIndices; // Initialize surfaceIndices vector to store surface indices
    for (auto &o : modelGeometry.objects) // iterate through the objects in modelGeometry
    {
        if (o.first == modelTextureNameToAlignTo) // check if the object name equal to modelTextureNameToAlignTo  
        {
            const auto &it_vert = o.second.attributes.find("vertex_indices"); //find the vertex indices in the object and store it in it_vert (as a ref)
            if (it_vert != o.second.attributes.end())
            {
                const auto &vs = std::get<pangolin::Image<unsigned int>>(it_vert->second);  // extracts the vertex data of a 3D model from the it_vert and stores it in the vs
                for (size_t i = 0; i < vs.h; ++i) 
                {
                    const Eigen::Map<const Eigen::Vector3<unsigned int>> v(vs.RowPtr(i));   // creating an Eigen vector v that maps to the triplet of vertex indices in the current row i of vs
                    surfaceIndices.emplace_back(v); // triplet of vertex indices represented by v is added to the surfaceIndices vector.
                }
            }
        }
    }
    surface = Eigen::MatrixXf(surfaceIndices.size() * 3, 3);    // Create a matrix to store the extracted surface vertices
    int currentIndex = 0;
    for (const auto &b : modelGeometry.buffers) //iterate through the model geometry buffer
    {
        const auto &it_vert = b.second.attributes.find("vertex"); //find the attribute with the key "vertex" in the attributes map
        if (it_vert != b.second.attributes.end())
        {
            const auto &vs = std::get<pangolin::Image<float>>(it_vert->second);    // extracts the vertex data of a 3D model from the it_vert and stores it in the vs
            for (auto &row : surfaceIndices)
            {
                for (auto &i : row)
                {
                    // Map the vertex data to an Eigen vector and store it in the surface matrix
                    const Eigen::Map<const Eigen::Vector3f> v(vs.RowPtr(i));
                    surface.row(currentIndex++) = v;
                }
            }
        }
    }
}

void Simulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value)
{
    double rand = double(value) * (M_PI / 180); // convert the value from degrees to radiant 

    // calulate the sin and cos of the angle
    double c = std::cos(rand); 
    double s = std::sin(rand);

    Eigen::Matrix3d R; // Creating a 3x3 rotation matrix
    R << 1, 0, 0,
        0, c, -s,
        0, s, c;

    //Creating a 4x4 identity matrix with the top left 3x3 block set to R
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());    // Extract the current model view matrix of the camera and convert it to Eigen matrix

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);    // Set the camera's model view matrix to the new matrix containing the pitch rotation
}

void Simulator::intervalOverCommand(
    const std::function<void(pangolin::OpenGlRenderState &, double &)> &func, double value,
    int intervalUsleep, double fps, int totalCommandTimeInSeconds)
{
    double intervalValue = value / (fps * totalCommandTimeInSeconds);   // Calculate the incremental value to be passed to the callback function for each interval.
    int intervalIndex = 0;  
    while (intervalIndex <= fps * totalCommandTimeInSeconds) // we will Execute the callback function for each interval within the  totalCommandTimeInSeconds.
    {
        Sleep(intervalUsleep);  // Sleep for intervalUsleep duration that for controling the timing of intervals.
        func(s_cam, intervalValue); //call the callback function (e.g applyYawRotationToModelCam)
        intervalIndex += 1;
    }
}

void Simulator::applyCommand(std::string &command, double value, int intervalUsleep, double fps,
                             int totalCommandTimeInSeconds)
{
    // Check the type of the command call the appropriate intervalOverCommand function based on the command
    if (command == "cw")
    {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);            // Rotate the camera clockwise
    }
    else if (command == "ccw")
    {
        intervalOverCommand(Simulator::applyYawRotationToModelCam, -1 * value,
                            intervalUsleep, fps,
                            totalCommandTimeInSeconds);     // Rotate the camera counter clockwise (-1 * value)
    }
    else if (command == "forward")
    {
        intervalOverCommand(Simulator::applyForwardToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);      // Move the camera forward
    }
    else if (command == "back")
    {
        intervalOverCommand(Simulator::applyForwardToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds); // Move the camera backward
    }
    else if (command == "right")
    {
        intervalOverCommand(Simulator::applyRightToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds); // Move the camera to the right 
    }
    else if (command == "left")
    {
        intervalOverCommand(Simulator::applyRightToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);  // Move the camera to the left
    }
    else if (command == "up")
    {
        intervalOverCommand(Simulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);     // Move the camera up
    }
    else if (command == "down")
    {
        intervalOverCommand(Simulator::applyUpModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);     // Move the camera down
    }
}

void Simulator::applyUpModelCam(pangolin::OpenGlRenderState &cam, double value)
{
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());   // convert pangolin::OpenGlRenderState cam.GetModelViewMatrix() to eigen matrix 
    camMatrix(1, 3) += value;    // add value to y coordinate of the camera position to move it up or down
    cam.SetModelViewMatrix(camMatrix); // setting the model view matrix to our updated one camMatrix
}

void Simulator::applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value)
{
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());   // convert pangolin::OpenGlRenderState cam.GetModelViewMatrix() to eigen matrix 
    camMatrix(2, 3) += value;   // add value to z coordinate of the camera position to move it forward or backward
    cam.SetModelViewMatrix(camMatrix);  // setting the model view matrix to our updated one camMatrix
}

void Simulator::applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value)
{
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());   // convert pangolin::OpenGlRenderState cam.GetModelViewMatrix() to eigen matrix 
    camMatrix(0, 3) += value;    // add value to x coordinate of the camera position to move it right or left
    cam.SetModelViewMatrix(camMatrix);  // setting the model view matrix to our updated one camMatrix
}

void Simulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value)
{
    double rand = double(value) * (M_PI / 180);   // convert the value from degrees to radiant 
    // calulate the sin and cos of the angle
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;  // Creating a 3x3 rotation matrix
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;

    //Creating a 4x4 identity matrix with the top left 3x3 block set to R
    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());   // Extract the current model view matrix of the camera and convert it to Eigen matrix

    // Left-multiply the rotation
    camMatrix = pangolinR * camMatrix;

    // Convert back to pangolin matrix and set
    pangolin::OpenGlMatrix newModelView;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            newModelView.m[j * 4 + i] = camMatrix(i, j);
        }
    }

    cam.SetModelViewMatrix(newModelView);   // Set the camera's model view matrix to the new matrix containing the yaw rotation
}

void Simulator::alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry,
                                             std::string modelTextureNameToAlignTo)
{
    Eigen::MatrixXf surface; // creating matrix surface to save the extracted Surface from the modelGeometry
    extractSurface(modelGeometry, modelTextureNameToAlignTo, surface); //call func so we will get our surface initiate 
   
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(surface, Eigen::ComputeThinU | Eigen::ComputeThinV);     // Perform  SVD on the surface matrix to find the normal vector v
    svd.computeV();
    Eigen::Vector3f v = svd.matrixV().col(2);

    const auto mvm = pangolin::ModelViewLookAt(v.x(), v.y(), v.z(), 0, 0, 0, 0.0,
                                               -1.0,
                                               pangolin::AxisY); //creating look at transformation using the normal vector 
    const auto proj = pangolin::ProjectionMatrix(viewportDesiredSize(0), viewportDesiredSize(1), K(0, 0), K(1, 1),
                                                 K(0, 2), K(1, 2), 0.1, 20); // create projection matrix  
    s_cam.SetModelViewMatrix(mvm);  // Set the camera's model view matrix to the new matrix to align it to the surface
    s_cam.SetProjectionMatrix(proj); //  Set the projection matrix to the new matrix 
    applyPitchRotationToModelCam(s_cam, -90);   // do pitch rotation to the camera rotating it by -90 degrees 

}