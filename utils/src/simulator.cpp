/*********** add-comments , task1 ***********/

#include "include/simulator.h"  

#define RESULT_POINT_X 0.1
#define RESULT_POINT_Y 0.2
#define RESULT_POINT_Z 0.3

//this function reads values from the descriptor
std::vector<cv::Mat> readDesc(const std::string& filename, int cols)
{
    std::ifstream file(filename);   //creating an input file stream object 
    if (!file.is_open())    //check, if we can open this file or not.
    {
        // Handle file open error
        return cv::Mat();
    }

    std::vector<cv::Mat> descs = std::vector<cv::Mat>();    // initializes an empty vector of cv::Mat objects, called descs

    std::string line;
    int row = 0;    //i think we can delete this. there is no update for it later. and in line 31 we can write 0 instead of row 
    while (std::getline(file, line)) {  //reading lines from file
        std::istringstream iss(line);   //creates a string stream from the current line

        cv::Mat mat(1, cols, CV_8UC1);
        int col = 0;
        std::string value;

        //the inner loop-> save values for each col
        while (std::getline(iss, value, ',')) {     //read every value from line, which is separatad by ','
            mat.at<uchar>(row, col) = static_cast<uchar>(std::stoi(value)); //store value in matrix (after changing it from string to int)
            col++;  //update num of col(so we can restore the new value in it)
        }
        descs.push_back(mat);   //add mat(who contains the values of line) to desc vector
    }
    file.close();   

    return descs;   //contains mat for each line, and mat containes values for line.
}

//this function reads values and returns keyPoints
std::vector<std::pair<long unsigned int, cv::KeyPoint>> readKeyPoints(std::string filename) {
    std::ifstream file(filename); //creating an input file stream object 
    if (!file.is_open())      //check, if we can open this file or not.
    {
        // Handle file open error
        return std::vector<std::pair<long unsigned int, cv::KeyPoint>>();
    }

    std::vector<std::pair<long unsigned int, cv::KeyPoint>> keyPoints = std::vector<std::pair<long unsigned int, cv::KeyPoint>>();

    std::string line;
    while (std::getline(file, line)) {  //reading lines from file
        std::istringstream iss(line);   //creates a string stream from the current line

        std::string value;
        std::vector<std::string> row;   //row vector that contains values of line

        while (std::getline(iss, value, ',')) {     //read every value from line, which is separatad by ','
            row.push_back(value);   //store value in row vector
        }

        std::pair<long unsigned int, cv::KeyPoint> currentKeyPoint;
       //[0]:frameID, [1]->x, [2]->y, [3]->size of the keypoint,
       //[4]->orientation(angle) of the keypoint, [5]->response of the keypoint
       //[6]->the octave, from which the keypoint has been extracted
       //[7]->the ID of a class to which the object can be assigned.
        long unsigned int frameId = stol(row[0]);
        cv::KeyPoint keyPoint(cv::Point2f(stof(row[1]), stof(row[2])), stof(row[3]), stof(row[4]), stof(row[5]), stoi(row[6]), stoi(row[7]));
        currentKeyPoint.first = frameId;
        currentKeyPoint.second = keyPoint;  
        keyPoints.push_back(currentKeyPoint);   //add current KeyPoint to the keyPoint vector
    }
    file.close();

    return keyPoints;   //return all the keyPoints
}

//this function is used to create and set up the settings for the simulator
void Simulator::createSimulatorSettings() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);    //gets the current working directory path and stores it into the char array
    std::string settingPath = currentDirPath;
    settingPath += "/../demoSettings.json"; //appends the relative path to the settings file demoSettings.json to the current working directory path
    std::ifstream programData(settingPath); //opens the settings file for reading
    programData >> this->mData;
    programData.close();    //close the settings file after reading its content
}

//this function initialize all points
void Simulator::initPoints() {
    //definitions
    std::ifstream pointData;    //creating an input file stream object 
    std::ifstream descData;   
    std::vector<std::string> row;   
    std::string line, word, temp;
    int pointIndex;
    std::vector<std::pair<long unsigned int, cv::KeyPoint>> currKeyPoints;
    std::string currKeyPointsFilename;
    std::vector<cv::Mat> currDesc;
    std::string currDescFilename;

    cv::Vec<double, 8> point;   //each point have 8 values to descripe it (x,y,angle...)
    OfflineMapPoint *offlineMapPoint;
    pointData.open(this->mCloudPointPath, std::ios::in);

    while (!pointData.eof()) {  //checkk if we got the end of the pointData file stream
        row.clear();    

        std::getline(pointData, line);  //reading lines from file

        std::stringstream words(line);  //creates a string stream from the current line

        if (line == "") {   //this line is empty, so check the next line
            continue;
        }

        while (std::getline(words, word, ',')) {     //read every value from line, which is separatad by ','
            try
            {
                std::stod(word);    //string to double
            }
            catch(std::out_of_range)
            {
                word = "0"; //handle error, if word out of range
            }
            row.push_back(word);    //add value to row vector
        }
        point = cv::Vec<double, 8>(std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7]), std::stod(row[8]));

        //update all values for the current point
        pointIndex = std::stoi(row[0]);
        currDescFilename = this->mSimulatorPath + "point" + std::to_string(pointIndex) + "_descriptors.csv";
        currDesc = readDesc(currDescFilename, 32);
        currKeyPointsFilename = this->mSimulatorPath + "point" + std::to_string(pointIndex) + "_keypoints.csv";
        currKeyPoints = readKeyPoints(currKeyPointsFilename);
        offlineMapPoint = new OfflineMapPoint(cv::Point3d(point[0], point[1], point[2]), point[3], point[4], cv::Point3d(point[5], point[6], point[7]), currKeyPoints, currDesc);
        this->mPoints.emplace_back(offlineMapPoint);
    }
    pointData.close();  //close file, that containes all cloud points
}

//this function initialize the Simulator
Simulator::Simulator() {
    this->createSimulatorSettings();        //this function is used to create and set up the settings for the simulator

    //defin and initialize all fields
    this->mPoints = std::vector<OfflineMapPoint*>();

    this->mSimulatorPath = this->mData["simulatorPointsPath"];
    this->mCloudPointPath = this->mSimulatorPath + "cloud0.csv";

    this->initPoints(); 

    this->mCloudScanned = std::vector<OfflineMapPoint*>();

    this->mRealResultPoint = cv::Point3d(RESULT_POINT_X, RESULT_POINT_Y, RESULT_POINT_Z);
    this->mResultPoint = cv::Point3d();

    this->mSimulatorViewerTitle = "Simulator Viewer";
    this->mResultsWindowTitle = "Results";

    this->mConfigPath = this->mData["DroneYamlPathSlam"];
    cv::FileStorage fSettings(this->mConfigPath, cv::FileStorage::READ);

    this->mViewpointX = fSettings["Viewer.ViewpointX"];
    this->mViewpointY = fSettings["Viewer.ViewpointY"];
    this->mViewpointZ = fSettings["Viewer.ViewpointZ"];
    this->mViewpointF = fSettings["Viewer.ViewpointF"];

    double startPointX = this->mData["startingCameraPosX"];
    double startPointY = this->mData["startingCameraPosY"];
    double startPointZ = this->mData["startingCameraPosZ"];

    this->mStartPosition = cv::Point3d(startPointX, startPointY, startPointZ);

    this->mStartYaw = this->mData["yawRad"];
    this->mStartPitch = this->mData["pitchRad"];
    this->mStartRoll = this->mData["rollRad"];

    this->mPointSize = fSettings["Viewer.PointSize"];
    this->mResultsPointSize = this->mPointSize * 5;

    this->mTwc.SetIdentity();
    this->mTcw.SetIdentity();

    this->mMovingScale = this->mData["movingScale"];
    this->mRotateScale = this->mData["rotateScale"];

    this->build_window(this->mSimulatorViewerTitle);

    this->mUseOrbSlam = this->mData["useOrbSlam"];
    this->mVocPath = this->mData["VocabularyPath"];
    this->mTrackImages = this->mData["trackImagesClass"];
    this->mLoadMap = this->mData["loadMap"];
    this->mLoadMapPath = this->mData["loadMapPath"];
    this->mSystem = nullptr;
    if (this->mUseOrbSlam) {
        this->mSystem = new ORB_SLAM2::System(this->mVocPath, this->mConfigPath, ORB_SLAM2::System::MONOCULAR, true, this->mTrackImages,
                                          this->mLoadMap, this->mLoadMapPath, false);
    }

    this->mFollowCamera = true;
    this->mShowPoints = true;
    this->mReset = false;
    this->mMoveLeft = false;
    this->mMoveRight = false;
    this->mMoveDown = false;
    this->mMoveUp = false;
    this->mMoveBackward = false;
    this->mMoveForward = false;
    this->mRotateLeft = false;
    this->mRotateRight = false;
    this->mRotateDown = false;
    this->mRotateUp = false;
    this->mFinishScan = false;

    // Define Camera Render Object (for view / scene browsing)
    this->mS_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, this->mViewpointF, this->mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(this->mViewpointX, this->mViewpointY, this->mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    this->mD_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(this->mS_cam));

    this->reset();  //reinitialize the simulator to its initial state
}

//this function creates a new Pangolin window and binds it for OpenGL rendering.
void Simulator::build_window(std::string title) {
    pangolin::CreateWindowAndBind(title, 1024, 768);    //title of the window, width and height 

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

std::vector<OfflineMapPoint*> Simulator::getPointsFromTcw() {
    // Check settings file
    cv::FileStorage fsSettings(this->mConfigPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())  //check, if we can open this file or not.
    {
        // Handle file open error
        std::cerr << "Failed to open settings file at: " << this->mConfigPath << std::endl;
        exit(-1);
    }

    //the camera's parameters
    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    //the range
    double minX = 3.7;
    double maxX = width;
    double minY = 3.7;
    double maxY = height;

    //transform from camera frame to world frame
    cv::Mat Tcw_cv = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw_cv.at<double>(i,j) = this->mTcw.m[j * 4 + i];
        }
    }

    
    cv::Mat Rcw = Tcw_cv.rowRange(0, 3).colRange(0, 3); //the rotation part of the transformation matrix
    cv::Mat Rwc = Rcw.t();  //the rotation from the camera frame to the world frame
    cv::Mat tcw = Tcw_cv.rowRange(0, 3).col(3); //extracts the translation vector
    cv::Mat mOw = -Rcw.t() * tcw;    //computes the position of the camera center in the world frame

    // Save Twc for s_cam
    cv::Mat Twc_cv = cv::Mat::eye(4, 4, CV_64FC1);
    Rwc.copyTo(Twc_cv.rowRange(0,3).colRange(0,3));
    Twc_cv.at<double>(0, 3) = mOw.at<double>(0);
    Twc_cv.at<double>(1, 3) = mOw.at<double>(1);
    Twc_cv.at<double>(2, 3) = mOw.at<double>(2);

    //copy values
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            this->mTwc.m[j * 4 + i] = Twc_cv.at<double>(i, j);  //*4 beacuse pangolin::OpenGlMatrix stores its data in column-major order
        }
    }

    std::vector<OfflineMapPoint*> seen_points;

    //iterate through all points
    for(OfflineMapPoint*  point : this->mPoints)
    {
        //define world position
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point->point.x;
        worldPos.at<double>(1) = point->point.y;
        worldPos.at<double>(2) = point->point.z;

        //define camera position
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

        //check range
        if(u<minX || u>maxX)
            continue;
        if(v<minY || v>maxY)
            continue;

        // Check distance is in the scale invariance region of the MapPoint
        const double minDistance = point->minDistanceInvariance;
        const double maxDistance = point->maxDistanceInvariance;
        const cv::Mat PO = worldPos-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point->normal.x;
        Pn.at<double>(1) = point->normal.y;
        Pn.at<double>(2) = point->normal.z;

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5) //check if the point is viewed from a sharp angle and might not be reliably visible,
            continue;

        seen_points.push_back(point);   // this point passed all th checks, (it's a good point)
    }

    return seen_points; //return points who passed all the checks
}

//rhis function restart the state of the simulator
void Simulator::reset() {
    this->mShowPoints = true;
    this->mFollow = true;
    this->mFollowCamera = true;
    this->mReset = false;

    this->mCurrentPosition = this->mStartPosition;
    this->mCurrentYaw = this->mStartYaw;
    this->mCurrentPitch = this->mStartPitch;
    this->mCurrentRoll = this->mStartRoll;

    // Opengl has inversed Y axis
    // Assign yaw, pitch and roll rotations and translation
    Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
    Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(this->mCurrentRoll, Eigen::Vector3d::UnitZ()) * 
                            Eigen::AngleAxisd(this->mCurrentYaw, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(this->mCurrentPitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Tcw_eigen(0, 3) = this->mCurrentPosition.x;
    Tcw_eigen(1, 3) = -this->mCurrentPosition.y;
    Tcw_eigen(2, 3) = this->mCurrentPosition.z;

    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            this->mTcw.m[j * 4 + i] = Tcw_eigen(i,j);
        }
    }

    this->mNewPointsSeen = this->getPointsFromTcw();

    this->mPointsSeen = std::vector<OfflineMapPoint*>();
}

//change state of the camera
void Simulator::ToggleFollowCamera() {
    this->mFollowCamera = !this->mFollowCamera;
}
//change state of showing the point
void Simulator::ToggleShowPoints() {
    this->mShowPoints = !this->mShowPoints;
}
//restart
void Simulator::DoReset() {
    this->mReset = true;
}

void Simulator::MoveLeft() {
    this->mMoveLeft = true;
}

void Simulator::MoveRight() {
    this->mMoveRight = true;
}

void Simulator::MoveDown() {
    this->mMoveDown = true;
}

void Simulator::MoveUp() {
    this->mMoveUp = true;
}

void Simulator::MoveBackward() {
    this->mMoveBackward = true;
}

void Simulator::MoveForward() {
    this->mMoveForward = true;
}

void Simulator::RotateLeft() {
    this->mRotateLeft = true;
}

void Simulator::RotateRight() {
    this->mRotateRight = true;
}

void Simulator::RotateDown() {
    this->mRotateDown = true;
}

void Simulator::RotateUp() {
    this->mRotateUp = true;
}

void Simulator::FinishScan() {
    this->mFinishScan = true;
}
//up if values are pos, and down if values are nagative
void Simulator::applyUpToModelCam(double value) {
    // Values are opposite
    this->mTcw.m[3 * 4 + 1] -= value;
}
//right if values are pos, and left if values are nagative
void Simulator::applyRightToModelCam(double value) {
    // Values are opposite
    this->mTcw.m[3 * 4 + 0] -= value;
}
//forward if values are pos, and backward if values are nagative
void Simulator::applyForwardToModelCam(double value) {
    // Values are opposite
    this->mTcw.m[3 * 4 + 2] -= value;
}

void Simulator::applyYawRotationToModelCam(double value) {
    Eigen::Matrix4d Tcw_eigen = pangolin::ToEigen<double>(this->mTcw);

    // Values are opposite
    double rand = -value * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    // Left-multiply the rotation
    Tcw_eigen = pangolinR * Tcw_eigen;

    // Convert back to pangolin matrix and set
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            this->mTcw.m[j * 4 + i] = Tcw_eigen(i, j);
        }
    }
}

void Simulator::applyPitchRotationToModelCam(double value) {
    Eigen::Matrix4d Tcw_eigen = pangolin::ToEigen<double>(this->mTcw);

    // Values are opposite
    double rand = -value * (M_PI / 180);
    double c = std::cos(rand);
    double s = std::sin(rand);

    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, c, -s,
        0, s, c;

    Eigen::Matrix4d pangolinR = Eigen::Matrix4d::Identity();
    pangolinR.block<3, 3>(0, 0) = R;

    // Left-multiply the rotation
    Tcw_eigen = pangolinR * Tcw_eigen;

    // Convert back to pangolin matrix and set
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            this->mTcw.m[j * 4 + i] = Tcw_eigen(i, j);
        }
    }
}
//this function is responsible for rendering the points of interest in the 3D space of the simulator
void Simulator::drawMapPoints()
{
    std::vector<OfflineMapPoint*> pointsExceptThisFrame = this->mPointsSeen;
    std::vector<OfflineMapPoint*>::iterator it;
    
    //erase points that are not in the current frame
    for (it = pointsExceptThisFrame.begin(); it != pointsExceptThisFrame.end();)
    {
        if (std::find(this->mCurrentFramePoints.begin(), this->mCurrentFramePoints.end(), *it) != this->mCurrentFramePoints.end())
        {
            it = pointsExceptThisFrame.erase(it);
        }
        else
        {
            ++it;
        }
    }

    //erase points that in this current frame but not newly seen 
    std::vector<OfflineMapPoint*> oldPointsFromFrame = this->mCurrentFramePoints;
    for (it = oldPointsFromFrame.begin(); it != oldPointsFromFrame.end();)
    {
        if (std::find(this->mNewPointsSeen.begin(), this->mNewPointsSeen.end(), *it) != this->mNewPointsSeen.end())
        {
            it = oldPointsFromFrame.erase(it);
        }
        else
        {
            ++it;
        }
    }

    //points in the excepted frame drawn in black
    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : pointsExceptThisFrame)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);
    }
    glEnd();

    //old points drawn in red
    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(auto point : oldPointsFromFrame)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);

    }
    glEnd();

    //newly seen points drawn in green
    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    for(auto point : this->mNewPointsSeen)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);

    }
    glEnd();
}

//check if matrix1 and matrix 2 are equal.
bool areMatricesEqual(const pangolin::OpenGlMatrix& matrix1, const pangolin::OpenGlMatrix& matrix2) {
    for (int i = 0; i < 16; i++) {
        if (matrix1.m[i] != matrix2.m[i])
            return false;
    }
    return true;
}

//save only points that were newly seen.
//if the point already in 'mNewPointsSeen' that mean that it's not new anymore
void Simulator::saveOnlyNewPoints() {
    this->mNewPointsSeen = this->mCurrentFramePoints;
    std::vector<OfflineMapPoint*>::iterator it;
    for (it = this->mNewPointsSeen.begin(); it != this->mNewPointsSeen.end();)
    {
        if (std::find(this->mPointsSeen.begin(), this->mPointsSeen.end(), *it) != this->mPointsSeen.end())
        {
            it = this->mNewPointsSeen.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

//matrix1=matrix2
void assignPreviousTwc(pangolin::OpenGlMatrix& matrix1, const pangolin::OpenGlMatrix& matrix2) {
    for (int i = 0; i < 16; i++) {
        matrix1.m[i] = matrix2.m[i];
    }
}

void Simulator::trackOrbSlam() {
    // Create timestamp
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto value = now_ms.time_since_epoch();
    double timestamp = value.count() / 1000.0;

    // TODO: Create std::vector<cv::KeyPoint> of all projections of the this->mCurrentFramePoints
    std::vector<cv::KeyPoint> keyPoints;
    for (auto point : this->mCurrentFramePoints) {
        for (auto keyPoint : point->keyPoints) {
            keyPoints.push_back(keyPoint.second);
        }
    }

    // TODO: Create cv::Mat of all the descriptors
    cv::Mat descriptors;
    for (auto point : this->mCurrentFramePoints) {
        for (auto descriptor : point->descriptors) {
            descriptors.push_back(descriptor);
        }
    }

    this->mSystem->TrackMonocular(descriptors, keyPoints, timestamp);
}

//run
void Simulator::Run() {
    pangolin::OpenGlMatrix previousTwc;

    while (!this->mFinishScan) {        //loop until finishing scanning
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //manage the camera state 
        if (this->mFollowCamera && this->mFollow) {
            this->mS_cam.Follow(this->mTwc);
        } else if (this->mFollowCamera && !this->mFollow) {
            this->mS_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            this->mS_cam.Follow(this->mTwc);
            this->mFollow = true;
        } else if (!this->mFollowCamera && this->mFollow) {
            this->mFollow = false;
        }

        this->mCurrentFramePoints = this->getPointsFromTcw();
        // If running with orb-slam move this points to orb-slam
        if (this->mUseOrbSlam)
            this->trackOrbSlam();

        if (!areMatricesEqual(previousTwc, this->mTwc)) {
            this->saveOnlyNewPoints();
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());
        }

        this->mD_cam.Activate(this->mS_cam);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (this->mShowPoints) {
            this->drawMapPoints();
        }

        pangolin::FinishFrame();

        assignPreviousTwc(previousTwc, this->mTwc);

        //check movement and rotation of camera
        if (this->mMoveLeft)
        {
            this->applyRightToModelCam(-this->mMovingScale);
            this->mMoveLeft = false;
        }

        if (this->mMoveRight)
        {
            this->applyRightToModelCam(this->mMovingScale);
            this->mMoveRight = false;
        }

        if (this->mMoveDown)
        {
            // Opengl has inversed Y axis so we pass -value
            this->applyUpToModelCam(this->mMovingScale);
            this->mMoveDown = false;
        }

        if (this->mMoveUp)
        {
            // Opengl has inversed Y axis so we pass -value
            this->applyUpToModelCam(-this->mMovingScale);
            this->mMoveUp = false;
        }

        if (this->mMoveBackward)
        {
            this->applyForwardToModelCam(-this->mMovingScale);
            this->mMoveBackward = false;
        }

        if (this->mMoveForward)
        {
            this->applyForwardToModelCam(this->mMovingScale);
            this->mMoveForward = false;
        }

        if (this->mRotateLeft)
        {
            this->applyYawRotationToModelCam(-this->mRotateScale);
            this->mRotateLeft = false;
        }

        if (this->mRotateRight)
        {
            this->applyYawRotationToModelCam(this->mRotateScale);
            this->mRotateRight = false;
        }

        if (this->mRotateDown)
        {
            this->applyPitchRotationToModelCam(-this->mRotateScale);
            this->mRotateDown = false;
        }

        if (this->mRotateUp)
        {
            this->applyPitchRotationToModelCam(this->mRotateScale);
            this->mRotateUp = false;
        }

        //restart
        if (mReset) {
            this->reset();
        }
    }

    pangolin::DestroyWindow(this->mSimulatorViewerTitle);   //destroy prev window
    this->build_window(this->mResultsWindowTitle);  //build window to show the results

    this->BuildCloudScanned();   // Erased mNewPointsSeen to only new points but not combined yet so insert both
}

//get function
std::vector<OfflineMapPoint*> Simulator::GetCloudPoint() {
    return this->mCloudScanned;
}

void Simulator::BuildCloudScanned() {
    // Erased mNewPointsSeen to only new points but not combined yet so insert both
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mPointsSeen.begin(), this->mPointsSeen.end());
}
//set function
void Simulator::SetResultPoint(const cv::Point3d resultPoint) {
    this->mResultPoint = resultPoint;
}

void Simulator::drawResultPoints() {
    // Remove result point and real result point from cloud scanned if exist
    for(int i = 0; i < this->mCloudScanned.size(); i++) {
        if (*this->mCloudScanned[i] == this->mResultPoint) {
            this->mCloudScanned.erase(this->mCloudScanned.begin() + i);
            break;
        }
    }
    for(int i = 0; i < this->mCloudScanned.size(); i++) {
        if (*this->mCloudScanned[i] == this->mRealResultPoint) {
            this->mCloudScanned.erase(this->mCloudScanned.begin() + i);
            break;
        }
    }

    // Draw the scanned points in black color
    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : this->mCloudScanned)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);
    }

    glEnd();

     // Draw the result point in red color
    glPointSize((GLfloat)this->mResultsPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    glVertex3f((float)this->mResultPoint.x, (float)this->mResultPoint.y, (float)this->mResultPoint.z);

    glEnd();

   // Draw the real result point in green color
    glPointSize((GLfloat)this->mResultsPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    glVertex3f((float)this->mRealResultPoint.x, (float)this->mRealResultPoint.y, (float)this->mRealResultPoint.z);

    glEnd();
}

void Simulator::updateTwcByResultPoint() {
    // TODO: Change Twc to center the result point when I do check results
}

void Simulator::CheckResults() {
    this->mCloseResults = false;

    // Enter a loop that will keep running until the results visualization is to be closed
    while (!this->mCloseResults) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        this->updateTwcByResultPoint(); //Change Twc to center the result point when I do check results

        // activate and mange the camera state (according to the world)
        this->mS_cam.Follow(this->mTwc);
        this->mD_cam.Activate(this->mS_cam);

        // Set the background color to white
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        
        //draw result points
        this->drawResultPoints();

        pangolin::FinishFrame();
    }
    // Destroy the window where results are displayed
    pangolin::DestroyWindow(this->mResultsWindowTitle);
}

//destructor of the Simulator -> free all the resources and the memory that was allocated for the simulator
Simulator::~Simulator() {
    for (auto ptr : this->mPoints) {
        free(ptr);
    }
    if (this->mSystem != nullptr)
        free(this->mSystem);
}
