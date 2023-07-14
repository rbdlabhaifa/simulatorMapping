#include "include/simulator.h"

#define RESULT_POINT_X 0.1
#define RESULT_POINT_Y 0.2
#define RESULT_POINT_Z 0.3

std::vector<cv::Mat> readDesc(const std::string& filename, int cols)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        // Handle file open error
        return cv::Mat();
    }

    std::vector<cv::Mat> descs = std::vector<cv::Mat>();

    std::string line;
    int row = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        cv::Mat mat(1, cols, CV_8UC1);
        int col = 0;
        std::string value;

        while (std::getline(iss, value, ',')) {
            mat.at<uchar>(row, col) = static_cast<uchar>(std::stoi(value));
            col++;
        }
        descs.push_back(mat);
    }
    file.close();

    return descs;
}

std::vector<cv::KeyPoint> readKeyPoints(std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open())
    {
        // Handle file open error
        return std::vector<cv::KeyPoint>();
    }

    std::vector<cv::KeyPoint> keyPoints = std::vector<cv::KeyPoint>();

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        std::string value;
        std::vector<std::string> row;

        while (std::getline(iss, value, ',')) {
            row.push_back(value);
        }
        cv::KeyPoint keyPoint(cv::Point2f(stof(row[0]), stof(row[1])), stof(row[2]), stof(row[3]), stof(row[4]), stoi(row[5]), stoi(row[6]));
        keyPoints.push_back(keyPoint);
    }
    file.close();

    return keyPoints;
}

void Simulator::createSimulatorSettings() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../demoSettings.json";
    std::ifstream programData(settingPath);
    programData >> this->mData;
    programData.close();
}

void Simulator::initPoints() {
    std::ifstream pointData;
    std::ifstream descData;
    std::vector<std::string> row;
    std::string line, word, temp;
    int pointIndex;
    std::vector<cv::KeyPoint> currKeyPoints;
    std::string currKeyPointsFilename;
    std::vector<cv::Mat> currDesc;
    std::string currDescFilename;

    cv::Vec<double, 8> point;
    OfflineMapPoint *offlineMapPoint;
    pointData.open(this->mCloudPointPath, std::ios::in);

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
        point = cv::Vec<double, 8>(std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7]), std::stod(row[8]));

        pointIndex = std::stoi(row[0]);
        currDescFilename = this->mSimulatorPath + "point" + std::to_string(pointIndex) + "_descriptors.csv";
        currDesc = readDesc(currDescFilename, 32);
        currKeyPointsFilename = this->mSimulatorPath + "point" + std::to_string(pointIndex) + "_keypoints.csv";
        currKeyPoints = readKeyPoints(currKeyPointsFilename);
        offlineMapPoint = new OfflineMapPoint(cv::Point3d(point[0], point[1], point[2]), point[3], point[4], cv::Point3d(point[5], point[6], point[7]), currKeyPoints, currDesc);
        this->mPoints.emplace_back(offlineMapPoint);
    }
    pointData.close();
}

Simulator::Simulator() {
    this->createSimulatorSettings();

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

    this->reset();
}

void Simulator::build_window(std::string title) {
    pangolin::CreateWindowAndBind(title, 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

std::vector<OfflineMapPoint*> Simulator::getPointsFromTcw() {
    // Check settings file
    cv::FileStorage fsSettings(this->mConfigPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << this->mConfigPath << std::endl;
        exit(-1);
    }

    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    double minX = 3.7;
    double maxX = width;
    double minY = 3.7;
    double maxY = height;

    cv::Mat Tcw_cv = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw_cv.at<double>(i,j) = this->mTcw.m[j * 4 + i];
        }
    }

    cv::Mat Rcw = Tcw_cv.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat tcw = Tcw_cv.rowRange(0, 3).col(3);
    cv::Mat mOw = -Rcw.t() * tcw;

    // Save Twc for s_cam
    cv::Mat Twc_cv = cv::Mat::eye(4, 4, CV_64FC1);
    Rwc.copyTo(Twc_cv.rowRange(0,3).colRange(0,3));
    Twc_cv.at<double>(0, 3) = mOw.at<double>(0);
    Twc_cv.at<double>(1, 3) = mOw.at<double>(1);
    Twc_cv.at<double>(2, 3) = mOw.at<double>(2);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            this->mTwc.m[j * 4 + i] = Twc_cv.at<double>(i, j);
        }
    }

    std::vector<OfflineMapPoint*> seen_points;

    for(OfflineMapPoint*  point : this->mPoints)
    {
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point->point.x;
        worldPos.at<double>(1) = point->point.y;
        worldPos.at<double>(2) = point->point.z;

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

        if(viewCos<0.5)
            continue;

        seen_points.push_back(point);
    }

    return seen_points;
}

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

void Simulator::ToggleFollowCamera() {
    this->mFollowCamera = !this->mFollowCamera;
}

void Simulator::ToggleShowPoints() {
    this->mShowPoints = !this->mShowPoints;
}

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

void Simulator::applyUpToModelCam(double value) {
    // Values are opposite
    this->mTcw.m[3 * 4 + 1] -= value;
}

void Simulator::applyRightToModelCam(double value) {
    // Values are opposite
    this->mTcw.m[3 * 4 + 0] -= value;
}

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

void Simulator::drawMapPoints()
{
    std::vector<OfflineMapPoint*> pointsExceptThisFrame = this->mPointsSeen;
    std::vector<OfflineMapPoint*>::iterator it;
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

    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : pointsExceptThisFrame)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);
    }
    glEnd();

    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(auto point : oldPointsFromFrame)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);

    }
    glEnd();

    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    for(auto point : this->mNewPointsSeen)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);

    }
    glEnd();
}

bool areMatricesEqual(const pangolin::OpenGlMatrix& matrix1, const pangolin::OpenGlMatrix& matrix2) {
    for (int i = 0; i < 16; i++) {
        if (matrix1.m[i] != matrix2.m[i])
            return false;
    }
    return true;
}

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
            keyPoints.push_back(keyPoint);
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

void Simulator::Run() {
    pangolin::OpenGlMatrix previousTwc;

    while (!this->mFinishScan) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

        if (mReset) {
            this->reset();
        }
    }

    pangolin::DestroyWindow(this->mSimulatorViewerTitle);
    this->build_window(this->mResultsWindowTitle);

    this->BuildCloudScanned();
}

std::vector<OfflineMapPoint*> Simulator::GetCloudPoint() {
    return this->mCloudScanned;
}

void Simulator::BuildCloudScanned() {
    // Erased mNewPointsSeen to only new points but not combined yet so insert both
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mPointsSeen.begin(), this->mPointsSeen.end());
}

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

    glPointSize((GLfloat)this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : this->mCloudScanned)
    {
        glVertex3f((float)point->point.x, (float)point->point.y, (float)point->point.z);
    }

    glEnd();

    glPointSize((GLfloat)this->mResultsPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    glVertex3f((float)this->mResultPoint.x, (float)this->mResultPoint.y, (float)this->mResultPoint.z);

    glEnd();

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

    while (!this->mCloseResults) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        this->updateTwcByResultPoint();

        this->mS_cam.Follow(this->mTwc);
        this->mD_cam.Activate(this->mS_cam);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        
        this->drawResultPoints();

        pangolin::FinishFrame();
    }

    pangolin::DestroyWindow(this->mResultsWindowTitle);
}

Simulator::~Simulator() {
    for (auto ptr : this->mPoints) {
        free(ptr);
    }
    if (this->mSystem != nullptr)
        free(this->mSystem);
}
