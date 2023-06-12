#include "include/simulator.h"

Simulator::Simulator() {
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    this->mCloudScanned = std::vector<cv::Point3d>();

    std::string configPath = data["DroneYamlPathSlam"];
    cv::FileStorage fSettings(configPath, cv::FileStorage::READ);

    this->mViewpointX = fSettings["Viewer.ViewpointX"];
    this->mViewpointY = fSettings["Viewer.ViewpointY"];
    this->mViewpointZ = fSettings["Viewer.ViewpointZ"];
    this->mViewpointF = fSettings["Viewer.ViewpointF"];

    std::string map_input_dir = data["mapInputDir"];
    this->mCloudPointPath = map_input_dir + "cloud1.csv";

    double startPointX = data["startingCameraPosX"];
    double startPointY = data["startingCameraPosY"];
    double startPointZ = data["startingCameraPosZ"];

    this->mStartPosition = cv::Point3d(startPointX, startPointY, startPointZ);

    this->mStartYaw = data["yawRad"];
    this->mStartPitch = data["pitchRad"];
    this->mStartRoll = data["rollRad"];

    this->mPointSize = fSettings["Viewer.PointSize"];

    this->mTwc.SetIdentity();
    this->mTcw.SetIdentity();

    this->mMovingScale = data["movingScale"];
    this->mRotateScale = data["rotateScale"];

    pangolin::CreateWindowAndBind("Simulator Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // For buttons TODO
    // pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

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

    this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
    this->mPointsSeen = std::vector<cv::Point3d>();
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
    glPointSize(this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(auto point : this->mPointsSeen)
    {
        glVertex3f((float)point.x, (float)point.y, (float)point.z);
    }
    glEnd();

    glPointSize(this->mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(auto point : this->mNewPointsSeen)
    {
        glVertex3f((float)point.x, (float)point.y, (float)point.z);

    }
    glEnd();
}

void Simulator::saveOnlyNewPoints() {
    std::vector<cv::Point3d>::iterator it;
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

std::vector<cv::Point3d> Simulator::GetCloudPoint() {
    return this->mCloudScanned;
}

void Simulator::BuildCloudScanned() {
    // Erased mNewPointsSeen to only new points but not combined yet so insert both
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());
    this->mCloudScanned.insert(this->mCloudScanned.end(), this->mPointsSeen.begin(), this->mPointsSeen.end());
}

void Simulator::Run() {
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

        this->mD_cam.Activate(this->mS_cam);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (this->mShowPoints) {
            this->drawMapPoints();
        }

        pangolin::FinishFrame();

        if (this->mMoveLeft)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyRightToModelCam(-this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveLeft = false;
        }

        if (this->mMoveRight)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyRightToModelCam(this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveRight = false;
        }

        if (this->mMoveDown)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            // Opengl has inversed Y axis so we pass -value
            this->applyUpToModelCam(this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveDown = false;
        }

        if (this->mMoveUp)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            // Opengl has inversed Y axis so we pass -value
            this->applyUpToModelCam(-this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveUp = false;
        }

        if (this->mMoveBackward)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyForwardToModelCam(-this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveBackward = false;
        }

        if (this->mMoveForward)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyForwardToModelCam(this->mMovingScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mMoveForward = false;
        }

        if (this->mRotateLeft)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyYawRotationToModelCam(-this->mRotateScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mRotateLeft = false;
        }

        if (this->mRotateRight)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyYawRotationToModelCam(this->mRotateScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mRotateRight = false;
        }

        if (this->mRotateDown)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyPitchRotationToModelCam(-this->mRotateScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mRotateDown = false;
        }

        if (this->mRotateUp)
        {
            this->mPointsSeen.insert(this->mPointsSeen.end(), this->mNewPointsSeen.begin(), this->mNewPointsSeen.end());

            this->applyPitchRotationToModelCam(this->mRotateScale);

            this->mNewPointsSeen = Auxiliary::getPointsFromTcw(this->mCloudPointPath, this->mTcw, this->mTwc);
            this->saveOnlyNewPoints();

            this->mRotateUp = false;
        }

        if (mReset) {
            this->reset();
        }
    }

    this->BuildCloudScanned();
}
