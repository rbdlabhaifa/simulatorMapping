//
// Created by dean on 11/14/23.
//

#include "simulatorMainClass.h"


cv::Mat rootSimulator::getCurrentLocation() {
    locationLock.lock();
    cv::Mat locationCopy = Tcw.clone();
    locationLock.unlock();
    return locationCopy;
}

void rootSimulator::command(std::string &command, int intervalUsleep, double fps, int totalCommandTimeInSeconds) {
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

void rootSimulator::saveMap(const std::string& prefix) {
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

void rootSimulator::intervalOverCommand(
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

void rootSimulator::applyCommand(std::string &command, double value, int intervalUsleep, double fps,
                                 int totalCommandTimeInSeconds) {
    if (command == "cw") {
        intervalOverCommand(rootSimulator::applyYawRotationToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "ccw") {
        intervalOverCommand(rootSimulator::applyYawRotationToModelCam, -1 * value,
                            intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "forward") {
        intervalOverCommand(rootSimulator::applyForwardToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "back") {
        intervalOverCommand(rootSimulator::applyForwardToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "right") {
        intervalOverCommand(rootSimulator::applyRightToModelCam, -1 * value, intervalUsleep,
                            fps, totalCommandTimeInSeconds);
    } else if (command == "left") {
        intervalOverCommand(rootSimulator::applyRightToModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "up") {
        intervalOverCommand(rootSimulator::applyUpModelCam, -1 * value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    } else if (command == "down") {
        intervalOverCommand(rootSimulator::applyUpModelCam, value, intervalUsleep, fps,
                            totalCommandTimeInSeconds);
    }
}

void rootSimulator::applyUpModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    camMatrix(1, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void rootSimulator::applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());
    camMatrix(2, 3) += value;
    cam.SetModelViewMatrix(camMatrix);
}

void rootSimulator::applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    auto camMatrix = pangolin::ToEigen<double>(cam.GetModelViewMatrix());

    camMatrix(0, 3) += value;

    cam.SetModelViewMatrix(camMatrix);
}
void rootSimulator::applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    cam.SetModelViewMatrix((cam.GetModelViewMatrix().Inverse() * pangolin::OpenGlMatrix::RotateY(value).Inverse()).Inverse());
}

void rootSimulator::applyRollRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    cam.SetModelViewMatrix((cam.GetModelViewMatrix().Inverse() * pangolin::OpenGlMatrix::RotateZ(value).Inverse()).Inverse());
}

void rootSimulator::applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value) {
    cam.SetModelViewMatrix((cam.GetModelViewMatrix().Inverse() * pangolin::OpenGlMatrix::RotateX(value).Inverse()).Inverse());
}

void rootSimulator::setSpeed(double speed)
{
    this->speedFactor = speed;
}

double rootSimulator::getSpeed() const
{
    return this->speedFactor;
}

void rootSimulator::faster()
{
    if(this->speedFactor < 3.0){
        this->speedFactor += 0.1;
    }
}

void rootSimulator::slower(){
    if(this->speedFactor > 0.5){
        this->speedFactor -= 0.1;
    }
}

rootSimulator::~rootSimulator() = default;
