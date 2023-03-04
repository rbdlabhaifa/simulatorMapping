#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <ctello.h>
#include "include/Auxiliary.h"

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6, 9};
std::shared_ptr<cv::VideoCapture> capture;
std::shared_ptr<cv::Mat> frame;
std::shared_ptr<bool> stop;
void videoDroneThread(std::string &videoPath){
    capture = std::make_shared<cv::VideoCapture>(videoPath);
    stop = std::make_shared<bool>(false);
    frame = std::make_shared<cv::Mat>();
    while(!*stop){
        capture->read(*frame);
    }
    capture->release();
}
int main() {
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i{0}; i < CHECKERBOARD[1]; i++) {
        for (int j{0}; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j*6, i*6, 0));
    }

    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();
    sleep(10);
    ctello::Tello tello;
    tello.SendCommandWithResponse("streamon");
    std::string videoPath = data["onlineVideoPath"];
    std::thread t = std::thread(videoDroneThread,std::ref(videoPath));
    sleep(5);

    cv::Mat  gray;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;
    int waitKey = 0;
    std::cout << "frame size" << *frame->size << std::endl;
    // Looping over all the images in the directory
    for (int i{0}; i < 100; i++) {
        cv::Mat frameCopy = *frame;
        if (frameCopy.empty()){
            i--;
            continue;
        }
        //cv::resize(frame, frame, cv::Size(640, 480));
        cv::cvtColor(frameCopy, gray, cv::COLOR_BGR2GRAY);
        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                            cv::CALIB_CB_NORMALIZE_IMAGE);

        if (success) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frameCopy, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        } else {
            i--;
        }
        cv::imshow("Image", frameCopy);
        cv::waitKey(1);
        sleep(1);
    }
    *stop = true;
    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */
    std::cout << "calibrate dont shut down" << std::endl;

    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;

    return 0;
}