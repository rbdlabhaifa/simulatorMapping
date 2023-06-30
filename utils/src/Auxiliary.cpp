//
// Created by rbdstudent on 17/06/2021.
//

#include "include/Auxiliary.h"

std::string Auxiliary::GetGeneralSettingsPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../generalSettings.json";
    return settingPath;
}

bool Auxiliary::isPointVisible(const cv::Point3f& point, const cv::Point3f& cameraPos, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree)
{
    // Define the position and orientation of the camera
    double roll_rad = roll_degree * CV_PI / 180.0;
    double pitch_rad = pitch_degree * CV_PI / 180.0;
    double yaw_rad = yaw_degree * CV_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << roll_rad, pitch_rad, yaw_rad);
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << cameraPos.x, cameraPos.y, cameraPos.z);

    cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) << fx, 0, cx,
                                                      0, fy, cy,
                                                      0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);

    // Project the 3D point onto the image plane
    std::vector<cv::Point3f> points = {point};
    std::vector<cv::Point2f> image_points;

    cv::projectPoints(points, R, tvec, camera_matrix, distCoeffs, image_points);

    // The image_point variable now contains the 2D projection of the 3D point on the image plane
    if (image_points[0].x >= 0 && image_points[0].x < width && image_points[0].y >= 0 && image_points[0].y < height)
    {
        return true;
    }

    return false;
}

void Auxiliary::getPoints(std::string csvPath, std::vector<cv::Point3f> *points, const cv::Point3f &camera_position, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2, int width, int height, float roll_degree, float yaw_degree, float pitch_degree) {
    std::fstream pointData;
    pointData.open(csvPath, std::ios::in);

    std::vector<std::string> row;
    std::string line, word, temp;

    while (!pointData.eof()) {
        cv::Point3f pointToCompare;

        row.clear();
        
        std::getline(pointData, line);

        std::stringstream words(line);

        if (line == "") {
            continue;
        }

        while (std::getline(words, word, ',')) {
            row.push_back(word);
        }
        
        pointToCompare = cv::Point3f(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
        
        if (Auxiliary::isPointVisible(pointToCompare, camera_position, fx, fy, cx, cy, k1, k2, k3, p1, p2, width, height, roll_degree, yaw_degree, pitch_degree)) {
            (*points).push_back(pointToCompare);
        }
    }
    pointData.close();
}

std::vector<cv::Point3f> Auxiliary::FilterPointsInView(std::vector<cv::Point3f> points, cv::Point3f cam_pos, cv::Vec3f cam_angle, cv::Vec3f focal)
{
    // Extract the rotations from the rotation matrix
    float Yaw = cam_angle[0];
    float Pitch = cam_angle[1];
    float Roll = cam_angle[2];

    // Calculate the rotation matrices

    // The rotation on the YZ-plane is the pitch
    float cp = cos(Pitch);
    float sp = sin(Pitch);
    cv::Mat Rx = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0,
                   0, cp, -sp, 0,
                   0, sp, cp, 0,
                   0, 0, 0, 1);
    
    // The rotation on the XZ-plane is the yaw
    float cy = cos(-Yaw);
    float sy = sin(-Yaw);
    cv::Mat Ry = (cv::Mat_<float>(4, 4) << cy, 0, sy, 0,
                   0, 1, 0, 0,
                   -sy, 0, cy, 0,
                   0, 0, 0, 1);
    
    // The rotation on the XY-plane is the roll
    float cr = cos(Roll);
    float sr = sin(Roll);
    cv::Mat Rz = (cv::Mat_<float>(4, 4) << cr, -sr, 0, 0,
                   sr, cr, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1);

    // Matrix to represent the change to cameras axises
    float Cx = cam_pos.x;
    float Cy = cam_pos.y;
    float Cz = cam_pos.z;
    cv::Mat Tc = (cv::Mat_<float>(4, 4) << 1, 0, 0, -Cx,
                   0, 1, 0, -Cy,
                   0, 0, 1, -Cz,
                   0, 0, 0, 1);
    
    // Calculate the extrinsic transformation
    cv::Mat Rt = Rz * Rx * Ry * Tc;

    // Empty set to hold the points that in the view range
    std::vector<cv::Point3f> SeenPoints;

    // Calculate the lengths seen on the picture frame
    float f_depth = focal[0];
    float f_height = focal[1];
    float f_width = focal[2];
    float vt = f_height / f_depth / 2;
    float ht = f_width  / f_depth / 2;

    // Iterate on the points, and deside which point is in the field of view
    for (cv::Point3f point : points)
    {
        // Extract point coordinates
        float Pwx = point.x;
        float Pwy = point.y;
        float Pwz = point.z;

        // Create homogeneous vector for the point 
        cv::Mat Pw = (cv::Mat_<float>(4, 1) << Pwx, Pwy, Pwz, 1);

        // Calculate the position relative to the camera
        cv::Mat Pc = Rt * Pw;

        // If the point have negative side of z-axis, its behind the camera
        if (Pc.at<float>(2, 0) <= 0) {
            continue;
        }

        // Check if horizantlly, relative to the camera, the point inside the FOV
        if (abs(Pc.at<float>(0, 0) / Pc.at<float>(2, 0)) > vt) {
            continue;
        }

        // Check if vertically, relative to the camera, the point inside the FOV
        if (abs(Pc.at<float>(1, 0) / Pc.at<float>(2, 0)) > ht) {
            continue;
        }

        // All the checks passed, the point is seen by the camera.
        // Append the point to the seen points
        SeenPoints.push_back(cv::Point3f(Pwx, Pwy, Pwz));
    }
    
    return SeenPoints;
}

std::vector<cv::Point3d> Auxiliary::getPointsFromPos(const std::string cloud_points, const cv::Point3d camera_position, double yaw, double pitch, double roll, cv::Mat &Twc)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;

    // Check settings file
    cv::FileStorage fsSettings(data["DroneYamlPathSlam"], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << data["DroneYamlPathSlam"] << std::endl;
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

    Eigen::Matrix4d Tcw_eigen = Eigen::Matrix4d::Identity();
    Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(-roll, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Tcw_eigen.block<3, 1>(0, 3) << -camera_position.x, camera_position.y, -camera_position.z;

    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            Tcw.at<double>(i,j) = Tcw_eigen(i,j);
        }
    }

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat mOw = -Rcw.t()*tcw;

    /* Create Matrix for s_cam */
    Eigen::Matrix4d tmp_Tcw_eigen = Eigen::Matrix4d::Identity();
    tmp_Tcw_eigen.block<3, 3>(0, 0) = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * 
                             Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
    tmp_Tcw_eigen.block<3, 1>(0, 3) << camera_position.x, camera_position.y, camera_position.z;

    cv::Mat tmpTcw = cv::Mat::eye(4, 4, CV_64FC1);
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            tmpTcw.at<double>(i,j) = tmp_Tcw_eigen(i,j);
        }
    }

    cv::Mat tmpRcw = tmpTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tmpRwc = tmpRcw.t();
    cv::Mat tmptcw = tmpTcw.rowRange(0,3).col(3);
    cv::Mat tmpMOw = -tmpRcw.t()*tmptcw;

    Twc = cv::Mat::eye(4,4,tmpTcw.type());
    tmpRwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Twc.at<double>(12) = mOw.at<double>(0);
    Twc.at<double>(13) = mOw.at<double>(1);
    Twc.at<double>(14) = mOw.at<double>(2);

    /* End create Matrix for s_cam */

    std::vector<cv::Vec<double, 8>> points;

    pointData.open(cloud_points, std::ios::in);

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
        points.push_back(cv::Vec<double, 8>(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7])));
    }
    pointData.close();

    std::vector<cv::Point3d> seen_points;

    for(cv::Vec<double, 8>  point : points)
    {
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point[0];
        worldPos.at<double>(1) = point[1];
        worldPos.at<double>(2) = point[2];

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
        const double minDistance = point[3];
        const double maxDistance = point[4];
        const cv::Mat PO = worldPos-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point[5];
        Pn.at<double>(1) = point[6];
        Pn.at<double>(2) = point[7];

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5)
            continue;

        seen_points.push_back(cv::Point3d(worldPos.at<double>(0), worldPos.at<double>(1), worldPos.at<double>(2)));
    }

    return seen_points;
}

std::vector<cv::Point3d> Auxiliary::getPointsFromTcw(const std::string cloud_points, const pangolin::OpenGlMatrix &Tcw, pangolin::OpenGlMatrix &Twc)
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::ifstream pointData;
    std::vector<std::string> row;
    std::string line, word, temp;

    // Check settings file
    cv::FileStorage fsSettings(data["DroneYamlPathSlam"], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << data["DroneYamlPathSlam"] << std::endl;
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
            Tcw_cv.at<double>(i,j) = Tcw.m[j * 4 + i];
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
            Twc.m[j * 4 + i] = Twc_cv.at<double>(i, j);
        }
    }

    std::vector<cv::Vec<double, 8>> points;

    pointData.open(cloud_points, std::ios::in);

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
        points.push_back(cv::Vec<double, 8>(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]), std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7])));
    }
    pointData.close();

    std::vector<cv::Point3d> seen_points;

    for(cv::Vec<double, 8>  point : points)
    {
        cv::Mat worldPos = cv::Mat::zeros(3, 1, CV_64F);
        worldPos.at<double>(0) = point[0];
        worldPos.at<double>(1) = point[1];
        worldPos.at<double>(2) = point[2];

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
        const double minDistance = point[3];
        const double maxDistance = point[4];
        const cv::Mat PO = worldPos-mOw;
        const double dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = cv::Mat(3, 1, CV_64F);
        Pn.at<double>(0) = point[5];
        Pn.at<double>(1) = point[6];
        Pn.at<double>(2) = point[7];

        const double viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.5)
            continue;

        seen_points.push_back(cv::Point3d(worldPos.at<double>(0), worldPos.at<double>(1), worldPos.at<double>(2)));
    }

    return seen_points;
}

std::vector<std::string> Auxiliary::GetAllFrameDatas()
{
    std::string settingPath = Auxiliary::GetGeneralSettingsPath();
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string map_input_dir = data["mapInputDir"];

    std::vector<std::string> filepaths;

    for (const auto& entry : std::filesystem::directory_iterator(map_input_dir))
    {
        const std::string filename = entry.path().filename().string();
        const int n = std::sscanf(filename.c_str(), "frameData%d.csv", &n);
        if (n == 0)
        {
            continue;
        }

        filepaths.push_back(entry.path().string());
    }

    return filepaths;
}

std::vector<std::string> Auxiliary::GetFrameDatas(double amount)
{
    const std::vector<std::string> input = Auxiliary::GetAllFrameDatas();
    std::vector<std::string> output;

    // calculate the number of strings to add (amount of input size)
    int num_strings_to_add = input.size() * amount;

    // create a vector of indices into the input vector
    std::vector<int> indices(input.size());
    std::iota(indices.begin(), indices.end(), 0);

    // shuffle the indices randomly
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    // add the randomly selected strings to the output vector
    for (int i = 0; i < num_strings_to_add; i++)
    {
        int idx = indices[i];
        output.push_back(input[idx]);
    }

    return output;
}

void Auxiliary::add_unique_points(std::vector<cv::Point3d>& target, const std::vector<cv::Point3d>& source) {
    for (const cv::Point3d& point : source) {
        // check if the point exists in the target vector
        auto it = std::find(target.begin(), target.end(), point);
        if (it == target.end()) {
            // point does not exist in target, so add it
            target.push_back(point);
        }
    }
}
