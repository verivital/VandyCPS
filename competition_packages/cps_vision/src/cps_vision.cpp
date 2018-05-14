#include <cps_vision/cps_vision.h>  //everything inside here
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;
RNG rng(12345);

CPSVision::CPSVision(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle){
    projectionMat_subscriber = node_handle.subscribe("/camera/rgb/camera_info", 1, &CPSVision::projectionMatCB, this);
    pose_subscriber = node_handle.subscribe("/mavros/local_position/odom", 1, &CPSVision::getPose, this);

    raw_image = cv::Mat::zeros(480, 640, CV_8UC3);

    C_mat = cv::Mat::zeros(3,4, CV_64FC1);

    P1_mat = cv::Mat::zeros(3, 1, CV_32FC1);
    P2_mat = cv::Mat::zeros(3, 1, CV_32FC1);
    pixel_mat = cv::Mat::zeros(3, 1, CV_32FC1);

    R_mat = cv::Mat::zeros(3, 3, CV_64FC1);
    T_mat = cv::Mat::zeros(3, 1, CV_64FC1);
    G1_mat = cv::Mat::eye(4,4, CV_64FC1);
    G2_mat = cv::Mat::eye(4,4, CV_64FC1); ////currently not used

    Gc_mat = (cv::Mat_<double>(4,4) << 0,   1,   0,   0,
	    					  -1,    0,   0,    0.0,
	    					  0,   0,   1,    0.04,
	    						0, 	   0,  0, 	1); //some rough number for translation

    freshpose = false;

};

CPSVision::~CPSVision() {
};

void CPSVision::projectionMatCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight){

    C_mat.at<double>(0,0) = projectionRight->P[0];
    C_mat.at<double>(0,1) = projectionRight->P[1];
    C_mat.at<double>(0,2) = projectionRight->P[2];
    C_mat.at<double>(0,3) = projectionRight->P[3];

    C_mat.at<double>(1,0) = projectionRight->P[4];
    C_mat.at<double>(1,1) = projectionRight->P[5];
    C_mat.at<double>(1,2) = projectionRight->P[6];
    C_mat.at<double>(1,3) = projectionRight->P[7];

    C_mat.at<double>(2,0) = projectionRight->P[8];
    C_mat.at<double>(2,1) = projectionRight->P[9];
    C_mat.at<double>(2,2) = projectionRight->P[10];
    C_mat.at<double>(2,3) = projectionRight->P[11];
    // ROS_INFO_STREAM("C MATRIX"<<C_mat);

}

void CPSVision::getPose(const nav_msgs::Odometry::ConstPtr &pose) {
    cv::Mat vect3 = cv::Mat::zeros(3,1,CV_64FC1);
    double angle = 2*acos(pose->pose.pose.orientation.w);
    vect3.at<double>(0,0) = angle * pose->pose.pose.orientation.x / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    vect3.at<double>(1,0) = angle * pose->pose.pose.orientation.y / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    vect3.at<double>(2,0) = angle * pose->pose.pose.orientation.z / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    cv::Rodrigues(vect3, R_mat);

    T_mat.at<double>(0,0) = pose->pose.pose.position.x;
    T_mat.at<double>(1,0) = pose->pose.pose.position.y;
    T_mat.at<double>(2,0) = pose->pose.pose.position.z;
    freshpose = true;
    // ROS_INFO_STREAM("T MAT: "<< T_mat);
}


void CPSVision::getG1() {
    R_mat.copyTo(G1_mat.colRange(0, 3).rowRange(0, 3));
    T_mat.copyTo(G1_mat.colRange(3, 4).rowRange(0, 3));

}

void CPSVision::getG2() {

  R_mat.copyTo(G2_mat.colRange(0, 3).rowRange(0, 3));
  T_mat.copyTo(G2_mat.colRange(3, 4).rowRange(0, 3));
}


bool CPSVision::matchPattern(std::string filenames,const cv::Mat &rawImg){

    bool match;

    std::vector<cv::Point2f> filtered_pixels;
    cv::Mat position_pixel = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::Mat targetImg = cv::Mat::zeros(480, 640, CV_8UC3);
    targetImg = imread(filenames, IMREAD_GRAYSCALE); //FIXME filename
    Size size(480,640);
    resize(targetImg,targetImg,size);
    int minHessian = 600; //threshold
    Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector->detect(targetImg, keypoints_1);
    detector->detect(rawImg, keypoints_2);

    if(keypoints_2.size() > 0){
        cv::Mat descriptor_target, descriptor_raw;
        detector->compute(targetImg,keypoints_1, descriptor_target);
        detector->compute(rawImg,keypoints_2, descriptor_raw);

        cv::BFMatcher matcher(NORM_L2);
        std::vector<DMatch> matches;
        std::vector<DMatch> matches_filtered;
//    matcher.knnMatch(descriptor_target,descriptor_raw,matches,2,noArray(),true);
        matcher.match(descriptor_target,descriptor_raw,matches,noArray());

        for (int i = 0; i < matches.size()-1; ++i) {
            for (int j = i; j < matches.size(); ++j) {
                if (matches[i].distance>matches[j].distance){
                    std::swap(matches[i],matches[j]);
                }
            }

        }

        for (int i = 0; i < matches.size()&& i<60; ++i) {
            matches_filtered.push_back(matches[i]);
//            ROS_INFO_STREAM("matches: "<<keypoints_2[matches[i].trainIdx].pt);
            filtered_pixels.push_back(keypoints_2[matches[i].trainIdx].pt);
        }

        for (int j = 0; j < filtered_pixels.size(); ++j) {
            position_pixel.at<float>(0,0) += filtered_pixels[j].x;
            position_pixel.at<float>(1,0) += filtered_pixels[j].y;
        }

        if(filtered_pixels.size() > 0){
            position_pixel.at<float>(0,0) /= filtered_pixels.size();
            position_pixel.at<float>(1,0) /= filtered_pixels.size();
            position_pixel.at<float>(2,0) = 1;
            match = true;
        }else{match = false;}

        cv::Mat match_mat;
        cv::drawMatches(targetImg, keypoints_1,rawImg,keypoints_2,matches_filtered,match_mat);
        imshow("matches image", match_mat);
//        cv::waitKey(10);
    }else{
        match = false;
    }

    pixel_mat = position_pixel.clone();
    P1_mat = position_pixel.clone();
    P2_mat = position_pixel.clone();

    return match;
}

bool CPSVision::findShape(const cv::Mat &blueImage){
    vector<vector<Point> > contours;
    vector<Point> approx;
    vector<Vec4i> hierarchy;
    cv::findContours(blueImage, contours,  hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    cv::Mat drawing = cv::Mat::zeros(480, 640, CV_8UC3);
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }
    imshow("Draw", drawing);
//    waitKey(10);
    for(int i = 0; i < contours.size();i++){
        ROS_INFO_STREAM("CONTOUR:"<<contours.size());
        approxPolyDP(cv::Mat(contours[i]),approx, cv::arcLength(cv::Mat(contours[i]),true)*0.01, true);
        if(approx.size() > 10 && approx.size() <= 14){
            ROS_INFO_STREAM("Cross!" << approx.size());
            return true;
        }
    }

    ROS_INFO_STREAM("NOT Cross! The number is:" << approx.size());
    return false;
}


cv::Mat CPSVision::computeGlobalPose() {
    cv::Mat P_mat = C_mat * Gc_mat * G1_mat;
    cv::Mat Q_mat = C_mat * Gc_mat * G2_mat;
    cv::Mat A_mat = cv::Mat::zeros(4, 4, CV_64FC1);

    float u1 = P1_mat.at<float>(0);
    float v1 = P1_mat.at<float>(1);
    cv::Mat temp_mat = u1 * P_mat.colRange(0, 4).rowRange(2, 3) - P_mat.colRange(0, 4).rowRange(0, 1);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(0, 1));

    temp_mat = v1 * P_mat.colRange(0, 4).rowRange(2, 3) - P_mat.colRange(0, 4).rowRange(1, 2);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(1, 2));

    float u2 = P2_mat.at<float>(0);
    float v2 = P2_mat.at<float>(1);
    temp_mat = u2 * Q_mat.colRange(0, 4).rowRange(2, 3) - Q_mat.colRange(0, 4).rowRange(0, 1);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(2, 3));
    temp_mat = v2 * Q_mat.colRange(0, 4).rowRange(2, 3) - Q_mat.colRange(0, 4).rowRange(1, 2);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(3, 4));

    cv::Mat target_position = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat resulting_position = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::SVD svd = cv::SVD(A_mat);
    resulting_position = svd.vt.colRange(3, 4).rowRange(0, 4);
    double scale_w = resulting_position.at<double>(3, 0);
    target_position = resulting_position / scale_w;
    ROS_INFO_STREAM("Reported posistion is: " << target_position);

    return target_position;
}


cv::Point2d CPSVision::getRelativePosition(){
    cv::Point2d relative_position; // the relative position according to drone's body frame
    double c_x = T_mat.at<double>(2,0) * (pixel_mat.at<float>(0,0) - C_mat.at<double>(0,2)) / C_mat.at<double>(0,0);
    double c_y = T_mat.at<double>(2,0) * (pixel_mat.at<float>(1,0) - C_mat.at<double>(1,2)) / C_mat.at<double>(1,1);

    /* do a transformation to drone coordinate 
     * for camera, z is point outwards against the shot, 
     * which should be the opposite direction to the drone's z when facing the camera to the ground 
     * Notice this is a 3D rigid body transformation
    */
    relative_position.x = -c_y;
    relative_position.y = c_x;

    return relative_position;

}