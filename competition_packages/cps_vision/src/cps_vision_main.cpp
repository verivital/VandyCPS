#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cps_vision/cps_vision.h>

#include <ros/package.h>
#include <std_msgs/MultiArrayLayout.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

using namespace cv;
using namespace std;
using namespace cv_projective;

bool freshImage;
bool match;

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		//cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
		//outputImage[0] = src.clone();
		cv_ptr = cv_bridge::toCvCopy(msg);
		outputImage[0] = cv_ptr->image;
		freshImage = true;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

}

bool findTarget(const cv::Mat &image,cv::Mat &blueImage){

    cv::Mat blueImage1 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage2 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage3 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage4 = cv::Mat::zeros(480, 640, CV_8UC1);
    /*
     * old set
     */
//	cv::inRange(image, cv::Scalar(100, 110, 10), cv::Scalar(200,200,60), blueImage4);   // 100, 60, 50   200, 200, 100 object
//    cv::inRange(image, cv::Scalar(100, 20, 0), cv::Scalar(200,100,60), blueImage1); //100, 20 ,0  200, 100, 100 new1
//    cv::inRange(image, cv::Scalar(200, 100, 0), cv::Scalar(255,200,100), blueImage2);  // new2
//    cv::inRange(image, cv::Scalar(20, 20, 0), cv::Scalar(100,100,10), blueImage3);  //new11

    /*
     * new set for real situation test
     */
    cv::inRange(image, cv::Scalar(0, 150, 10), cv::Scalar(255,250,60), blueImage);   //for pictures with grass
//    cv::inRange(image, cv::Scalar(100, 20, 0), cv::Scalar(200,100,60), blueImage1); //100, 20 ,0  200, 100, 100 new1
//    cv::inRange(image, cv::Scalar(200, 100, 0), cv::Scalar(255,200,100), blueImage2);  // new2
//    cv::inRange(image, cv::Scalar(20, 20, 0), cv::Scalar(100,100,10), blueImage3);  //new11

//    cv::add(blueImage1, blueImage2, blueImage);
//    cv::add(blueImage3, blueImage, blueImage);
//    cv::add(blueImage4, blueImage, blueImage);

	imshow("Image with only blue pixel", blueImage);
//	cv::waitKey();

	return cv::countNonZero(blueImage) > 100; // Need to be specified for different cases.
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    CPSVision CPSVision(&nh);
    ros::Duration(2).sleep(); //wait for the callbacks to start

    freshImage = false;
    match = false; // the match flag to test if the blue marker is here
    ros::Rate loop_rate(50); //looprate to sleep for publishers

    std_msgs::Int32 marker_exist; //the marker flag data 1 or 0
    marker_exist.data = 0;

    std_msgs::Float64MultiArray marker_position_data; // the position to send to the control node
    marker_position_data.layout.dim.push_back(std_msgs::MultiArrayDimension()); // set up the size and other params of the position
    marker_position_data.layout.dim[0].size = 2; //size of 2 array
    marker_position_data.layout.dim[0].stride = 1;
    marker_position_data.layout.dim[0].label = "position";

	std::string cps_vision_pkg = ros::package::getPath("cps_vision"); // the path of the package
	std::string model_path = cps_vision_pkg + "/object.jpg"; // our template

    ros::Publisher marker_flag = nh.advertise<std_msgs::Int32>("/navigation/marker_find", 1000);
    ros::Publisher marker_position = nh.advertise<std_msgs::Float64MultiArray>("/navigation/marker_position", 1000);

    //get image size from camera model, or initialize segmented images,
    cv::Mat raw_image = cv::Mat::zeros(480, 640, CV_8UC3);//this is 3 channel image

    /***
     * for debugging and field testing
     */
    raw_image = imread("/home/ranhao/ros_ws/src/cps_vision/samples/2"
                               ".jpg",IMREAD_COLOR);
    Size size(480, 640);
    resize(raw_image,raw_image,size);
    cv::imshow("raw image ", raw_image);

    cv::Mat blueImage = cv::Mat::zeros(480, 640, CV_8UC1);
    if(findTarget(raw_image, blueImage) /*&& CPSVision.findShape(blueImage)*/)
        match = CPSVision.matchPattern("/home/ranhao/ros_ws/src/cps_vision/object.jpg",blueImage);
    cv::waitKey();
    ROS_INFO_STREAM("match "<< match);

    /***
     * start responding to the real camera node
     */
//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber img_sub_l = it.subscribe(
//            "/camera/rgb/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &raw_image)));
//
//    ROS_INFO("---- done subscribe -----");
//    ros::Duration(1).sleep();
//	while (nh.ok()) {
//		ros::spinOnce();
//		// if camera is ready, track segmented image
//		if (freshImage) {
//			ros::spinOnce();
//            cv::cvtColor(raw_image, raw_image, CV_BGR2RGB);
//            CPSVision.getG1(); // get the pose, for getting the T mat and R mat
//            if (findTarget(raw_image, blueImage) && CPSVision.freshpose)
//            {
//                ROS_INFO("target found");
//                match = CPSVision.matchPattern(model_path, blueImage);
//                if(match){ //have all the keypoints matched
//                        Point2d rel_position;
//                        rel_position = CPSVision.getRelativePosition();
//                        marker_exist.data = 1;  // report: find
//                        ROS_INFO_STREAM("rel_position"<< rel_position);
//
//                        marker_position_data.data.clear();
//                        marker_position_data.data.push_back(rel_position.x);
//                        marker_position_data.data.push_back(rel_position.y);
//
//                }else{
//                    marker_exist.data = 0;  // the 2nd one didn't find target
//                }
//
//            }else{
//                // ROS_INFO_STREAM("NO fresh image of No fresh Pose!");
//                marker_exist.data = 0;}
//
//            marker_flag.publish(marker_exist);
//            marker_position.publish(marker_position_data);
//            loop_rate.sleep();
//			freshImage = false;
//		}
//
//	}
	return 0;
}
