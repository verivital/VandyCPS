#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cps_vision/cps_vision.h>

#include <ros/package.h>
#include <sstream>
#include <iostream>
using namespace cv;
using namespace std;
using namespace cv_projective;

bool freshImage;

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

int main(int argc, char **argv) {

    ros::init(argc, argv, "pic_collection_node");
    ros::NodeHandle nh;

    CPSVision CPSVision(&nh);
    ros::Duration(2).sleep(); //wait for the callbacks to start

    freshImage = false;

	std::string cps_vision_pkg = ros::package::getPath("cps_vision"); // the path of the package

    //get image size from camera model, or initialize segmented images,
    cv::Mat raw_image = cv::Mat::zeros(480, 640, CV_8UC3);//this is 3 channel image

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe(
            "/camera/rgb/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &raw_image)));

    ROS_INFO("---- done subscribe -----");
    ros::Duration(1).sleep();
    for (int i = 0; i < 20; ++i)
    {
        stringstream i_string;
        i_string << i;
        std::string img_path = cps_vision_pkg + "/samples/" + i_string.str() + ".jpg";
        ros::spinOnce();
        // if camera is ready, track segmented image
        if (freshImage) {
            cv::cvtColor(raw_image, raw_image, CV_BGR2RGB);
            imwrite(img_path, raw_image);

            freshImage = false;
        }
        ROS_INFO_STREAM("Finish picture: " << i);
        ros::Duration(3).sleep();
    }

	return 0;
}
