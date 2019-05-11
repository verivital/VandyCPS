//
// Created by tipakorng on 6/6/16.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_opencv_common/image_label.h>

#include "../include/cwru_opencv_common/manual_labeling.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Manual_labeling_service");

	ManualLabeling* manualLabeling;

    ros::NodeHandle nodeHandle;

    manualLabeling = new ManualLabeling(nodeHandle);

    ros::spin();

    delete manualLabeling;
    return 1;
}