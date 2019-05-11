//
// Created by tipakorng on 6/1/16.
//

#ifndef CWRU_OPENCV_COMMON_MANUAL_LABELING_H
#define CWRU_OPENCV_COMMON_MANUAL_LABELING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cwru_opencv_common/opencv_local.h>
#include <cwru_opencv_common/opencv_ui.h>
#include <cv_bridge/cv_bridge.h>
#include <cwru_opencv_common/image_label.h>
#include <image_transport/image_transport.h>

/**
 * \brief This class is a manual labeling server.
 */
class ManualLabeling {

public:

    /**
     * \brief Constructor
     */
    ManualLabeling(ros::NodeHandle &nodeHandle);

    /**
     * \brief Destructor
     */
    ~ManualLabeling();

    /**
     * \brief Manual labeling callback function
     */
    bool manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
                                cwru_opencv_common::image_label::Response& response);


    /**
     * \brief image subscription callback.
     */
    void newImageCallback(const sensor_msgs::ImageConstPtr& msg);

protected:

 
    /**
     * \brief ROS node handle
     */
    ros::NodeHandle nodeHandle_;

    /**
     * \brief Manual image labeling server
     */
    ros::ServiceServer manualLabelingServer_;

    /**
     * \brief image transport object.
     */
    image_transport::ImageTransport it;

    /**
     * \brief local image def.
     */
    cv::Mat localImage;


    /**
     * \brief image point
     */
    cv::Point imagePt;

    /**
     * \brief is an image ready
     */
    bool imageRdy;


    image_transport::Subscriber img_sub;


    geometry_msgs::Polygon ptList;


};

#endif // CWRU_OPENCV_COMMON_MANUAL_LABELING_H
