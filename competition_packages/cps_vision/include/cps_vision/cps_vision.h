/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Vanderbilt University
 *
 *   Seraphy Wang <ruohan.wang@vanderbilt.edu>
 *	 Yufei Yan <yufei.yan@vanderbilt.edu>
 *	 Ran Hao <ran.hao@vanderbilt.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *	 copyright notice, this list of conditions and the following
 *	 disclaimer in the documentation and/or other materials provided
 *	 with the distribution.
 *   * Neither the name of Vanderbilt University, nor the names of its
 *	 contributors may be used to endorse or promote products derived
 *	 from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPSVISION_H
#define CPSVISION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string>
#include <cstring>
#include <vector>

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <cv_bridge/cv_bridge.h>

#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>

class CPSVision {

private:

	ros::NodeHandle node_handle;

    /**
     * The subscriber of the projection matrices for left and right camera. This is subscribing to camera_info.
     */
	ros::Subscriber projectionMat_subscriber;

    ros::Subscriber pose_subscriber;

    /**
     * @brief get P_right from subsriber
     * @param projectionRight
     */
    void projectionMatCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight);

    /*
     * What is this?
     */
    cv::Mat projection_mat;

    bool freshCameraInfo;

    cv::Mat G1_mat;
    cv::Mat G2_mat;
	cv::Mat Gc_mat;

    cv::Mat R_mat;
    cv::Mat T_mat;
    /**
     * The intrinsic camera matrix.
     */
    cv::Mat C_mat;

public:
    /**
     * The left and right raw images from image pipeline
     */
	cv::Mat raw_image;

	/**
	* @brief - The default constructor
	* Call initializeParticles()
	* Compute Gcb as a cv::mat
	* Initialize empty tool images
	*/
	CPSVision(ros::NodeHandle *nodehandle);

	/**
	 * @brief- The deconstructor
	 */
	~CPSVision();

    bool freshpose;

    cv::Mat P1_mat;
    cv::Mat P2_mat;
    cv::Mat pixel_mat;  //use this for the relative position computation

    void getPose(const nav_msgs::Odometry::ConstPtr &pose);

    void getG1();

    void getG2();

    bool matchPattern(std::string filenames,const cv::Mat &rawImg );
    bool findShape(const cv::Mat &blueImage);

    cv::Mat computeGlobalPose();

    cv::Point2d getRelativePosition();
};

#endif
