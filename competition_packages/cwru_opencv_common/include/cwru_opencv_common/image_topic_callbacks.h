/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015 Russell Jackson
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/


#ifndef IMAGE_CALLBACKS_H
#define IMAGE_CALLBACKS_H


// This file defines a common image processing call back which can be used to subscribe to image topics.
// The purpose is to make the main files leaner by not needed a full topic call back definition/ declaration.


/*
 * @brief The function will load and attempt to transcribe the image type:
 */
image_transport::Subscriber imageTopicSubscription(image_transport::ImageTransport &,
    const std::string &imageTopic,const std::string & encoding, cv::Mat * imagePtr, bool * updateBool, int = 1);


it.subscribe("/catadioptric_play/segmented_catheter", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr &)>
      (boost::bind(newImageCallback, _1, std::string("mono8"), &localImageSeg, &freshBw)));


/*
 * @todo: move the function below to a *.cpp file
 */
void imageTopicCallback(const sensor_msgs::ImageConstPtr& msg,const std::string &imageType,cv::Mat* outputImage, bool *newImage)
{
    try
    {
       outputImage[0] =  cv_bridge::toCvShare(msg, imageType.c_str())->image.clone();
       newImage[0] = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to %s.",imageType.c_str(),msg ->encoding.c_str());
        newImage[0] = false;
    }
}

#endif
