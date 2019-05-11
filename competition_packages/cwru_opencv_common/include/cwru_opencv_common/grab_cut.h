/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell Jackson <rcj33@case.edu>
 *
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
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *
 */


/* 
 * The purpose of this file is to demo the grab cut algorithm using ROS compatible hooks.
 */

#ifndef GRAB_CUT_ROS_H
#define GRAB_CUT_ROS_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace cv_grab_cut
{


class GrabCutObject
{
public:
    enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
    static const int radius = 2;
    static const int thickness = -1;

    void reset();
    void setImageAndWinName( const cv::Mat& _image, const std::string& _winName );
    void showImage() const;
    void mouseClick( int event, int x, int y, int flags, void* param );
    int nextIter();
    int getIterCount() const { return iterCount; }
    int getMask(cv::Mat & binMask) const;
private:
    void setRectInMask();
    void setLblsInMask( int flags, cv::Point p, bool isPr );

    const std::string* winName;
    const cv::Mat* image;
    cv::Mat mask;
    cv::Mat bgdModel, fgdModel;

    uchar rectState, lblsState, prLblsState;
    bool isInitialized;

    cv::Rect rect;
    std::vector<cv::Point> fgdPxls, bgdPxls, prFgdPxls, prBgdPxls;
    int iterCount;
};

void grabCutDemo(const cv::Mat &, cv::Mat & );

};



#endif
