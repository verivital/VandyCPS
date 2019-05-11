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


#ifndef CWRU_OPENCV_COMMON_CIRCLE_DETECTION_H
#define CWRU_OPENCV_COMMON_CIRCLE_DETECTION_H
// Opencv Includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>


namespace cv_circle
{

/**
 * @brief fill an ellipse from a seed point. 
 *
 * @param const cv::Mat & inputImg the black and white input image.
 * @param cv::Point seedPt a point inside the ellipse.
 *
 * This function implicitly assumes that the image is BW and that the ellipses are black.
 */
cv::RotatedRect fillEllipseBW(const cv::Mat & inputImg, cv::Point seedPt);

/*
 * This function optimizes an ellipse based on an initial seed ellipse.
 *
 * @param const cv::Mat & inputImg, a black and white input image.
 * @param cv::RotatedRect &ellipse The ellipse that is being optimized.
 * @param int padding @TODO(biocubed) identify the purpose of the padding.
 *
 * This function optimizes the ellipse from an initial ellipse guess.
 */
void optimizeEllipseBW(const cv::Mat & inputImg, cv::RotatedRect &ellipse, int padding);

};  // namespace cv_circle

#endif  // CWRU_OPENCV_COMMON_CIRCLE_DETECTION_H
