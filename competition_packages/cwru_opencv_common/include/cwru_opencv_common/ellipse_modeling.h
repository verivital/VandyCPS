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
 * The functions declared in this file are meant to handle the projective geometry of circles and ellipses.
 * Examples including projecting a planar circle in 3d into an ellipse in the camera image space.
 */

#ifndef ELLIPSEMODEL_H
#define ELLIPSEMODEL_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cwru_opencv_common/opencv_local.h"

namespace cv_ellipse
{

/**
 *  @brief ellipse2Mat takes a RotatedRect (ellipse) and generates a 3x3 Conic Mat.
 *
 * The input is an opencv RotatedRect, the output is a 3x3 CV_64FC1 matrix.
 * The matrix is generated based on the conic matrix representation (v^T)A_qv = 0.
 * The polynoimial is A x^2 + B xy +C^2 + Dx + Ey + F = 0;
 * A_q = [ A B/2 D/2; B/2 C E/2; D/2 E/2 F]
 * v = [x y 1]^T
 */
cv::Mat ellipse2Mat(cv::RotatedRect, cv::OutputArray = cv::noArray());


/** \brief findEllipseRotTransMat takes a detected ellipse from an image. 
 *  Then computes the 3x3 rotation translation [r1 r2 t]  matrix from it.
 *
 * The input is an opencv RotatedRect of the detected ellipse as well as the known radius of the original circle
 * The camera intrinsic matrix is the 3rd input.
 * The 3x3 output matrix has 2 normalized orthogonal rotation basis vectors as well as a translation vector.
 */
cv::Mat findEllipseRotTransMat(cv::RotatedRect, double, cv::Mat);


float getResultsDerivative(const cv::Mat& vect,const cv::Mat & ellipseMat, cv::OutputArray = cv::noArray());

double computeEllipseEnergy(const cv::Rect &, const cv::RotatedRect&, const cv::Mat &,cv::OutputArray = cv::noArray());

cv::Point2d ellipsePoint(const cv::RotatedRect & input, double angle);

cv::Point ellipsePointR(const cv::RotatedRect & input, double angle);

cv::Mat ellipsePointMat(const cv::RotatedRect & input, double angle);

cv::Mat ellipsePointMatR(const cv::RotatedRect & input, double angle);

};  // namespace cv_ellipse



#endif