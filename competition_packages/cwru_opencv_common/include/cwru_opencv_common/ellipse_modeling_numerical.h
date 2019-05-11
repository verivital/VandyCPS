/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell C Jackson <rcj33@case.edu>
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


 //  This file contains functions which are  meant to handle numerical ellipse optimization.


namespace cv_ellipse_num
{

/*
 * @brief ellipseEnergy computes the energy functional of the alignment between the rotated rect & the ellipse.
 * 
 * @param cv::Mat &: The input image (assumed to be a single channel image)
 * @param cv::Mat &: The 3x4 camera projection matrix.
 * @param cv::Mat &: The transform matrix between the circle frame and the
 * @param cv::Point3d &:  The circle center in (R^3) (The normal is implicitly [0, 0, 1]^T)
 * @param double:   The circle radius.
 * @param int   : the number of circle segments.
 */
double circleEnergy(const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Point3d &, double, int = 10, cv::OutputArray = cv::noArray());


/*
 * @brief projectCirclePoints 
 * 
 * @param std::vector<Point> &: The resulting list of image points (integers)
 * @param cv::Mat &: The 3x4 camera projection matrix.
 * @param cv::Mat &: The transform matrix between the circle frame and the
 * @param cv::Point3d &:  The circle center in (R^3) (The normal is implicitly [0, 0, 1]^T)
 * @param double:   The circle radius.
 * 
 * @return the ROI which encompasses the list of points.
 */
cv::Rect projectCirclePoints(std::vector<cv::Point> &, const cv::Mat &, const cv::Mat &, const cv::Point3d &, double rad, int = 10);


/**
 * @brief deprojectEllipseEnds deproject the endpoints of the ellipse
 * 
 * @param cv_local::rotatedRectStereoCorr &: The ellipse information
 * @param cv::Point3d &: The 3x4 camera projection matrix.
 * @param cv::Mat &: The transform matrix between the circle frame and the
 * @param cv::Point3d &:  The circle center in (R^3) (The normal is implicitly [0, 0, 1]^T)
 * @param double:   The circle radius.
 * 
 * @return the ROI which encompasses the list of points.
 */
void deprojectEllipseEnds(const cv_local::rotatedRectStereoCorr &, cv::Point3d & , cv::Point3d & , const cv::Mat & , const cv::Mat &);

};  // namespace cv_ellipse_num
