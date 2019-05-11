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


#ifndef BLOCKDETECTION_H
#define BLOCKDETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "cwru_opencv_common/opencv_local.h"


/** \brief detectBlock detects a quadrilateral (in an image) from a seed point.
 *
 * This function uses BW growth segmentation. (flood fill)
 * @param cv::Mat &inputImg : the source image. (single channel?)
 * @param cv::Point seedPt: The seed point of the block detection.
 * @param std::vector<cv::Point2f> &corners: The output of the 4 corners (output by ref)
 * @param bool display: display the results of the detection.
 */
void detectBlock(const cv::Mat &, cv::Point, std::vector<cv::Point2f> &, bool=false);


/** \brief refineBlock refines a detected quadrilateral (in an image).
 *
 * @param cv::Mat &inputImg : the source image. (single channel?)
 * @param std::vector<cv::Point2f> &corners: The output of the 4 corners (output by ref)
 * This param must be pre initialized.
 * @param bool display: display the results of the refinement.
 */
int refineBlock(const cv::Mat&, std::vector<cv::Point2f> &, bool=false);


/** \brief sortPtGrid sorts the point array into a grid. (won't work that well).
 *
 * @param std::vector<cv::Point2f> &corners: The output of the 4 corners (output by ref)
 * This param must be pre initialized.
 * @param bool display: display the results of the sorting.
 */
void sortPtGrid(std::vector<cv::Point2f> &, const cv::Size &, bool = true);


/** \brief exportPointFile(const char*,std::vector<std::vector<Point3f>> &) 
 *         save the point array of arrays.
 *
 * @param const char* filePath: path to safe the point list.
 * @param const std::vector< std::vector< cv::Point3f > > &inputPoints: The list of object corners
 */
void exportPointFile(const char*, const std::vector< std::vector< cv::Point3f > > &);


#endif
