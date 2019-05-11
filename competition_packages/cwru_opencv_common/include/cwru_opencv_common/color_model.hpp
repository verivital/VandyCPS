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
 * This file defines a class of color tracking model.
 * The model assumes that a color distribution is a gaussian in RGB space. 
 * The segmentation is simply a multivariate gaussian probability (the final result is normalized using the l_infinity norm).
 * A mask is used to generate the initial fit.
 */


#ifndef COLORMODEL_H
#define COLORMODEL_H  

#include <cv.h>


namespace cv_color_model{

class ColorModel{

public:
  explicit ColorModel(const cv::Mat & , const cv::Mat &); // uses a pre-defined mask
  explicit ColorModel(const ColorModel&);  // copy constructor;
  // explicit ColorModel(const cv::Mat &); // manually defined mask.

  cv::Mat segmentImage(const cv::Mat &);

  void printModelInfo();

  void floatMaskInit(const cv::Mat &, const cv::Mat&);
  void binaryMaskInit(const cv::Mat &, const cv::Mat&);

private:
    cv::Matx<float, 3, 3> colorVariance;
    cv::Matx<float, 3, 1> colorMean;
};


// HSV Color Model.
class ColorModelHSV{

public:
  explicit ColorModelHSV(const cv::Mat & , const cv::Mat &, int); // uses a pre-defined mask
  explicit ColorModelHSV(const ColorModelHSV&);  // copy constructor;
  // explicit ColorModelHSV(const cv::Mat &); // manually defined mask.

  cv::Mat segmentImage(const cv::Mat &);

  void printModelInfo();

  void floatMaskInit(const cv::Mat &, const cv::Mat&, int);
  void binaryMaskInit(const cv::Mat &, const cv::Mat&, int);

private:
    cv::Matx<float, 3, 3> colorVariance;
    cv::Matx<float, 3, 1> colorMean;
};


};  // namespace cv_color_model



#endif
