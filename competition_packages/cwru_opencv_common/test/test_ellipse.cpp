/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Case Western Reserve University
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




// This program tests the ellipse geometry (i.e. derivative etc)
// nominally this program will use debugging code.

#include <ros/ros.h>
#include "cwru_opencv_common/ellipse_modeling.h"


int main(int argc, char** argv)
{
  
  cv::Size imageSize(100,100);
  cv::Size ellipseSize(50,25);
  double ellipseAngle(0);
  cv::Point2d ellipseCenter(50,50);
  cv::RotatedRect testEllipse(ellipseCenter, ellipseSize, ellipseAngle);

  cv::Size ellipseSize1(45,25);
  cv::RotatedRect testEllipse1(ellipseCenter, ellipseSize1, ellipseAngle);
  

  cv::Mat image = cv::Mat::zeros(imageSize,CV_32FC1);
  cv::Mat imageInv = cv::Mat::ones(imageSize,CV_8UC1)*255;
  
  cv::Mat image8U1 = cv::Mat::zeros(imageSize,CV_8UC1);

  cv::Mat image8U3 = cv::Mat::zeros(imageSize,CV_8UC3);  

  cv::Mat ellipseMatJac;
  cv::Mat ellipseMat = cv_ellipse::ellipse2Mat(testEllipse,ellipseMatJac);

  cv::ellipse(image8U1, testEllipse, cv::Scalar(255), 1);
  cv::ellipse(imageInv, testEllipse, cv::Scalar(0), -1);

    cv::Point2f vertices[4];
    testEllipse.points(vertices);
    for (int i = 0; i < 4; i++)
    cv::line(image8U1, vertices[i], vertices[(i+1)%4], cv::Scalar(255));

  cv::Mat vect(3, 1, CV_32FC1);
  float minVal(0);
  float maxVal(0);
  for (int i(0); i < imageSize.height; i++)
  {
  	for (int j(0); j < imageSize.width; j++)
  	{
  		vect.at<float> (0) = static_cast<float> (j);
        vect.at<float> (1) = static_cast<float> (i);
        vect.at<float> (2) = static_cast<float> (1.0);
        float pixVal(cv_ellipse::getResultsDerivative(vect, ellipseMat));

        image.at <float>(i, j) = pixVal;

        if (pixVal < minVal) minVal = pixVal;
        if (pixVal > maxVal) maxVal = pixVal;
  	}
  }
  
  cv::Rect subRect(testEllipse.boundingRect());

  ROS_INFO("Computing ellipse energy");

  cv::imshow("ellipse",imageInv);
  
  cv::waitKey(0);

  subRect -= cv::Point(5, 5);
  subRect += cv::Size(10, 10);
  ROS_INFO_STREAM(subRect);
  cv::Mat deriv;
  double energy(cv_ellipse::computeEllipseEnergy(subRect, testEllipse, imageInv,deriv));
  double energy1(cv_ellipse::computeEllipseEnergy(subRect, testEllipse1, imageInv,deriv));

  std::cout << deriv << std::endl;


  image += minVal;
  image *= (1/(maxVal-minVal));
 

  vect.at<float> (0) = static_cast<float> (50);
  vect.at<float> (1) = static_cast<float> (50);
  vect.at<float> (2) = static_cast<float> (1.0);

  double centerE(cv_ellipse::getResultsDerivative(vect,ellipseMat));
  ROS_INFO("center E: %f",centerE);

  ROS_INFO_STREAM(ellipseMat);
  ROS_INFO("minVal: %f",minVal);
  ROS_INFO("maxVal: %f",maxVal);

  ROS_INFO("energy: %f",energy);
  ROS_INFO("bad energy: %f",energy1);

  cv::imshow("ellipse",image8U1);
  
  cv::waitKey(0);
  
  cv::imshow("ellipse",image);
  
  cv::waitKey(0);

  cv::ellipse(image, testEllipse, cv::Scalar(1.0), 1);
  cv::imshow("ellipse",image);
  
  cv::waitKey(0);
  
  double angle(0.0);
  
  cv::Mat newImage = cv::Mat::zeros(imageSize,CV_8UC1);
  
  while(angle < 6.29)
  {
  	cv::Mat localMat1 = cv_ellipse::ellipsePointMat(testEllipse, angle);
    cv::Mat localMat2 = cv_ellipse::ellipsePointMatR(testEllipse, angle);
    cv::Point localPt = cv_ellipse::ellipsePointR(testEllipse, angle);
  	cv::Mat val1 = localMat1.t()*ellipseMat*localMat1;
  	cv::Mat val2 = localMat2.t()*ellipseMat*localMat2;
  	angle += 0.01;
  	newImage.at< unsigned char >(localPt.y, localPt.x) = 255;
  }

  cv::imshow("ellipse",newImage);
  cv::waitKey(0);
  ROS_INFO("Quiting the vesselness GPU node\n");
  return 0;
}