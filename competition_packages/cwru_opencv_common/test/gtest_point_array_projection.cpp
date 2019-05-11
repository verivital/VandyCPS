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


#include <gtest/gtest.h>
#include <vector>
#include <cwru_opencv_common/projective_geometry.h>

#include <iostream>
#include <random>


using namespace cv;


struct pointProjectionsTest : testing::Test
{
    // member information:
    Mat P;
    Mat G;
    Mat ptList;
    int w;
    int h;
    int n;
    // test object constructor
    pointProjectionsTest():
    P(3, 4, CV_64FC1),
    h(3), w(3), n(h*w)
    {
        ptList = Mat::zeros(3, n, CV_64FC1);
        // populate the projection Matrix:
    	P.setTo(0.0);
    	P.at<double>(0, 0) = 1000.0;
    	P.at<double>(1, 1) = 1000.0;
    	P.at<double>(0, 2) = 100.0;
    	P.at<double>(1, 2) = 100.0;
    	P.at<double>(2, 2) = 1.0;

      //create the SE(3) matrix. 
        G = Mat::eye(4, 4, CV_64FC1);
        G.at<double>(2, 3) = 100.0;
      
      double wD(10.0), hD(10.0), wO(-25.0), hO(-50.0);
      ptList.setTo(0);
      for (int ix(0); ix < w; ix++)
      {
          // h major ordering.
          for (int iy(0); iy < h; iy++)
          {
              ptList.at<double>(0, ix*h + iy) = wO + wD*ix;
              ptList.at<double>(1, ix*h + iy) = hO + hD*iy;
              ptList.at<double>(2, ix*h + iy) = 0.0;
          }
    }

    };
    ~pointProjectionsTest()
    {
    }
};


TEST_F(pointProjectionsTest, testSE3Jac)
{
    // Import image
    // transform the pointList (and get the JAC)
    Mat jacSE3;
    
    // project the ptList into RP3.
    Mat ptList_o(4, n, CV_64FC1);
    
    ptList_o.setTo(1.0);
    ptList.copyTo(ptList_o.rowRange(0,3));

    std::cout << "The original points are:\n";
    std::cout << ptList << std::endl;
    
    std::cout << "The SE(3) matrix is:\n";
    std::cout << G << std::endl;
  
    Mat ptList_c(cv_projective::transformPointsSE3(ptList_o, G, jacSE3));
    // output the results of the projection:
    std::cout << "The result of the transform is: " << std::endl;
    std::cout << ptList_c << std::endl;
    
    // output the jacobian matrix:
    std::cout << "The jacobian matrix is" << std::endl;
    std::cout << jacSE3 << std::endl;
    
    // verify the translational jacobian:
    
    Mat dG = Mat::zeros(12,1, CV_64FC1);

    // dx
    dG.at<double>(3, 0) = 18.0;

    // dy
    dG.at<double>(7, 0) = -13.0;

    // dz
    dG.at<double>(11, 0) = 6.0;

    Mat dPt(jacSE3*dG);

    std::cout << dPt << std::endl;

    ASSERT_TRUE(true);
	}

/* TEST_F(ellipseMatchingTest, testEllipseMatchingSmallResults)
{
    // Import image

    // 1. create an ellipse image:
    // create the image:
    Mat test_image(200, 200, CV_8UC1);
	test_image.setTo(0);

    // project the base circle:
    std::vector< std::vector<Point> > ptList;
    ptList.resize(1);
    ptList[0].clear();
    cv::Rect circleRect(cv_ellipse_num::projectCirclePoints(ptList[0], this->P, this->G, this->circleCenter, this->rad, 20));
	drawContours(test_image, ptList, 0, Scalar(255, 255, 255), -1, 8, noArray(), INT_MAX);

    double idealMatch(cv_ellipse_num::circleEnergy(test_image, this->P, this->G, this->circleCenter, this->rad, 20));

	std::default_random_engine generator;
    std::normal_distribution<double> distributionLin(0.0, 0.025);
    std::normal_distribution<double> distributionRot(0.0, 0.3);

    // make a for loop to test random perturbations.
    for (int ind(0); ind < 100000; ind++)
    {
        Mat G_bad = this->G.clone();
    	// create the perturbations:
        // Rotation perturbation:
		double xOff(distributionLin(generator));
		double yOff(distributionLin(generator));
		double zOff(distributionLin(generator));

		double qX(distributionLin(generator));
		double qY(distributionLin(generator));
		double qZ(distributionLin(generator));
		double ang(distributionRot(generator));

    	G_bad.at<double> (0, 3) = xOff;
    	G_bad.at<double> (1, 3) = yOff;
    	G_bad.at<double> (2, 3) = zOff;

    	cv_rot::Quaternion qua_rotation;
		qua_rotation.data[0] = cos(ang/2);
		qua_rotation.data[1] = sin(ang/2)*qX;
		qua_rotation.data[2] = sin(ang/2)*qY;
		qua_rotation.data[3] = sin(ang/2)*qZ;


		cv_rot::Quaternion normedQ(QuatNormalize(qua_rotation));

		Mat rot = cv_rot::QuatToMatrix(normedQ);

		rot.copyTo(G_bad.rowRange(0, 3).colRange(0, 3));

    	double perturbMatch(cv_ellipse_num::circleEnergy(test_image, this->P, G_bad, this->circleCenter, this->rad, 20));
    	ASSERT_TRUE(perturbMatch <= idealMatch);
	}
} */

// @TODO Make a unit test for identifying the optimal location of a circle.
// include a numerical based derivative estimation.


/* TEST_F(ellipseMatchingTest, testMultipleEllipseMatchingResults)
{
    // Import image
    // @TODO: finish this for a vector of ellipses
    // 1. create an ellipse image:
    // create the image:
    Mat test_image(200, 200, CV_8UC1);
	test_image.setTo(0);

    // project the base circle:
    std::vector< std::vector<Point> > ptList;
    ptList.resize(1);
    ptList[0].clear();
    cv::Rect circleRect(cv_ellipse_num::projectCirclePoints(ptList[0], this->P, this->G, this->circleCenter, this->rad, 20));
	drawContours(test_image, ptList, 0, Scalar(255, 255, 255), -1, 8, noArray(), INT_MAX);

    double idealMatch(cv_ellipse_num::circleEnergy(test_image, this->P, this->G, this->circleCenter, this->rad, 20));

	std::default_random_engine generator;
    std::normal_distribution<double> distributionLin(0.0, 0.025);
    std::normal_distribution<double> distributionRot(0.0, 0.3);

    // make a for loop to test random perturbations.
    for (int ind(0); ind < 1000; ind++)
    {
        Mat G_bad = this->G.clone();
    	// create the perturbations:
        // Rotation perturbation:
		double xOff(distributionLin(generator));
		double yOff(distributionLin(generator));
		double zOff(distributionLin(generator));

		double qX(distributionLin(generator));
		double qY(distributionLin(generator));
		double qZ(distributionLin(generator));
		double ang(distributionRot(generator));

    	G_bad.at<double> (0, 3) = xOff;
    	G_bad.at<double> (1, 3) = yOff;
    	G_bad.at<double> (2, 3) = zOff;

    	cv_rot::Quaternion qua_rotation;
		qua_rotation.data[0] = cos(ang/2);
		qua_rotation.data[1] = sin(ang/2)*qX;
		qua_rotation.data[2] = sin(ang/2)*qY;
		qua_rotation.data[3] = sin(ang/2)*qZ;


		cv_rot::Quaternion normedQ(QuatNormalize(qua_rotation));

		Mat rot = cv_rot::QuatToMatrix(normedQ);

		rot.copyTo(G_bad.rowRange(0, 3).colRange(0, 3));

    	double perturbMatch(cv_ellipse_num::circleEnergy(test_image, this->P, G_bad, this->circleCenter, this->rad, 20));
    	ASSERT_TRUE(perturbMatch <= idealMatch);
	}
} */


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
