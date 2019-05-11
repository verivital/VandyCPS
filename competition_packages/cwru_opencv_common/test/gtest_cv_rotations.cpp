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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <vector>
#include <cwru_opencv_common/cv_rotations.h>
#include <iostream>
#include <random>


struct rotationMatchingTest : testing::Test
{
  // member information:
  std::default_random_engine generator;

  std::normal_distribution<double> distributionLin;

  rotationMatchingTest(): generator(),
  distributionLin(0.0, 0.75)
  {}

  ~rotationMatchingTest()
  {}
};


// This test validates that quaternions can be converted to rotations and back without error introduction.
TEST_F(rotationMatchingTest, testRotations)
{
  // make a for loop to test random perturbations.
  for (int ind(0); ind < 10000000; ind++)
  {
    // randomly generate a quaternion
    double qX(distributionLin(generator));
    double qY(distributionLin(generator));
    double qZ(distributionLin(generator));
    double qU(distributionLin(generator));

    cv_rot::Quaternion qua_rotation;
    qua_rotation.data[0] = abs(qU);
    qua_rotation.data[1] = qX;
    qua_rotation.data[2] = qY;
    qua_rotation.data[3] = qZ;


    // normalize the quaternion and convert it to a 3x3 rotation matrix
    cv_rot::Quaternion Qn1(QuatNormalize(qua_rotation));

    cv::Mat rot1 = cv_rot::QuatToMatrix(Qn1);

    // then back to a quaternion
    cv_rot::Quaternion Qn2(cv_rot::MatrixToQuat(rot1));

    // and back to a rotation matrix again.
    cv::Mat rot2 = cv_rot::QuatToMatrix(Qn2);

    // error bound.
    double eb(0.00001);

    // compute the error:
    double qE(QuatError(Qn1, Qn2));

    ASSERT_NEAR(qE, 0.0, eb);

    for (int i(0); i < 9; i++)
    {
      int row(i%3);
      int col(i/3);
      ASSERT_NEAR(rot1.at<double>(row, col), rot2.at<double>(row, col), eb);
    }
  }
}

// @TODO(BIOCUBED) Itendify the test purpose
/* TEST_F(rotationMatchingTest, testDeriv)
{
  // make a for loop to test random perturbations.
  for (int ind(0); ind < 1000; ind++)
  {
    double qX(distributionLin(generator));
    double qY(distributionLin(generator));
    double qZ(distributionLin(generator));
    double qU(distributionLin(generator));


    cv_rot::Quaternion qua_rotation;
    qua_rotation.data[0] = abs(qU);
    qua_rotation.data[1] = qX;
    qua_rotation.data[2] = qY;
    qua_rotation.data[3] = qZ;


    cv_rot::Quaternion Qn1(QuatNormalize(qua_rotation));


    cv::Mat jac;
    cv::Mat rot1 = cv_rot::QuatToMatrix(Qn1, jac);

    std::cout << rot1 << std::endl;
    double eb(0.00001);
    double del(0.00000001);

    for (int i(0); i < 4; i++)
    {
        std::cout << Qn1.data[i] << std::endl;
    }

    for (int i(0); i < 4; i++)
    {
      cv_rot::Quaternion QnL(Qn1);
      cv_rot::Quaternion QnH(Qn1);

      QnL.data[i] -= del;
      QnH.data[i] += del;

      cv_rot::Quaternion QnLp(QuatNormalize(QnL));
      cv_rot::Quaternion QnHp(QuatNormalize(QnH));

      double delL(Qn1.data[i]-QnLp.data[i]);
      double delH(QnHp.data[i]-Qn1.data[i]);


      cv::Mat rotH = cv_rot::QuatToMatrix(QnH);
      cv::Mat rotL = cv_rot::QuatToMatrix(QnL);

      cv::Mat delR = (rotH-rotL)/(delL+delH);
      std::cout << jac << std::endl;
      std::cout << delR << std::endl;
      for (int k(0); k < 9; k++)
      {
        int row(k%3);
        int col(k/3);
        ASSERT_NEAR(delR.at<double>(col, row), jac.at<double>(k, i), eb);
      }
    }
  }
} */

// @TODO(BIOCUBED) Itendify the test purpose
/* TEST_F(rotationMatchingTest, testConv)
{
  for (int ind(0); ind < 10; ind++)
  {
    double qX1(distributionLin(generator));
    double qY1(distributionLin(generator));
    double qZ1(distributionLin(generator));
    double qU1(distributionLin(generator));


    double qX2(distributionLin(generator));
    double qY2(distributionLin(generator));
    double qZ2(distributionLin(generator));
    double qU2(distributionLin(generator));

    cv_rot::Quaternion qua_start;
    qua_start.data[0] = abs(qU1);
    qua_start.data[1] = qX1;
    qua_start.data[2] = qY1;
    qua_start.data[3] = qZ1;


    cv_rot::Quaternion qua_goal;
    qua_goal.data[0] = abs(qU2);
    qua_goal.data[1] = qX2;
    qua_goal.data[2] = qY2;
    qua_goal.data[3] = qZ2;


    cv_rot::Quaternion Qg(QuatNormalize(qua_goal));
    cv_rot::Quaternion Qs(QuatNormalize(qua_start));

    // instead of solving for R directly, solve using a random point:

    cv::Mat pointArray(3, 2, CV_64FC1);
    pointArray.at<double> (0, 0) = distributionLin(generator);
    pointArray.at<double> (1, 0) = distributionLin(generator);
    pointArray.at<double> (2, 0) = distributionLin(generator);
    pointArray.at<double> (0, 1) = distributionLin(generator);
    pointArray.at<double> (1, 1) = distributionLin(generator);
    pointArray.at<double> (2, 1) = distributionLin(generator);

    cv::Mat jac;
    cv::Mat r_g = cv_rot::QuatToMatrix(Qg);

    cv::Mat r_s = cv_rot::QuatToMatrix(Qs, jac);

    cv::Mat p_g = r_g*pointArray;

    double eb(0.00001);
    double del(0.05);
    std::cout << jac << std::endl;
    for ( int i(0); i < 10; i++)
    {
      cv::Mat dCdr_s, dCdp;
      cv::matMulDeriv(r_s, pointArray, dCdr_s, dCdp);

      cv::Mat p_s = r_s*pointArray;

      cv::Mat jac_q = dCdr_s * jac;

      cv::Mat e_p = p_g - p_s;

      std::cout << jac_q.size() << std::endl;

      double error = norm(p_s-p_g);

      std::cout << error << std::endl;

      if (error < eb) break;

      cv::Mat dq = jac_q.inv(cv::DECOMP_SVD)*e_p;

      for (int j(0); j < 4; j++)
      {
          Qs.data[i] += dq.at<double> (j, 0)*del;
      }
      Qs = QuatNormalize(Qs);
      r_s = cv_rot::QuatToMatrix(Qs, jac);
    }
    ASSERT_TRUE(true);
  }
} */


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
