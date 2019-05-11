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

/*This file relies on the following external libraries:

OpenCV (2.3.1)
Eigen  (?????)
*/

//  This file generates the function prototypes that will become common to multiple projects.
//  The library will rely on opencv to define the basic math, but will be expanded to include
//  other material, ie screws, twists, D-H params, and Quaternions.

#ifndef CV_ROTATIONS_H
#define CV_ROTATIONS_H


#include <opencv2/opencv.hpp>
/* @todo
 * merge this library with tf::quaternian.
 */
//  transformation Quaternion

namespace cv_rot
{

typedef cv::Vec<double, 7> transQuatTransform;

template <class T>  double computePointArrayError(cv::Point3_<T>  newPoint,
    int &index, cv::Point3_<T>* inputArray, int arrayCount)
{
    double errorOutput(0.0);
    for(int i(0); i < arrayCount; i++)
	{
        errorOutput += norm(newPoint-inputArray[i]);
    }
    index = (index+1)%arrayCount;
    inputArray[index] = newPoint;

    return errorOutput/(static_cast<double> (arrayCount));
};


struct Quaternion{

	///* data[i] = qi and v = (q1,q2,q3) *///
	double data[4];
	double operator[](int index)
	{
		return data[index];
	}
	Quaternion() {}

	Quaternion(const Quaternion& q_)
	{
		for (int i(0); i < 4; i++)
		{
			data[i] = q_.data[i];
		}
	}

	Quaternion log() const
	{	
		///* log(q) =  log(q0)* + v/||v||*arccos( q0/||v|| )*///
		Quaternion Q;
		if (data[0] < 0)
		{			
			for (unsigned int i = 0; i < 4; i++)
			{
				Q.data[i] = -data[i];
			}			
		}
		else
		{
			for (unsigned int i = 0; i < 4; i++)
			{
				Q.data[i] = data[i];
			}			
		}
		double a;
		if (Q.data[0]/length() > 1 || Q.data[0]/length() < -1)
		{
			a = 0;
		}
		else
		{
			a = (double)acos(Q.data[0]/length());
		}
		double vlength = sqrt(pow(Q.data[1],2) + pow(Q.data[2],2) + pow(Q.data[3],2));
		//double sina = (double)sin(a);
		Quaternion ret;
		


		ret.data[0] = std::log(Q.data[0]);
		ret.data[1] = Q.data[1]/vlength*a;
		ret.data[2] = Q.data[2]/vlength*a;
		ret.data[3] = Q.data[3]/vlength*a;


		//if (sina > 0)
		//{
		//	ret.data[1] = a*data[0]/sina;
		//	ret.data[2] = a*data[1]/sina;
		//	ret.data[3] = a*data[2]/sina;
		//} else {
		//	ret.data[1] = ret.data[2] = ret.data[3] = 0;
		//}
		return ret;
	}
	Quaternion exp() const
	{
		///* exp(q) =  exp(q0)*(cos||v|| + v/||v||*sin||v||)*///
		double vlength = sqrt(pow(data[1],2) + pow(data[2],2) + pow(data[3],2));
		double a = (double)vlength;
		double sina = (double)sin(a);
		double cosa = (double)cos(a);
		Quaternion ret;

		double exp_term = std::exp(data[0]);
		ret.data[0] = exp_term*cosa;
		ret.data[1] = exp_term*sina/vlength*data[1];
		ret.data[2] = exp_term*sina/vlength*data[2];
		ret.data[3] = exp_term*sina/vlength*data[3];
		//if (a > 0)
		//{
		//	ret.data[1] = sina * data[1] / a;
		//	ret.data[2] = sina * data[2] / a;
		//	ret.data[3] = sina * data[3] / a;
		//} else {
		//	ret.data[1] = ret.data[2] = ret.data[3] = 0;
		//}
		return ret;
	}
	void conjugate()
	{ 
		data[1] = -data[1]; 
		data[2] = -data[2]; 
		data[3] = -data[3]; 
	}
	double length() const
	{
		return sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); 
	}
	double length_squared() const
	{
		return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); 
	}
	const Quaternion operator /(double scale) const
	{
		Quaternion ret;
		for( unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] /= scale;
		}
		return ret; 
	}
	const Quaternion &operator /= (double scale)			
	{
		for( unsigned int i = 0; i < 4; i++)
		{
			data[i] /= scale;
		}
		return *this; 
	}
	Quaternion invert() const
	{
		Quaternion ret;
		ret.data[0] = data[0]/length_squared();
		ret.data[1] = -data[1]/length_squared();
		ret.data[2] = -data[2]/length_squared();
		ret.data[3] = -data[3]/length_squared();
	 
		return ret;
	}
	Quaternion norm() const
	{
		Quaternion ret;
		ret.data[0] = data[0]/length();
		ret.data[1] = data[1]/length();
		ret.data[2] = data[2]/length();
		ret.data[3] = data[3]/length();
		return ret;
	}
	Quaternion neg() const
	{
		Quaternion ret;
		ret.data[0] = -data[0];
		ret.data[1] = -data[1];
		ret.data[2] = -data[2];
		ret.data[3] = -data[3];
	 
		return ret;
	}
	const Quaternion operator *(double scale) const
	{ 
		Quaternion ret;
		for( unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] *= scale;
		}
		return ret;
	}
	const Quaternion operator *(const Quaternion &q) const	
	{	
		Quaternion ret;
		ret.data[0] = data[0]*q.data[0] - data[1]*q.data[1] - data[2]*q.data[2] - data[3]*q.data[3];
		ret.data[1] = data[2]*q.data[3] - data[3]*q.data[2] + data[0]*q.data[1] + data[1]*q.data[0];
		ret.data[2] = data[3]*q.data[1] - data[1]*q.data[3] + data[0]*q.data[2] + data[2]*q.data[0];
		ret.data[3] = data[1]*q.data[2] - data[2]*q.data[1] + data[0]*q.data[3] + data[3]*q.data[0];
		return  ret;
	}

	const Quaternion &operator +=(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] += q.data[i];
		}
		return *this; 
	}
	const Quaternion &operator -=(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] -= q.data[i];
		}
		return *this; 
	}
	Quaternion &operator =(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] = q.data[i];
		} 
		return *this; 
	}
	const Quaternion operator +(const Quaternion &q) const	
	{ 
		Quaternion ret;
		for (unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] = data[i] + q.data[i];
		}
		return ret;
	}
	const Quaternion operator -(const Quaternion &q) const	
	{ 
		Quaternion ret;
		for (unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] = data[i] - q.data[i];
		}
		return ret;
	}

};


cv::Mat Rot_xFun(double dbInput);//unit--degree
cv::Mat Rot_yFun(double dbInput);//unit--degree
cv::Mat Rot_zFun(double dbInput);//unit--degree

cv::Mat Ginv(cv::Mat matInput);

Quaternion MatrixToQuat(cv::Mat& matInput); //The matrix should either be a 3x3 or a 4x4. 
	//The matrix will be confirmed to be in SO(3)


cv::Mat QuatToMatrix(Quaternion, cv::OutputArray = cv::noArray());

Quaternion EulerXYZToQuat(cv::Point3d inputEXYZ);
cv::Point3d MatrixToEulerXYZ(cv::Mat inputMat);
cv::Mat EulerXYZToMatrix(cv::Point3d inputEXYZ);


Quaternion QuatNormalize(Quaternion inputQ);


cv::Mat InvRotMatrix(cv::Mat inputM);

cv::Vec3d QuatRotVec(Quaternion, cv::Vec3d);

cv::Point3d QuatToEulerXYZ(Quaternion inputQ);

double QuatError(Quaternion inputQ1,Quaternion inputQ2);

};

#endif


