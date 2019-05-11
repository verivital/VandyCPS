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
 * The functions declared in this file are meant to handle the projective geometry of the cameras
*/


#ifndef PROJECTIVE_GEOMETRY_H
#define PROJECTIVE_GEOMETRY_H


#include <ros/ros.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cwru_opencv_common/opencv_local.h"
#include <sensor_msgs/CameraInfo.h>
#include <string>


namespace cv_projective
{


class cameraProjectionMatrices
{
public:
    cameraProjectionMatrices(ros::NodeHandle&, const std::string& , const std::string&);

    cv::Mat getLeftProjectionMatrix() const;
    cv::Mat getRightProjectionMatrix() const;

private:
    void projectionSubscriptionCb(const sensor_msgs::CameraInfoConstPtr &, int);
    // left and right projective matrices
    bool stereo;
    cv::Mat P_l;
    cv::Mat P_r;

    // left and right subscribers
    ros::Subscriber subL;
    ros::Subscriber subR;
};

/** 
 * @brief reprojectPoint Projects a point in 3d coords to a point in a camera image.
 *
 * @param const cv::Point3d &point the point in 3d space.
 * @param const cv::Mat &P the projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (2 x 9).
 *
 * @return cv::Point2d the point in the image space.
 *
 * The reprojected point is assumed to be in some object frame o, which is converted to the camera frame through rvec and tvec. 
 */
cv::Point2d reprojectPoint(const cv::Point3d &point, const cv::Mat &P,
    const cv::Mat& rvec = cv::Mat(), const cv::Mat & = cv::Mat(), cv::OutputArray = cv::noArray());

/** 
 * @brief reprojectPoints Projects an array of point in 3d coords to an array of point in a camera image.
 *
 * @param const cv::InputArray spacialPoints the point list in 3d space.
 * @param const cv::OutputArray imagePoints the list of points in image space.
 * @param const cv::Mat &P the projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (2*n x 9).
 *
 * 
 * 
 * The reprojected points are assumed to be in some object frame o, which are converted to the camera frame through rvec and tvec. 
 * There is no return type.
 */
void reprojectPoints(cv::InputArray spacialPoints, cv::OutputArray imagePoints,
    const cv::Mat &P, const cv::Mat &, const cv::Mat &);

/** 
 * @brief reprojectPointsSE3 Projects an array of point in 3d coords to an array of point in a camera image.
 *
 * @param const cv::InputArray spacialPoints the point list in 3d space.
 * @param const cv::OutputArray imagePoints the list of points in image space.
 * @param const cv::Mat &P the projection Matrix (3x4).
 * @param const cv::Mat &g=cv::Mat() the SE(3) transformation Matrix (4x4).
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t G. (2*n x 6).
 *
 * The reprojected points are assumed to be in some object frame o, which are converted to the camera frame through G. 
 * There is no return type.
 */ 
void reprojectPointsSE3(cv::InputArray spacialPoints, cv::OutputArray imagePoints, const cv::Mat &P, const cv::Mat &G, cv::OutputArray = cv::noArray());

/** 
 * @brief reprojectPointStereo Projects a point in 3d coords to a stereo pair of point in a camera image.
 *
 * @param const cv::Point3d &point the point in 3d space.
 * @param const cv::Mat &P_l the left projection Matrix (3x4).
 * @param const cv::Mat &P_r the right projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (4 x 9).
 *
 * @return cv_local::stereoCorrespondence the point pair in the images.
 *
 * The reprojected point is assumed to be in some object frame o, which is converted to the camera frame through rvec and tvec. 
 */
cv_local::stereoCorrespondence reprojectPointStereo(const cv::Point3d &point,
    const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat());

/** 
 * @brief reprojectPointDerivativeStereo Projects a point in 3d coords to a stereo pair of point in a camera image.
 *
 * @param const cv::Point3d &point the point in 3d space.
 * @param const cv::Point3d &pointDeriv the point derivative in 3d space (unit vector).
 * @param const cv::Mat &P_l the left projection Matrix (3x4).
 * @param const cv::Mat &P_r the right projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (4 x 9).
 *
 * @return cv_local::stereoCorrespondence the tangent pair in the images.
 *
 * The reprojected point and derivative is assumed to be in some object frame o, which is converted to the camera frame through rvec and tvec. 
 */
cv_local::stereoCorrespondence reprojectPointDerivativeStereo(const cv::Point3d &point, const cv::Point3d &pointDeriv, 
    const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat());

/** 
 * @brief reprojectPointsStereo Projects an array of points in 3d coords to a vector of stereo pairs of points in a camera image.
 *
 * @param const cv::InputArray &points the point in 3d space.
 * @param const std::vector <cv_local::stereoCorrespondence> & stereoList the point list in image pair space.
 * @param const cv::Mat &P_l the left projection Matrix (3x4).
 * @param const cv::Mat &P_r the right projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (4 x 9).
 *
 * The reprojected points are assumed to be in some object frame o, which is converted to the camera frame through rvec and tvec. 
 */
void reprojectPointsStereo(cv::InputArray points, std::vector < cv_local::stereoCorrespondence > & ,const cv::Mat &P_l, const cv::Mat &P_r,const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat() );

/** 
 * @brief reprojectPointsStereo Projects an array of points in 3d coords to a vector of stereo pairs of points in a camera image.
 *
 * @param const cv::InputArray &points the point in 3d space.
 * @param const cv::OutputArray & points_left the point list in the left image space.
 * @param const cv::OutputArray & points_right the point list in the right image space.
 * @param const cv::Mat &P_l the left projection Matrix (3x4).
 * @param const cv::Mat &P_r the right projection Matrix (3x4).
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (4 x 9).
 *
 * The reprojected points are assumed to be in some object frame o, which is converted to the camera frame through rvec and tvec. 
 */
void reprojectPointsStereo(cv::InputArray points, cv::OutputArray points_left, cv::OutputArray points_right  ,const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(),  const cv::Mat & = cv::Mat() );


/** 
 * @brief reprojectPointTangent Projects a point in 3d coords to a stereo pair of point in a camera image.
 *
 * @param const cv::Point3d &point the point in 3d space.
 * @param const cv::Point3d &pointDeriv the point derivative in 3d space (unit vector).
 * @param const cv::Mat &P the left projection Matrix (3x4).
 * @TODO implement the rvec, and tvec parameters and functionality.
 * @param const cv::Mat &rvec=cv::Mat() the Rodrigues rotation vector (3x1).
 * @param const cv::Mat &tvec=cv::Mat() the translation vector (3x1). 
 * @param const cv::OutputArray &jac=cv::Mat() the jacobian matrix w.r.t the point, rvec, and tvec. (4 x 9).
 *
 * @return cv::Point2d the tangent vector in the image.
 *
 */
cv::Point2d reprojectPointTangent(const cv::Point3d &point, const cv::Point3d &pointDeriv, const cv::Mat & P);

/** 
 * @brief deprojectStereoTangent:  Deprojects a camera tangent pair in a stereo image to a 3d vector in the camera 3d frame.
 *
 * @param const cv_local::stereoCorrespondence& imagePoints the point pair in the images.
 * @param const cv_local::stereoCorrespondence& imagePointsTangent the tangent pair in the images.
 * @param const cv::Mat &P_l the left projection matrix.
 * @param const cv::Mat &P_r the right projection matrix.
 *
 * @return cv::Point3d the tangent vector in the camera 3d space.
 *
 */
cv::Point3d deprojectStereoTangent(const cv_local::stereoCorrespondence &imagePoints,
    const cv_local::stereoCorrespondence &imagePointsTangent, const cv::Mat & P_l, const cv::Mat &P_r);


/** 
 * @brief deprojectStereoPoint:  Deprojects a camera point pair in a stereo image to a 3d point in the camera 3d frame.
 *
 * @param const cv_local::stereoCorrespondence& imagePoints the point pair in the images.
 * @param const cv::Mat &P_l the left projection matrix.
 * @param const cv::Mat &P_r the right projection matrix.
 *
 * @return cv::Point3d the point in the camera 3d space.
 *
 */
cv::Point3d deprojectStereoPoint(const cv_local::stereoCorrespondence&, const cv::Mat &P_l, const cv::Mat &P_r);


/** 
 * @brief deprojectStereoPoints:  Deprojects an array of camera point pairs in a stereo image to a 3d vector in the camera 3d frame.
 *
 * @param const std::vector< cv_local::stereoCorrespondence > & imagePoints the point pair in the images.
 * @param const cv::OutputArray & spacialPoints The 3d points in the camera frame.
 * @param const cv::Mat &P_l the left projection matrix.
 * @param const cv::Mat &P_r the right projection matrix.
 *
 * @return cv::Point3d the tangent vector in the camera 3d space.
 *
 */
void deprojectStereoPoints(const std::vector < cv_local::stereoCorrespondence > &inputArray,
    cv::OutputArray outputPointArray, const cv::Mat& P_l , const cv::Mat P_r);

/**
 * @brief transformJacobian computes the gMat and an associated jacobian given rvec and the tvec.
 * 
 
 * @param const cv::Mat &rvect the rodrigues rotation in 3d space.
 * @param const cv::Mat &tvect the translation in 3d space.
 * @param cv::Mat &gMat the translation in 3d space.
 * @param cv::OutputArray &jac=cv::noArray() the jacobian matrix of the gMat w.r.t rvect and tvect (6 x 6).
 * 
 */
void transformJacobian(const cv::Mat &rvect ,const cv::Mat & tvect, cv::Mat & trans , cv::OutputArray trans_jac = cv::noArray());

/**
 * @brief computeRvecTvec computes the rvec and the tvec, given the transform matrix G.
 * 
 * @param const cv::Mat &gMat the translation in 3d space.
 * @param cv::Mat &rvect the rodrigues rotation in 3d space.
 * @param cv::Mat &tvect the translation in 3d space.
 * @param const cv::OutputArray &jac=cv::noArray() the jacobian matrix of rvect and tvect w.r.t the gMat. (6 x 6).
 * 
 * The output of this function is the rvec and tvec computed from gMat.
 */
void computeRvecTvec(const cv::Mat & gMat, cv::Mat & rvect, cv::Mat & tvect, cv::OutputArray trans_jac = cv::noArray());

/**
 * @brief transformPoints transforms a list of points using rvec and tvec along with an optional jacobian output.
 * 
 * @param const cv::Mat &points the point list (in 3d).
 * @param const cv::Mat &rvect the rodrigues rotation in 3d space.
 * @param const cv::Mat &tvect the translation in 3d space.
 * @param cv::OutputArray &jac=cv::noArray() the jacobian matrix of the result points w.r.t rvect and tvect (6 x 6).
 * 
 * @return cv::Mat the transformed points
 *
 */
cv::Mat transformPoints(const cv::Mat &points, const cv::Mat& rvec,
    const cv::Mat &tvec, cv::OutputArray jac = cv::noArray());


/**
 * @brief transformPointsSE3: transform a 4xn matrix of RP3 points using a 4x4 mat G (option jacobian output)
 *  
 * @param const cv::Mat &points the point list (in 3d).
 * @param const cv::Mat &gMat the 4x4 transformation matrix.
 * @param cv::OutputArray &jac=cv::noArray() the jacobian matrix of the result points w.r.t gMat (3*n x 6).
 * 
 * @return cv::Mat the transformed points
 *
 */
cv::Mat transformPointsSE3(const cv::Mat &,const cv::Mat &, cv::OutputArray = cv::noArray());



};


#endif

