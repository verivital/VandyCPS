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


/**
 * @brief functions and models required for 3d object detecion and tracking
 *
 */
#ifndef OPENCV_GEOMETRY_3D_H
#define OPENCV_GEOMETRY_3D_H


#include <opencv2/opencv.hpp>


namespace cv_3d
{

  /**
   * @brief This is a sphere in 3D space.
   */
  struct sphere
  {

    cv::Point3d center; /**< The center of the sphere. */

    double radius;      /**< The radius of the sphere. */

    /**
     * @brief The constructor.
     * @param center_ The center of the sphere.
     * @param radius_ The radius of the sphere.
     */
    sphere(const cv::Point3d &center_=cv::Point3d(0.0,0.0,0.0) , double radius_ = 1.0 ):
      center(center_),
      radius(radius_)
    {
    }

  };

  /**
   * @brief This is a cylinder in 3D space.
   */
  struct cylinder
  {

    cv::Point3d center; /**< The center of the cylinder. */

    double theta;       /**< The azimuth angle in the polar coordinates. */

    double phi;         /**< The zenith angle in the polar coordinates. */

    double height;      /**< The height of the cylinder */

    double radius;      /**< The radius of the cylinder */

    /**
     * @brief The constructor.
     * @param center_ The center of the cylinder.
     * @param theta_  The azimuth angle in the polar coordinates.
     * @param phi_    The zenith angle in the polar coordinates.
     * @param height_ The height of the cylinder.
     * @param radius_ The radius of the cylinder.
     */
    cylinder(const cv::Point3d& center_ = cv::Point3d(0.0, 0.0, 0.0), double theta_ = 0.0,
             double phi_ = 0.0, double height_ = 0.0, double radius_ = 0.0):
      center(center_),
      theta(theta_),
      phi(phi_),
      height(height_),
      radius(radius_)
    {
    }
  };

  /**
   * Optimize position of the sphere against a segmented image of the sphere.
   * TODO: Does not have a definition. Delete?
   *
   * @param segmentedImage    The segmented image.
   * @param sphereIn          The sphere to be optimized.
   * @param projectionLeft    The projection matrix of the left camera.
   * @param projectionRight   The projection matrix of the right camera.
   */
  void iterateSphere_3d(const cv::Mat &segmentedImage , sphere &sphereIn , const cv::Mat& , const cv::Mat&);

  /**
   * @brief Draw a sphere on the image.
   *
   * @param inputImage The image which the sphere will be drawn on.
   * @param sphereIn   The sphere to be drawn.
   * @param projection The camera projection matrix.
   * @param centerPt   A floating point 3-tuple (x, y, r), where (x, y) is the center of the sphere in the image
   *                   in pixels, and r is the radius of the rendered sphere in pixels.
   * @param jacobian   A 2-by-3 matrix that maps changes of position of the sphere relative to the camera
   *                   to the changes of position the rendered sphere in the image.
   * @param color      Optional color of the drawn sphere. Default is 8-bit white.
   * @return The minimal bounding box containing the sphere.
   */
  cv::Rect renderSphere(cv::Mat &inputImage, const sphere &sphereIn, const cv::Mat &projection,
    cv::OutputArray centerPt = cv::noArray(), cv::OutputArray jacobian = cv::noArray(),
    const cv::Scalar &color = cv::Scalar(255, 255, 255));

  /**
   * @brief Draw a cylinder on the image.
   *
   * @param  inputImage The image which the cylinder will be drawn on.
   * @param  cylinderIn The cylinder to be drawn on the image.
   * @param  projection The camera projection matrix.
   * @param  tips       A floating point 6-tuple (x1, y1, r1, x2, y2, r2), where (x1, y1) and (x2, y2) are the center
   *                    of the two ends of the rendered cylinder in pixels, r1 and r2 are the radius of the two ends
   *                    in pixels.
   * @param  jacobian   A 4-by-5 matrix that maps the changes in the configuration of the cylinder (x, y, z, theta, phi)
   *                    to the changes of the positions of the two ends of the rendered cylinder.
   * @param  color      Optional color of the drawn cylinder. Default is 8-bit white.
   * @return The minimal bounding box containing the cylinder.
   */
  cv::RotatedRect renderCylinder(cv::Mat &inputImage, const cylinder &cylinderIn, const cv::Mat &projection,
    cv::OutputArray tips = cv::noArray(), cv::OutputArray jacobian = cv::noArray(),
    const cv::Scalar& color = cv::Scalar(255, 255, 255));

  /**
   * @brief Optimize the location of the sphere such that it fits the blob on the segment image.
   *
   * @param  sphereIn        The sphere whose position will be optimized.
   * @param  segmentedImage  The segmented image.
   * @param  projectionLeft  The projection matrix of the left camera.
   * @param  projectionRight The projection matrix of the right camera.
   * @param  kWidth          Padding of bounding box. Net padding is 2 * kWidth
   * @param  kVar            The variance of the Gaussian used to blurred the rendered image for convolution.
   * @param  displayPause    Pause to display intermediary results.
   * @return                 Distance between the before and the after spheres.
   */
  double optimizeSphere(sphere &sphereIn, const cv::Mat &segmentedImage, const cv::Mat &projectionLeft,
    const cv::Mat &projectionRight , int kWidth, double kVar, bool displayPause = false);

  /**
   * @brief Optimize the pose of the cylinder such that it fits the blob on the segmented image.
   * @param  cylinderIn      The cylinder whose configuration will be optimized.
   * @param  segmentedImage  The segmented image.
   * @param  projectionLeft  The projection matrix of the left camera.
   * @param  projectionRight The projection matrix of the right camera.
   * @param  kWidth          Padding of bounding box. Net padding is 2 * kWidth
   * @param  kVar            The variance of the Gaussian used to blurred the rendered image for convolution.
   * @param  displayPause    Pause to display intermediary results.
   * @return                 Distance between the before and the after cylinders.
   */
  double optimizeCylinder(cylinder &cylinderIn, const cv::Mat &segmentedImage, const cv::Mat &projectionLeft,
    const cv::Mat &projectionRight , int kWidth, double kVar, bool displayPause= false);

  /**
   * @brief Compute rectangular coordinates of an R^3 unit vector in rectangular coordinates.
   *
   * @param theta     The azimuth angle.
   * @param phi       The zenith angle.
   * @param jacobian  How the vector changes with theta and phi.
   * @return          The unit vector rectangular coordinates.
   */
  cv::Point3d computeNormalFromSpherical(double theta, double phi, cv::OutputArray jacobian = cv::noArray());

  /**
   * @brief Compute spherical coordinates of an R^3 unit vector in spherical coordinates.
   *
   * @param unitVector  An R^3 unit vector in rectangular coordinates.
   * @return            The unit vector in polar coordinates (azimuth and zenith).
   */
  cv::Point2d computeSphericalFromNormal(cv::Point3d unitVector);
};

#endif
