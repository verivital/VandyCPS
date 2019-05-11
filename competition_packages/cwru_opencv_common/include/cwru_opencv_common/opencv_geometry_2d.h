/* 
 * opencv_geometry_2d.h:
 * 
 * Created By Russell Jackson
 * 10/19/2012
 */


/**
 * @brief functions and models required for 2d image geometric identification
 *
 */


#ifndef OPENCV_GEOMETRY_2D_H
#define OPENCV_GEOMETRY_2D_H


#include <opencv2/opencv.hpp>


namespace cv_2d{


double blobCenterDirections(const cv::Mat & , cv::Point2d* , cv::Point2d* = NULL , cv::Point2d* = NULL );



double iterate_circle(const cv:: Mat &);



}; // namespace cv_2d

#endif