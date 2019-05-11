

#include "cwru_opencv_common/opencv_geometry_2d.h"


namespace cv_2d{

using cv::Mat;
using cv::Point2d;
using cv::Moments;

double blobCenterDirections(const Mat & imageMask, Point2d* center, Point2d* major , Point2d* minor )
{
    if (imageMask.channels() != 1)
    {
        return -1;
    }

    Moments maskMom(moments(imageMask));


    // Identify the eigenvalues.
    cv::Matx<double , 2, 2> dir;

    dir(0,0) = maskMom.nu20;

    dir(0,1) = maskMom.nu11;
    dir(1,0) = maskMom.nu11;

    dir(1,1) = maskMom.nu02;

    Mat eigenVal, eigenVec;

    cv::eigen(Mat(dir), eigenVal, eigenVec);

    // Populate the center , and directions.
    if (center != NULL)
    {
        center->x = (maskMom.m10/maskMom.m00);
        center->y = (maskMom.m01/maskMom.m00);
    }
    //

    if ( major != NULL )
    {
        Point2d dirP(eigenVec.at<double>(0), eigenVec.at<double>(1));
        dirP *= 1/(norm(dirP));
        major-> x = dirP.x;
        major-> y = dirP.y;
    }

    if ( minor != NULL )
    {
        Point2d dirT(eigenVec.at<double>(2), eigenVec.at<double>(3));
        dirT *= 1/(norm(dirT));
        minor->x = dirT.x;
        minor->y = dirT.y;
    }
    if (abs(eigenVal.at<double>(1)) >  abs(eigenVal.at<double>(0)) )
    {
        return (eigenVal.at<double>(0)/eigenVal.at<double>(1));
    }
    else
    {
        return (eigenVal.at<double>(1)/eigenVal.at<double>(0));
    }
    // unreachable
}


};  // namespace cv_2d