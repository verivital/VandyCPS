/* 
 * projective_geometry.h
 * Copyright 2014 Russell Jackson
 * All rights reserved.
 */

/*
 * The functions declared in this file are meant to handle the projective geometry of the camera image space and P^2 to p^3.
 */



#include "cwru_opencv_common/projective_geometry.h"

#include "cwru_opencv_common/ellipse_modeling.h"

using namespace cv;
using namespace cv_local;

namespace cv_ellipse
{


Point2d ellipsePoint(const RotatedRect & input, double angle)
{
    Point2d newPoint(0,0);

    newPoint.x = input.center.x+(input.size.width*cos(input.angle*3.14159/180)*cos(angle)+input.size.height*sin(input.angle*3.14159/180)*cos(angle))/2;
    newPoint.y = input.center.y+(input.size.height*cos(input.angle*3.14159/180)*sin(angle)+input.size.width*sin(input.angle*3.14159/180)*sin(angle))/2;

    return newPoint;
}

Mat ellipsePointMat(const RotatedRect & input, double angle)
{
    Point2d newPoint(ellipsePoint(input, angle));

    Mat vect(3, 1, CV_32FC1);
    vect.at<float>(0) = static_cast<float> (newPoint.x);
    vect.at<float>(1) = static_cast<float> (newPoint.y);
    vect.at<float>(2) = static_cast<float> (1.0);

    return vect;
}

cv::Point ellipsePointR(const cv::RotatedRect & input, double angle)
{
    Point2d newPoint(ellipsePoint(input, angle));

    Point newPoint_( nearbyint(newPoint.x),  nearbyint(newPoint.y));

    return newPoint_;
}


cv::Mat ellipsePointMatR(const cv::RotatedRect & input, double angle)
{
    Point newPoint(ellipsePointR(input, angle));

    Mat vect(3, 1, CV_32FC1);
    vect.at<float>(0) = static_cast<float> (newPoint.x);
    vect.at<float>(1) = static_cast<float> (newPoint.y);
    vect.at<float>(2) = static_cast<float> (1.0);

    return vect;
}

Mat ellipse2Mat(RotatedRect input, cv::OutputArray matJac)
{
    Mat conicMat(3, 3, CV_32FC1);

    // generate the first stage of parameters.
    const double dadw(0.5);
    const double dbdh(0.5);
    const double dthetadangle(3.14159265359/180.0);
    double a = input.size.width/2;
    double b = input.size.height/2;
    double xc = input.center.x;
    double yc = input.center.y;
    double theta = (double) (input.angle)*3.14159265359/180;

    double stheta = sin(theta);
    double ctheta = cos(theta);

    // compute the Polynomial coefficients.
    double A = a*a*stheta*stheta+b*b*ctheta*ctheta;
    double B = 2*(b*b-a*a)*stheta*ctheta;
    double C = a*a*ctheta*ctheta+b*b*stheta*stheta;
    double D = -2*A*xc-B*yc;
    double E = -B*xc-2*C*yc;
    double F = A*xc*xc+B*xc*yc+C*yc*yc-a*a*b*b;

    if ( matJac.needed() )
    {
        matJac.create(6, 5, CV_32FC1);

        Mat matJac_ =  matJac.getMat();

        // derivative of A.
        double dAda(2.0*a*stheta*stheta);
        double dAdb(2.0*b*ctheta*ctheta);
        double dAdtheta(-B);

        matJac_.at< float > (0, 0) = static_cast<float> (dAda*dadw);  // dA/dWidth
        matJac_.at< float > (0, 1) = static_cast<float> (dAdb*dbdh);  // dA/dheight
        matJac_.at< float > (0, 2) = static_cast<float> (-B*dthetadangle);  // dA/dangle
        matJac_.at< float > (0, 3) = 0.0;  // dA/dcx
        matJac_.at< float > (0, 4) = 0.0;  // dA/dcy

        // derivative of B
        double dBda(-4*a*stheta*ctheta);
        double dBdb(4*b*stheta*ctheta);
        double dBdtheta(2*A-2*C);

        matJac_.at< float > (1, 0) = static_cast<float> (dBda*dadw);  // dB/dWidth
        matJac_.at< float > (1, 1) = static_cast<float> (dBdb*dbdh);  // dB/dheight
        matJac_.at< float > (1, 2) = static_cast<float> (dBdtheta*dthetadangle);  // dB/dangle
        matJac_.at< float > (1, 3) = 0.0;  // dB/dcx
        matJac_.at< float > (1, 4) = 0.0;  // dB/dcy

        // derivative of C
        double dCda(2*a*ctheta*ctheta);
        double dCdb(2*b*stheta*stheta);
        double dCdtheta(B);
        matJac_.at< float > (2, 0) = static_cast<float> (dCda*dadw);  // dC/dWidth
        matJac_.at< float > (2, 1) = static_cast<float> (dCdb*dbdh);  // dC/dheight
        matJac_.at< float > (2, 2) = static_cast<float> (dCdtheta*dthetadangle);  //dC/dangle
        matJac_.at< float > (2, 3) = 0.0;  // dB/dcx
        matJac_.at< float > (2, 4) = 0.0;  // dB/dcy

        // derivative of D
        matJac_.at< float > (3, 0) = static_cast<float> ((-2*xc*dAda-yc*dBda)*dadw);  // dD/dWidth
        matJac_.at< float > (3, 1) = static_cast<float> ((-2*xc*dAdb-yc*dBdb)*dbdh);  // dD/dheight
        matJac_.at< float > (3, 2) = static_cast<float> ((-2*xc*dAdtheta-yc*dBdtheta)*dthetadangle);  // dD/dangle
        matJac_.at< float > (3, 3) = static_cast<float> (-2*A);  // dD/dcx
        matJac_.at< float > (3, 4) = static_cast<float> (-B);  // dD/dcy

        // derivative of E
        matJac_.at< float > (4, 0) = static_cast<float> ((-xc*dBda-2*yc*dCda)*dadw);  // dE/dWidth
        matJac_.at< float > (4, 1) = static_cast<float> ((-xc*dBdb-2*yc*dCdb)*dbdh);  // dE/dheight
        matJac_.at< float > (4, 2) = static_cast<float> ((-xc*dBdtheta-2*yc*dCdtheta)*dthetadangle);  // dE/dangle
        matJac_.at< float > (4, 3) = static_cast<float> (-B);  // dE/dcx
        matJac_.at< float > (4, 4) = static_cast<float> (-2*C);  // dE/dcy

        // derivative of F
        matJac_.at< float > (5, 0) = static_cast<float> (
            (xc*xc*dAda+xc*yc*dBda+yc*yc*dCda-2*a*b*b)*dadw);  // dF/dWidth
        matJac_.at< float > (5, 1) = static_cast<float> (
            (xc*xc*dAdb+xc*yc*dBdb+yc*yc*dCdb-2*a*a*b)*dbdh);  // dF/dheight
        matJac_.at< float > (5, 2) = static_cast<float> (
            (xc*xc*dAdtheta+xc*yc*dBdtheta+yc*yc*dCdtheta)*dthetadangle);  // dF/dangle
        matJac_.at< float > (5, 3) = static_cast<float> (2*xc*A+yc*B);  // dF/dcx
        matJac_.at< float > (5, 4) = static_cast<float> (2*yc*C+xc*B);  // dF/dcy
    }

    conicMat.at<float>(0, 0) = A;
    conicMat.at<float>(1, 1) = C;
    conicMat.at<float>(2, 2) = F;

    conicMat.at<float>(0, 1) = B/2;
    conicMat.at<float>(1, 0) = B/2;

    conicMat.at<float>(2, 0) = D/2;
    conicMat.at<float>(0, 2) = D/2;

    conicMat.at<float>(2, 1) = E/2;
    conicMat.at<float>(1, 2) = E/2;

    // use a test point inside the circle to ensure that the inside is negative:
    Mat testPt(3, 1, CV_32FC1);
    testPt.at<float>(0) = static_cast<float> (input.center.x);
    testPt.at<float>(1) = static_cast<float> (input.center.y);
    testPt.at<float>(2) = 1.0;

    Mat result = testPt.t()*conicMat*testPt;

    double scaleFactor = abs(1/result.at<float>(0));

    conicMat *= scaleFactor;
    if (result.at<float>(0) > 0)
    {
        conicMat *= -1.0;
        if ( matJac.needed() )
        {
            Mat matJac_ =  matJac.getMat();
            matJac_ *= -1.0*scaleFactor;
        }
    }
    return conicMat;
}


Mat findEllipseRotTransMat(RotatedRect ellipse, double radius, Mat intrinsics)
{
    Mat circleMat = Mat::eye(3, 3, CV_64FC1);
    circleMat.at<double>(2, 2) = -radius*radius;

    std::cout << std::endl;
    std::cout << circleMat;
    std::cout << std::endl;

    Mat ellipseMat = ellipse2Mat(ellipse);

    std::cout << std::endl;
    std::cout << ellipseMat;
    std::cout << std::endl;

    Mat rotTrans;

    std::cout << std::endl;
    std::cout << intrinsics;
    std::cout << std::endl;

    rotTrans = intrinsics.inv()*ellipseMat*circleMat.inv();

    Mat r0 = rotTrans.col(0);
    Mat r1 = rotTrans.col(1);

    std::cout << std::endl;
    std::cout << rotTrans;
    std::cout << std::endl;

    double n0 = norm(r0);
    double n1 = norm(r1);

    double nMean = sqrt(n0*n1);

    rotTrans *= 1/(nMean);

    return rotTrans;
}


double computeEllipseEnergy(const Rect &subRect, const RotatedRect& ellipseRect, const cv::Mat &imageGrey, cv::OutputArray ellipseEnergyDerivative)
{
    // compute image gradiant
   
    Mat subImage = imageGrey(subRect);
   
   
    Mat subImage_x, subImage_y;
    Scharr(subImage, subImage_x, CV_32F, 1, 0);
    Scharr(subImage, subImage_y, CV_32F, 0, 1);
    

    // The total energy of the ellipse is the Image brightness at  X,Y offset by the image cuttoff.
    // mulitplied by the result of the energy functional
    // splus the image gradient
    int totalSize(subImage.rows*subImage.cols);


    RotatedRect offsetEllipse(ellipseRect);
    offsetEllipse.center -= Point2f(subRect.x, subRect.y);

    // the energy is computed.

    Mat thresh;
    double I_c(threshold(subImage, thresh, 127, 255, THRESH_BINARY_INV+THRESH_OTSU));


    Mat imageMask(subImage.size(), CV_8UC1);
    Mat imageMaskOutline(subImage.size(), CV_8UC1);
    imageMask = Scalar(0);
    imageMaskOutline = Scalar(0);
  
    
    ellipse(imageMaskOutline, offsetEllipse, Scalar(255), 1);  // line width is variable (5)
    ellipse(imageMask, offsetEllipse, Scalar(1), -1);  // filled in ellipse.

    Mat deriv;

    if (ellipseEnergyDerivative.needed())
    {
        ellipseEnergyDerivative.create(1, 6, CV_32FC1);
        deriv =  ellipseEnergyDerivative.getMat();
    }
    cv::imshow("ellipse",imageMaskOutline);
    cv::waitKey(0);


    // compute the offset vector.
    Size subSize;
    Point imgOffset;
    subImage.locateROI(subSize, imgOffset);
    Mat jacobian;
    Mat ellipseMat = ellipse2Mat(ellipseRect, jacobian);
    Mat vect(3, 1, CV_32FC1);



    
    float totalVal = 0.0;
    for (int i = 0; i < totalSize; i++)
    {
        int x((i % subRect.width) + imgOffset.x);
        int y((i / (subRect.width)) + imgOffset.y);
        vect.at<float> (0) = static_cast<float> (x);
        vect.at<float> (1) = static_cast<float> (y);
        vect.at<float> (2) = static_cast<float> (1.0);
        

        if (imageMask.at < unsigned char > (i) > 0)
        {
        
            Mat derivEllipse;

           
            float value =  getResultsDerivative(vect, ellipseMat, derivEllipse);
            float imagePixel =  static_cast<float> (imageMask.at < unsigned char > (i)) - I_c;
            totalVal += value*imagePixel;

            if (ellipseEnergyDerivative.needed())
            {
                deriv += derivEllipse*imagePixel;
            }
        }
        if (imageMaskOutline.at < unsigned char > (i) > 0)
        {

            Mat imageGrad(3, 1, CV_32FC1);

            imageGrad.at<float>(1) = subImage_x.at< float > (i)*1/255;
            imageGrad.at<float>(0) = subImage_y.at< float > (i)*-1/255;
            imageGrad.at<float>(2) = 0.0f;

            
            Mat inter = ellipseMat*vect*2;
            double value = imageGrad.dot(inter);
            
          

        
            totalVal += value;

            
            if (ellipseEnergyDerivative.needed())
            {
                Mat localDeriv(1,6,CV_32FC1);

                localDeriv.at< float >(0)  = static_cast< float > ( vect.at < float > (0)*imageGrad.at < float > (0));  // dvdA
                localDeriv.at< float >(1)  = static_cast< float > ( vect.at < float > (1)*imageGrad.at < float> (0)*0.5
                    + vect.at < float > (0)*imageGrad.at < float> (1)*0.5);  // dvdB
                localDeriv.at< float >(2)  = static_cast< float > ( vect.at < float> (1)*vect.at < float> (1));  // dvDC 
                localDeriv.at< float >(3)  = static_cast< float > ( imageGrad.at < float> (0) * 0.5 );                      // dvDD
                localDeriv.at< float >(4)  = static_cast< float > ( imageGrad.at < float> (1) *0.5 );                      // dvdE
                localDeriv.at< float >(5)  = static_cast< float > ( 0.0 );                             // dvdF

                deriv += localDeriv*2.0;
            }
           
        }
    }
    return totalVal;
}


double ellipseEnergyFunctional(Mat & image, const RotatedRect &ellipseRect)
{
	
}

void generatepixelValues(Mat & blankImage, const RotatedRect& ellipseRect, Mat& jacobian)
{

}

void generatepixelValues(Mat & blankImage, Mat & ellipseMat , Mat& jacobian)
{
    
}

float getResultsDerivative(const Mat& vect,const Mat & ellipseMat, cv::OutputArray derivativeEllipse)
{
    Mat results = vect.t()*ellipseMat*vect;

    float output = results.at<float> (0);

    // generate the derivative.

    if (derivativeEllipse.needed())
    {
        derivativeEllipse.create(1,6,CV_32FC1);

        Mat derivativeEllipse_ =  derivativeEllipse.getMat();

        derivativeEllipse_.at< float >(0)  = static_cast< float > ( vect.at < float> (0)*vect.at < float> (0));  // dvdA
        derivativeEllipse_.at< float >(1)  = static_cast< float > ( vect.at < float> (1)*vect.at < float> (0));  // dvdB
        derivativeEllipse_.at< float >(2)  = static_cast< float > ( vect.at < float> (1)*vect.at < float> (1));  // dvDC 
        derivativeEllipse_.at< float >(3)  = static_cast< float > ( vect.at < float> (0) );
        derivativeEllipse_.at< float >(4)  = static_cast< float > ( vect.at < float> (1) );
        derivativeEllipse_.at< float >(5)  = static_cast< float > ( 1.0 );
    }

	// compute value;
	
    return output;
}


};  // namespace cv_ellipse