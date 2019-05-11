/*
 * color_model.cpp
 * Copyright 2016  Russell Jackson, All Rights Reserved
 */
 
/*
 * This file defines a class of color tracking model.
 * The model assumes that a color distribution is a gaussian in RGB space. 
 * The segmentation is simply a multivariate gaussian probability (the final result is normalized using the l_infinity norm).
 * A mask is used to generate the initial fit.
 */



#include <ros/ros.h>

#include <cwru_opencv_common/color_model.hpp>

using cv::Mat;


namespace cv_color_model{

ColorModel::ColorModel(const ColorModel& source_){
        
    for (int i(0); i < 3; i++)
    {
        this->colorMean(i) = source_.colorMean(i);
        for (int j(0); j < 3; j++)
        {
            this->colorVariance(i,j) = source_.colorVariance(i,j);
        }
    }
}

ColorModel::ColorModel(const Mat & sourceImage, const cv::Mat& maskImage){
    if (maskImage.type() == CV_8UC1)
    {
        binaryMaskInit(sourceImage,maskImage);
    }
    else if (maskImage.type() == CV_32FC1)
    {
        floatMaskInit(sourceImage,maskImage);
    }
    else
    {
        // Error of some sort
    }

}


void ColorModel::binaryMaskInit(const Mat & sourceImage, const cv::Mat& maskImage){
    // To obtain a full covariance matrix, use the calcCovar
    int nonZero(countNonZero(maskImage));
    Mat samplesMat(3,nonZero,CV_32FC1);
    // For now, a really slow painful form is implemented.
    int nonZeroIndex(0); 
    for ( int i(0); i < maskImage.rows*maskImage.cols; i++)
    {
        if (maskImage.at<uchar>(i) > 0 )
        {
            cv::Vec3b pixelVal(sourceImage.at< cv::Vec3b >(i));
            
            samplesMat.at< float >(0, nonZeroIndex) = pixelVal[0];
            samplesMat.at< float >(1, nonZeroIndex) = pixelVal[1];
            samplesMat.at< float >(2, nonZeroIndex) = pixelVal[2];
            
            nonZeroIndex++;
        }
    }
    Mat covar,mean;
    calcCovarMatrix(samplesMat,covar,  mean, CV_COVAR_NORMAL+CV_COVAR_COLS+CV_COVAR_SCALE,CV_32F);
    // Generate the full data.
    // Assign it to the mean and covariance.
    for (int i(0); i < 3; i++)
    {
        this->colorMean(i) = mean.at<float>(i);
        for (int j(0); j < 3; j++)
        {
            colorVariance(i,j) = covar.at<float>(i,j);;
        }
    }
}

void ColorModel::floatMaskInit(const Mat & sourceImage, const cv::Mat& maskImage){
    // To obtain a full covariance matrix, use the calcCovar
    int nonZero(countNonZero(maskImage));
    Mat samplesMat(3,nonZero,CV_32FC1);
    // For now, a really slow painful form is implemented.
    int nonZeroIndex(0); 
    cv::Scalar maskSum(sum(maskImage));
    for ( int i(0); i < maskImage.rows*maskImage.cols; i++)
    {
        if (maskImage.at<float>(i) > 0 )
        {
            cv::Vec3b pixelVal(sourceImage.at< cv::Vec3b >(i));
            
            samplesMat.at< float >(0, nonZeroIndex) = static_cast<float> (pixelVal[0])*maskImage.at<float>(i)/maskSum[0];
            samplesMat.at< float >(1, nonZeroIndex) = static_cast<float> (pixelVal[1])*maskImage.at<float>(i)/maskSum[0];
            samplesMat.at< float >(2, nonZeroIndex) = static_cast<float> (pixelVal[2])*maskImage.at<float>(i)/maskSum[0];
            nonZeroIndex++;
        }
    }
    float nonZeroNum(nonZeroIndex);
    Mat covar, mean;
    calcCovarMatrix(samplesMat,covar,  mean, CV_COVAR_NORMAL+CV_COVAR_COLS+CV_COVAR_SCALE,CV_32F);
    // Generate the full data.
    // Assign it to the mean and covariance.
    for (int i(0); i < 3; i++ )
    {
        
        colorMean(i) = mean.at<float>(i)*nonZeroNum;
        for (int j(0); j < 3; j++)
        {
            colorVariance(i,j) = covar.at<float>(i,j)*nonZeroNum;
        }
    }
}

cv::Mat ColorModel::segmentImage(const Mat &inputImage){
 
    Mat result(inputImage.size(), CV_32FC1);
    float maxResult = -1;
    cv::Matx<float, 3,1> tempMat;

    // Pre-compute variables that will be needed inside the for loop.
    cv::Matx< float, 3, 3 > inverse = colorVariance.inv(cv::DECOMP_CHOLESKY);
    float  det(cv::determinant(colorVariance));
    float denom(sqrt(3.14*3.14*3.14*8*det));
    float frac(1/denom);
    



 for (int i(0); i < inputImage.rows*inputImage.cols; i++)
 {
     // Fill this line in with the covariant probability density function.
     // Available: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    cv::Vec3b  tempVec(inputImage.at<cv::Vec3b>(i));

    // if (i == 0) ROS_INFO("Data 1)");


    tempMat(0) = static_cast<float> (tempVec[0]);
    tempMat(1) = static_cast<float> (tempVec[1]);
    tempMat(2) = static_cast<float> (tempVec[2]);

    cv::Matx<float, 3, 1> diff(tempMat-colorMean);

    // if (i == 0) ROS_INFO("Data 1)");


    cv::Matx<float, 1, 1> exponMatx(diff.t()*inverse*diff*-0.5);


    // if (i == 0) ROS_INFO("Data 1)");

    float pixelVal(frac*exp(exponMatx(0)));

    // if (i == 0) ROS_INFO("Data 1)");

    result.at<float>(i) = pixelVal;

    // if (i == 0) ROS_INFO("Data 1)");

    if (pixelVal > maxResult)
    {
        maxResult = pixelVal;
    }

 }
 //normalize result
 result = result*(1/maxResult);
 return result;
}

void ColorModel::printModelInfo()
{
    ROS_INFO("Color mean is < %f , %f , % f >", colorMean(0), colorMean(1), colorMean(2));
    ROS_INFO_STREAM("Color covariance is " << std::endl <<  colorVariance << std::endl);
}


// Begin of the HSV model

ColorModelHSV::ColorModelHSV(const ColorModelHSV& source_){
        
    for (int i(0); i < 3; i++)
    {
        this->colorMean(i) = source_.colorMean(i);

        for (int j(0); j < 3; j++)
        {
            this->colorVariance(i,j) = source_.colorVariance(i,j);
        }
    }
}

ColorModelHSV::ColorModelHSV(const Mat & sourceImage, const cv::Mat& maskImage, int meanBias){
    if (maskImage.type() == CV_8UC1)
    {
        binaryMaskInit(sourceImage,maskImage, meanBias);
    }
    else if (maskImage.type() == CV_32FC1)
    {
        floatMaskInit(sourceImage,maskImage, meanBias);
    }
    else
    {
        // Error of some sort
    }

}

float hueMap(float colorIn, float mean){
    float error(colorIn-mean);
    float errorP(colorIn-180-mean);
    float errorN(colorIn-(mean-180));
    
    float errorT;
    float errorFin;
    if(abs(error) < abs(errorP))
        errorT = error;
    else errorT = errorP;

    if(abs(errorT) < abs(errorN)) errorFin = errorT;
    else errorFin = errorN;

    return (errorFin+mean);

}

void ColorModelHSV::binaryMaskInit(const Mat & sourceImage, const cv::Mat& maskImage, const int meanBias){
    // To obtain a full covariance matrix, use the calcCovar
    int nonZero(countNonZero(maskImage));
    Mat samplesMat(3, nonZero, CV_32FC1);
    // For now, a really slow painful form is implemented.

    int nonZeroIndex(0);
    for ( int i(0); i < maskImage.rows*maskImage.cols; i++)
    {
        if (maskImage.at<uchar>(i) > 0 )
        {
            cv::Vec3b pixelVal(sourceImage.at< cv::Vec3b >(i));

            samplesMat.at< float >(0, nonZeroIndex) = hueMap(pixelVal[0], meanBias);
            samplesMat.at< float >(1, nonZeroIndex) = pixelVal[1];
            samplesMat.at< float >(2, nonZeroIndex) = pixelVal[2];

            nonZeroIndex++;
        }
    }
    Mat covar,mean;
    calcCovarMatrix(samplesMat,covar,  mean, CV_COVAR_NORMAL+CV_COVAR_COLS+CV_COVAR_SCALE,CV_32F);
    // Generate the full data.
    // Assign it to the mean and covariance.
    for (int i(0); i < 3; i++ )
    {
        
        colorMean(i) = mean.at<float>(i);
        for (int j(0); j < 3; j++)
        {
            colorVariance(i,j) = covar.at<float>(i,j);
        }
    }
}

void ColorModelHSV::floatMaskInit(const Mat & sourceImage, const cv::Mat& maskImage, int meanBias){
    // To obtain a full covariance matrix, use the calcCovar
    int nonZero(countNonZero(maskImage));
    Mat samplesMat(3,nonZero,CV_32FC1);
    // For now, a really slow painful form is implemented.
    int nonZeroIndex(0); 
    cv::Scalar maskSum(sum(maskImage));
    for ( int i(0); i < maskImage.rows*maskImage.cols; i++)
    {
        if (maskImage.at<float>(i) > 0 )
        {
            cv::Vec3b pixelVal(sourceImage.at< cv::Vec3b >(i));
            
            samplesMat.at< float >(0, nonZeroIndex) = hueMap(pixelVal[0], meanBias)*maskImage.at<float>(i)/maskSum[0];
            samplesMat.at< float >(1, nonZeroIndex) = static_cast<float> (pixelVal[1])*maskImage.at<float>(i)/maskSum[0];
            samplesMat.at< float >(2, nonZeroIndex) = static_cast<float> (pixelVal[2])*maskImage.at<float>(i)/maskSum[0];
            nonZeroIndex++;
        }
    }
    float nonZeroNum(nonZeroIndex);
    Mat covar, mean;
    calcCovarMatrix(samplesMat,covar,  mean, CV_COVAR_NORMAL+CV_COVAR_COLS+CV_COVAR_SCALE, CV_32F);

    // Generate the full data.
    // Assign it to the mean and covariance.
    for (int i(0); i < 3; i++ )
    {
        
        colorMean(i) = mean.at<float>(i)*nonZeroNum;
        for (int j(0); j < 3; j++)
        {
            colorVariance(i,j) = covar.at<float>(i,j)*nonZeroNum;
        }
    }
}

cv::Mat ColorModelHSV::segmentImage(const Mat &inputImage){
 
    Mat result(inputImage.size(), CV_32FC1);
    float maxResult = -1;
    cv::Matx<float, 3,1> tempMat;

    // Pre-compute variables that will be needed inside the for loop.
    cv::Matx< float, 3, 3 > inverse = colorVariance.inv(cv::DECOMP_CHOLESKY);
    float  det(cv::determinant(colorVariance));
    float denom(sqrt(3.14*3.14*3.14*8*det));
    float frac(1/denom);
    
    for (int i(0); i < inputImage.rows*inputImage.cols; i++)
    {
        // Fill this line in with the covariant probability density function.
        // Available: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
        cv::Vec3b  tempVec(inputImage.at<cv::Vec3b>(i));

        // if (i == 0) ROS_INFO("Data 1)");


        tempMat(0) = hueMap(tempVec[0],colorMean(0));
        tempMat(1) = static_cast<float> (tempVec[1]);
        tempMat(2) = static_cast<float> (tempVec[2]);

        cv::Matx<float, 3, 1> diff(tempMat-colorMean);

        // if (i == 0) ROS_INFO("Data 1)");


        cv::Matx<float, 1, 1> exponMatx(diff.t()*inverse*diff*-0.5);


        // if (i == 0) ROS_INFO("Data 1)");

        float pixelVal(frac*exp(exponMatx(0)));

        // if (i == 0) ROS_INFO("Data 1)");

        result.at<float>(i) = pixelVal;

        // if (i == 0) ROS_INFO("Data 1)");

        if (pixelVal > maxResult)
        {
            maxResult = pixelVal;
        }

    }
 //normalize result
 result = result*(1/maxResult);
 return result;
}

void ColorModelHSV::printModelInfo()
{
    ROS_INFO("Color mean is < %f , %f , % f >", colorMean(0), colorMean(1), colorMean(2));
    ROS_INFO_STREAM("Color covariance is " << std::endl <<  colorVariance << std::endl);
}



}; //namespace cv_color_model
