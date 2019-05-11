/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015 Russell Jackson
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/

/**
 * @file The functions declared in this file are meant to handle the projective geometry of the camera image space and P^2 to p^3.
 */



#include <cwru_opencv_common/projective_geometry.h>
#include <string>

using namespace cv;
using namespace cv_local;

namespace cv_projective
{

cameraProjectionMatrices::cameraProjectionMatrices(ros::NodeHandle& nh, const std::string &leftCameraTopic, const std::string &rightCameraTopic):
    stereo(false),
    P_l(3,4,CV_64FC1),
    P_r(3,4,CV_64FC1)
{
    // initialize the subsrciber callbacks such that the left callback will update P_l and the right callback will update P_r.
    subL = nh.subscribe<sensor_msgs::CameraInfo>(leftCameraTopic, 1, boost::bind(&cameraProjectionMatrices::projectionSubscriptionCb, this, _1, 0));
    subR = nh.subscribe<sensor_msgs::CameraInfo>(rightCameraTopic, 1, boost::bind(&cameraProjectionMatrices::projectionSubscriptionCb, this, _1, 1));
    
}

void cameraProjectionMatrices::projectionSubscriptionCb(const sensor_msgs::CameraInfoConstPtr &input, int lr) 
{
	Mat temp;
	if(lr == 0)
	{
		temp = P_l;
	}
    else
    {
        temp = P_r;
	}
	
	for (int index(0); index < 12; index++)
	{
        temp.at<double>(index) = input->P[index];
	}
}

cv::Mat cameraProjectionMatrices::getLeftProjectionMatrix() const
{
	return P_l.clone();
}

cv::Mat cameraProjectionMatrices::getRightProjectionMatrix() const
{
	return P_r.clone();
}

cv::Point2d reprojectPoint(const cv::Point3d &point, const cv::Mat &P,const cv::Mat& rvec, const cv::Mat &tvec, cv::OutputArray jac)
{


    Mat prjPoints(4,1,CV_64FC1);
    Mat results(3,1,CV_64FC1);
    cv::Point2d output;

    Mat ptMat(4,1,CV_64FC1);
    ptMat.at<double>(0,0) = point.x;
    ptMat.at<double>(1,0) = point.y;
    ptMat.at<double>(2,0) = point.z;
    ptMat.at<double>(3,0) = 1.0;

    Mat transDeriv;

    if(rvec.total()==3 && tvec.total()==3)
    {
        if(jac.needed())
        {
            prjPoints = transformPoints(ptMat,rvec,tvec,transDeriv);
        }
        else
        {
            prjPoints = transformPoints(ptMat,rvec,tvec);
        }
    }
    else
    {
        prjPoints = ptMat;
    }

    results = P*prjPoints;
    output.x = results.at<double>(0,0)/results.at<double>(2,0);
    output.y = results.at<double>(1,0)/results.at<double>(2,0);

    if(jac.needed())
    {
        Mat derivPt =Mat::zeros(2,3,CV_64FC1);
        derivPt.at<double>(0,0) = 1.0;
        derivPt.at<double>(0,2) = -results.at<double>(0,0)*1.0/(results.at<double>(2,0)*results.at<double>(2,0));
        derivPt.at<double>(1,1) = 1.0;
        derivPt.at<double>(1,2) = -results.at<double>(1,0)*1.0/(results.at<double>(2,0)*results.at<double>(2,0));

        Mat dABdA,dABdB;
        matMulDeriv(P,prjPoints,dABdA,dABdB);
        Mat prjDeriv = dABdB.rowRange(0,3).colRange(0,3);
        Mat prjDerivD;
        prjDeriv.convertTo(prjDerivD,CV_64FC1);
        if(rvec.total()==3 && tvec.total()==3)
        {

            jac.create(2,9,CV_64FC1);
            Mat jacMat = jac.getMat();

            Mat fullDeriv =  derivPt*prjDeriv*transDeriv;
            fullDeriv.copyTo(jacMat);
        }
        else
        {
            //ROS_INFO("No rvec needed for Jac");
            //ROS_INFO_STREAM(derivPt);
            //ROS_INFO_STREAM(prjDerivD);
            jac.create(2,3,CV_64FC1);
            Mat jacMat = jac.getMat();
            Mat finalJac = derivPt*prjDerivD;
            finalJac.copyTo(jacMat);
        }

    }
    return output;
}


void reprojectPointsSE3(cv::InputArray _spacialPoints, cv::OutputArray _imagePoints, const cv::Mat &P, const cv::Mat &G, cv::OutputArray jac)
{
    // verify the correct form of the spacial points and that they have the correct dimensions


    Mat spacialPoints = _spacialPoints.getMat();

    // is one dimension 3 (or 4)?
    bool tuple3((spacialPoints.type() == CV_32FC3 && (spacialPoints.rows == 1 || spacialPoints.cols == 3) && (spacialPoints.rows == 3 || spacialPoints.cols == 1))); 

    /** @todo Add the CV_Asserts and necessary checks for the format of the input array. */
   	// reformat as needed.

    Mat spacialPointsA(Mat(spacialPoints.t()).reshape(1).t());

    // Resize the spatial point so that they are members of RP3
    Mat spacialPointsRP3(4, spacialPointsA.cols, CV_64FC1);

    // scope the temporary variables that are only used to project R3 to RP3
    {
        Mat spacialPointsA64;
        spacialPointsA.convertTo(spacialPointsA64,CV_64F);
        spacialPointsRP3.setTo(1.0);
        Mat subMat(spacialPointsRP3.rowRange(0, 3));
        spacialPointsA64.copyTo(subMat);
    }


    

    // convert the points from image frame into the camera frame.
    // i.e. 4 x n to 4 x n
    Mat transJac, cameraPoints;
    if (jac.needed())
    {
        // if transJac is needed, it will be 3 x n x 12
        cameraPoints = transformPointsSE3(spacialPointsRP3, G, transJac);
    }
    else
    {
        cameraPoints = transformPointsSE3(spacialPointsRP3, G);
    }


    // project the points into the image frame.
    Mat results(P*cameraPoints);

    int pointCount(cameraPoints.cols);

    _imagePoints.create(2, pointCount, CV_64FC1);
    Mat imagePoints = _imagePoints.getMat();

    // initialize the jacobian if it is needed.
    // The jacobian will rely on the SE(3) component of the transform.
    Mat derivPt;
    if (jac.needed())
    {
        derivPt = Mat::zeros(2*pointCount,3*pointCount,CV_64FC1);
    }

    // normalize the point in image space.
    for (int ind(0); ind < pointCount; ind++)
    {
        imagePoints.at<double>(0,ind) = results.at<double>(0,ind)/results.at<double>(2,ind);
        imagePoints.at<double>(1,ind) = results.at<double>(1,ind)/results.at<double>(2,ind);

        // if the Jacobian is required, add to it\
        //The jac is going to be n*2 x 3*n (using the transform)
        if (jac.needed())
        {
            // dx dR0
            derivPt.at<double>(ind*2, ind*3) = 1.0;

            // dx dR2
            derivPt.at<double>(ind*2, ind*3+2) = 
            -results.at<double>(0,0)*1.0/(results.at<double>(2,0)*results.at<double>(2,0));
            
            // dy dR1
            derivPt.at<double>(ind*2+1, ind*3+1) = 1.0;

            // dy dR2
            derivPt.at<double>(ind*2+1, ind*3+2) = 
            -results.at<double>(1,0)*1.0/(results.at<double>(2,0)*results.at<double>(2,0));
        }
    }

    if (jac.needed())
    {         
        Mat jacProd = derivPt*transJac;
        
        jac.create(2*pointCount, 12, CV_64FC1);
        Mat jacOut = jac.getMat();

        jacProd.copyTo(jacOut);
    }
            // @todo finish this.
        //Mat dABdA,dABdB;
        //matMulDeriv(P,prjPoints,dABdA,dABdB);
        //Mat prjDeriv = dABdB.rowRange(0,3).colRange(0,3);
        //Mat prjDerivD;
        //prjDeriv.convertTo(prjDerivD,CV_64FC1);
      
        /* {
            //ROS_INFO("No rvec needed for Jac");
            //ROS_INFO_STREAM(derivPt);
            //ROS_INFO_STREAM(prjDerivD);
            jac.create(2,3,CV_64FC1);
            Mat jacMat = jac.getMat();
            Mat finalJac = derivPt*prjDerivD;
            finalJac.copyTo(jacMat);
        }
        */
    }


// make this valid for multiple points.
void reprojectPoints(InputArray pointsIn, OutputArray pointsOut,const cv::Mat &P,const cv::Mat& rvec, const cv::Mat &tvec)
{

    Mat pointsInMat = pointsIn.getMat();
    CV_Assert(pointsInMat.depth() == CV_64F || pointsInMat.depth() == CV_32F);
    CV_Assert(pointsInMat.channels() == 3);
    CV_Assert(pointsInMat.rows > 0);

    Mat pointsInMatT;

    int n = pointsInMat.cols;

    pointsInMat.reshape(1,n).convertTo(pointsInMatT,CV_32F);


    Mat pointsInMatH_ = Mat::ones(4,n,CV_64FC1);

    Mat pointsInMat_ = pointsInMatT.t();

    pointsInMat_.copyTo(pointsInMatH_.rowRange(0,3));


    Mat prjPoints;


    if(rvec.total()==3 && tvec.total()==3)
    {
        // @todo Fix this
        prjPoints = transformPoints(pointsInMatH_,rvec,tvec); 
    }
    else
    {
        prjPoints = pointsInMatH_;
    }
    Mat results = P*prjPoints;

    pointsOut.create(2,n,CV_64FC1);

    Mat pointsOut_ = pointsOut.getMat();

    for(int index = 0; index < n; index++)
    {

        pointsOut_.at<double>(0,index) = results.at<double>(0,index)/results.at<double>(2,index);
        pointsOut_.at<double>(1,index) = results.at<double>(1,index)/results.at<double>(2,index);
    }
    return;
}


stereoCorrespondence reprojectPointStereo(const Point3d &point, const Mat &P_l, const Mat &P_r,const cv::Mat&rvec,const cv::Mat&tvec)
{
        stereoCorrespondence output;
        Mat stP[2];
        stP[0]= P_l;
        stP[1]= P_r;
        for(int lr = 0; lr < 2; lr++){
            output[lr] = reprojectPoint(point,stP[lr],rvec,tvec);
        }
        return output;
}


void reprojectPointsStereo(cv::InputArray points, cv::OutputArray points_left, cv::OutputArray points_right  ,const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & rvec,  const cv::Mat & tvec )
{

    reprojectPoints(points,points_left,P_l,rvec,tvec);
    reprojectPoints(points,points_right,P_r,rvec,tvec);
    return;
}


void reprojectPointsStereo(cv::InputArray points, std::vector < cv_local::stereoCorrespondence > &outputPoints  ,const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & rvec,  const cv::Mat & tvec )
{


    Mat points_left_,points_right_;


    reprojectPoints(points,points_left_,P_l,rvec,tvec);
    reprojectPoints(points,points_right_,P_r,rvec,tvec);

    Mat points_ = points.getMat();

   
    int n = points_.cols;

    for(int index = 0; index < n; index++)
    {
        outputPoints[index][0].x = points_left_.at<double>(0,index);
        outputPoints[index][0].y = points_left_.at<double>(1,index);

        outputPoints[index][1].x = points_right_.at<double>(0,index);
        outputPoints[index][1].y = points_right_.at<double>(1,index);
    }

    return;
}



cv_local::stereoCorrespondence reprojectPointDerivativeStereo(const cv::Point3d &point, const cv::Point3d &pointDeriv, 
    const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & rvec,  const cv::Mat & tvec)
{
    stereoCorrespondence tempDeriv;
    stereoCorrespondence tempOutput;
    Mat ptDMat(4, 2, CV_64FC1);
    Mat dResults(3, 2, CV_64FC1);
    Mat stP[2];

    stP[0]= P_l;
    stP[1]= P_r;


    ptDMat.at<double>(0, 0) = point.x;
    ptDMat.at<double>(1, 0) = point.y;
    ptDMat.at<double>(2, 0) = point.z;
    ptDMat.at<double>(3, 0) = 1.0;  // This is a point so it has a value of 1 at the 4th part. (projective geometry)
    ptDMat.at<double>(0, 1) = pointDeriv.x;
    ptDMat.at<double>(1, 1) = pointDeriv.y;
    ptDMat.at<double>(2, 1) = pointDeriv.z;
    ptDMat.at<double>(3, 1) = 0.0;  // This is a vector and not a point...


    for(int lr = 0; lr < 2; lr++){
        dResults = stP[lr]*ptDMat;
        tempDeriv[lr].x = (dResults.at<double>(2, 0)*dResults.at<double>(0, 1)-dResults.at<double>(0, 0)*dResults.at<double>(2, 1))/(dResults.at<double>(2, 0)*dResults.at<double>(2, 0));
        tempDeriv[lr].y = (dResults.at<double>(2, 0)*dResults.at<double>(1, 1)-dResults.at<double>(1, 0)*dResults.at<double>(2, 1))/(dResults.at<double>(2, 0)*dResults.at<double>(2, 0));
    }
    return tempDeriv;
}

Point2d reprojectPointTangent(const cv::Point3d &point, const cv::Point3d &pointDeriv, const Mat & P)
{
    Point2d tempDeriv;
    Mat ptDMat(4, 2, CV_64FC1);
    Mat dResults(3, 2, CV_64FC1);

    // populate the tangent matrix.
    ptDMat.at<double>(0, 0) = point.x;
    ptDMat.at<double>(1, 0) = point.y;
    ptDMat.at<double>(2, 0) = point.z;
    ptDMat.at<double>(3, 0) = 1.0;  // This is a point so it has a value of 1 at the 4th part. (projective geometry)
    ptDMat.at<double>(0, 1) = pointDeriv.x;
    ptDMat.at<double>(1, 1) = pointDeriv.y;
    ptDMat.at<double>(2, 1) = pointDeriv.z;
    ptDMat.at<double>(3, 1) = 0.0;  // This is a vector and not a point...

    dResults = P*ptDMat;
    tempDeriv.x = (dResults.at<double>(2, 0)*dResults.at<double>(0, 1)
        -dResults.at<double>(0, 0)*dResults.at<double>(2, 1))/(dResults.at<double>(2, 0)*dResults.at<double>(2, 0));
    tempDeriv.y = (dResults.at<double>(2,0)*dResults.at<double>(1, 1)
    	-dResults.at<double>(1, 0)*dResults.at<double>(2, 1))/(dResults.at<double>(2, 0)*dResults.at<double>(2, 0));
    
    return tempDeriv;
}

cv::Point3d deprojectStereoTangent(const cv_local::stereoCorrespondence &imagePoint,const cv_local::stereoCorrespondence &imagePointTangent,const cv::Mat &P_l ,const cv::Mat &P_r )
{

    cv::Point3d pt3d(deprojectStereoPoint(imagePoint , P_l , P_r));

    Mat Ki_l = P_l.colRange(0, 3).inv(DECOMP_LU);
    Mat Ki_r = P_r.colRange(0, 3).inv(DECOMP_LU);

    // Matrixfy the points...
    Mat pts_l(3, 1, CV_64FC1);
    Mat pts_r(3, 1, CV_64FC1);

    pts_l.at<double>(0) = imagePoint[0].x;
    pts_l.at<double>(1) = imagePoint[0].y;
    pts_l.at<double>(2) = 1.0;

    pts_r.at<double>(0) = imagePoint[1].x;
    pts_r.at<double>(1) = imagePoint[1].y;
    pts_r.at<double>(2) = 1.0;

    Mat pts_dl(3, 1, CV_64FC1);
    Mat pts_dr(3, 1, CV_64FC1);

    pts_dl.at<double>(0) = imagePointTangent[0].x;
    pts_dl.at<double>(1) = imagePointTangent[0].y;
    pts_dl.at<double>(2) = 0.0;

    pts_dr.at<double>(0) = imagePointTangent[1].x;
    pts_dr.at<double>(1) = imagePointTangent[1].y;
    pts_dr.at<double>(2) = 0;

    // Set up the colums

    // rhs...

    Mat rhs = Ki_l*pts_l;

    // lhs ratio mat.

    Mat lhsMat(3,3,CV_64FC1);

    Mat(Ki_l*pts_dl*(-1.0)).copyTo(lhsMat.col(0));
    Mat(Ki_r*pts_r).copyTo(lhsMat.col(1));
    Mat(Ki_r*pts_dr).copyTo(lhsMat.col(2));

    ROS_INFO_STREAM("points: " << imagePoint.left_ << imagePoint.right_);
    ROS_INFO_STREAM("tangents: " << imagePointTangent.left_ << imagePointTangent.right_);

    ROS_INFO_STREAM("Ki's " << Ki_l << Ki_r);

    ROS_INFO_STREAM("Matrix Equation\n" << rhs << lhsMat);

    Mat results = lhsMat.inv(DECOMP_LU)*rhs;

    ROS_INFO_STREAM("Results of the lambdas\n" << results);

    Mat tangent = rhs+Ki_l*pts_dl*results.at<double>(0);

    ROS_INFO_STREAM("Image space tangent\n" << tangent);


    Point3d outputTan;

    outputTan.x = tangent.at<double>(0);
    outputTan.y = tangent.at<double>(1);
    outputTan.z = tangent.at<double>(2);

    double scale(norm(outputTan));

    // normalize the output results.
    return outputTan*(1/scale);
}


cv::Point3d deprojectStereoPoint(const cv_local::stereoCorrespondence& inputPts, const  cv::Mat &P_l , const cv::Mat &P_r)
{
    //construct the output mat:
    Mat results(4,1,CV_64FC1);
    Mat p_l(2,1,CV_64FC1);
    Mat p_r(2,1,CV_64FC1);

    p_l.at<double>(0) = inputPts[0].x;
    p_l.at<double>(1) = inputPts[0].y;

    p_r.at<double>(0) = inputPts[1].x;
    p_r.at<double>(1) = inputPts[1].y;

    triangulatePoints(P_l,P_r,p_l,p_r,results);

    Point3d output;

    output.x =  results.at<double>(0)/results.at<double>(3);
    output.y =  results.at<double>(1)/results.at<double>(3);
    output.z =  results.at<double>(2)/results.at<double>(3);

    return output;
}


void deprojectStereoPoints(const std::vector < cv_local::stereoCorrespondence > &inputArray , cv::OutputArray outputPointArray, const cv::Mat& P_l , const cv::Mat P_r){

    int n = inputArray.size();
    std::vector < Point2f > pts_left;
    std::vector < Point2f > pts_right;

    pts_left.resize(n);
    pts_right.resize(n);

    Mat results(4,n,CV_64FC1);

    //break up the stereo correspondence.
    for(int index =0; index < n; index++)
    {
        pts_left[index] = inputArray[index][0];
        pts_right[index] = inputArray[index][1];
    }

    //triangulate the full arrays.
    //ROS_INFO("Triuangulating a point array");
    triangulatePoints(P_l,P_r,pts_left,pts_right,results);

    //Mat homoResults = results.t();
    Mat resultsOut(3,n,CV_64FC1);
    //ROS_INFO_STREAM(results);
    //ROS_INFO("De-Homonegeousing");
    for(int index = 0; index < n ; index++)
    {
        for(int xyz = 0; xyz <3; xyz++)
        {
            if(results.depth() == CV_64F)
            {
                resultsOut.at<double>(xyz,index) = results.at<double>(xyz,index)/results.at<double>(3,index);
            }
            else
            {
                 resultsOut.at<double>(xyz,index) = results.at<float>(xyz,index)/results.at<float>(3,index);
            }

        }
    }
    //ROS_INFO_STREAM(resultsOut);
    //convertPointsFromHomogeneous(homoResults,resultsOut);
    /**
     * @todo The output array creation is not as robust as I had hoped.
     * Will have to be more careful.
     */
    if(outputPointArray.channels() == 3)
    {
        //ROS_INFO("The channel output is 3");
        //Array of points in 3d.
        outputPointArray.create(1,n,CV_64FC3);
        Mat outputMat = outputPointArray.getMat();
        Vec3d outputPoint;

        for(int index = 0; index < n; index++)
        {
            for(int xyz = 0; xyz < 3; xyz++)
            {

                outputPoint[xyz] = resultsOut.at<double>(xyz,index);
            }
            outputMat.at<Vec3d>(index) = outputPoint;
        }
    }
    else
    {
        outputPointArray.create(3,n,CV_64FC1);
        Mat outputMat =  outputPointArray.getMat();
        resultsOut.copyTo(outputMat);
    }

}



void reprojectArrayStereo(cv::InputArray points, std::vector < cv_local::stereoCorrespondence > & ,const cv::Mat &P_l, const cv::Mat &P_r){


}

void reprojectArrayStereo(cv::InputArray points, cv::OutputArray pointts_left, cv::OutputArray points_right  ,const cv::Mat &P_l, const cv::Mat &P_r){



}



void deprojectEllipseEnds(const cv_local::rotatedRectStereoCorr &inputRects, Point3d &pt0 , Point3d &pt1 , const cv::Mat &P_l , const cv::Mat &P_r){

    cv_local::stereoCorrespondence pair0;
    cv_local::stereoCorrespondence pair1;

    /*
     * This segment here is coded to assume that the pairs are based on the rotated rect with and are aligned
     *
     */
    for(int lr = 0 ; lr <2; lr++)
    {

        pair0[lr] = inputRects[lr].center+Point2f(sin(inputRects[lr].angle*3.14/180),-cos(inputRects[lr].angle*3.14/180))*(inputRects[lr].size.height/2.0);

        pair1[lr] = inputRects[lr].center-Point2f(sin(inputRects[lr].angle*3.14/180),-cos(inputRects[lr].angle*3.14/180))*(inputRects[lr].size.height/2.0);

        if(pair0[lr].y < pair1[lr].y)
        {
            Point2f temp(pair0[lr]);
            pair0[lr] = pair1[lr];
            pair1[lr] = temp;
        }

    }
    ROS_INFO_STREAM(inputRects[0].center);
    ROS_INFO_STREAM(inputRects[1].center);
    ROS_INFO_STREAM(pair0[0]);
    ROS_INFO_STREAM(pair0[1]);
    ROS_INFO_STREAM(pair1[0]);
    ROS_INFO_STREAM(pair1[1]);

    pt0 =  deprojectStereoPoint(pair0, P_l, P_r);
    pt1 = deprojectStereoPoint(pair1, P_l, P_r);



}

//This function computes the transformation and derivative from an r vec and a t vec. The derivative is also returned.
//derivative is 12x6.
void transformJacobian(const Mat &rvec ,const Mat & tvec, Mat & trans , OutputArray trans_jac){

    Mat G_lo = Mat::eye(4,4,CV_64FC1);

    //ROS_INFO("Point 1");
    Mat R_lo = G_lo(Range(0,3),Range(0,3));

    Mat R_temp(3,3,CV_64FC1);

    Mat R_jac(3,9,CV_64FC1);

    Rodrigues(rvec,R_temp,R_jac); //R_jac is a 9x3 matrix.

    R_jac = R_jac.t();

    //ROS_INFO("Point 2");

    R_temp.copyTo(R_lo);

    Mat T_lo = G_lo.col(3).rowRange(0,3);

    Mat G_jac=Mat::zeros(12,6,CV_64FC1);

    tvec.copyTo(T_lo);

    for(int ind1 = 0; ind1 < 3; ind1++)
    {
        for(int ind = 0; ind < 3; ind++)
        {
            R_jac.row(ind+ind1*3).copyTo(G_jac.row(ind+ind1*4).colRange(0,3));
        }
        G_jac.at<double>(ind1*4+3,3+ind1) = 1.0;
    }



    trans = G_lo.clone();
    if(trans_jac.needed())
    {
        trans_jac.create(G_jac.size(),G_jac.type());

        Mat trans_jac_ =  trans_jac.getMat();
        G_jac.copyTo(trans_jac_);

    }

    return;

}


void computeRvecTvec(const cv::Mat & trans, cv::Mat & rvect, cv::Mat & tvect, cv::OutputArray trans_jac)
{
    // pull of the rotation matrix as well as the translation vector.
    Mat R_t = trans(Range(0, 3), Range(0, 3));

    tvect = trans.col(3).rowRange(0, 3).clone();

    Rodrigues(R_t, rvect);
    return;
}


//Transforms a point using rvec and tvec.
Mat transformPoints(const cv::Mat &points,const cv::Mat& rvec, const cv::Mat &tvec, cv::OutputArray jac)
{
        CV_Assert(points.depth() == CV_32F || points.depth() == CV_64F);
        CV_Assert(points.cols > 0 && points.rows == 4);
        Mat transform;
        if(jac.needed())
        {

            Mat transformJac;
            transformJacobian(rvec,tvec,transform,transformJac);
            Mat dABdA,dABdB;
            matMulDeriv(transform,points,dABdA,dABdB); 
            //dABdA is 4x16 because it ignores the fact that the last 4 numbers (last row) of G is a constant. and the last number of points is a constant
            //dABdB is 4x4.
            //The full jacobian must be 3x9
            // [ d (transform points) / d points , d (transform points) / d rvec , d (transform points) / d tvec ]^T
            // [ 3 x 3 , 3 x 3 , 3x3 ]^T

            jac.create(3 , 9, CV_64FC1);
            Mat jacMat = jac.getMat();
            Mat pointDerivative = dABdB.rowRange(0,3).colRange(0,3);
            Mat jacPoint = jacMat.colRange(0,3);

            pointDerivative.copyTo(jacPoint);

            Mat transDerivative = dABdA.colRange(0,12).rowRange(0,3)*transformJac;
            Mat jacTrans = jacMat.colRange(3,9);
            transDerivative.copyTo(jacTrans);

        }
        else
        {
            transformJacobian(rvec,tvec,transform);
        }

        return transform*points;
}

//Transforms a point using rvec and tvec.
Mat transformPointsSE3(const cv::Mat &points,const cv::Mat &G, cv::OutputArray jac)
{
        CV_Assert(points.depth() == CV_32F || points.depth() == CV_64F);
        CV_Assert(points.cols > 0 && points.rows == 4);
        CV_Assert(G.cols == 4 && G.rows == 4);
        // This is an n point transform. (i.e. points is a 4 x n matrix).
        if (jac.needed())
        {
            int n(points.cols);
            Mat transformJac;
            Mat dABdA, dABdB;
            matMulDeriv(G, points, dABdA, dABdB); 
            // dABdA is (4*n)x(16) because it ignores the fact that the last 4 numbers (last row) of G is a constant. and the last number of a point in RP3 is a constant.
            // dABdB is (4*n)x(4*n). This is unused
            // The output jacobian must be (3*n)x(12) (may technically by (3*n) x 6 since the rotation matrix derivative is in (R^3).

            

            // obtain the correct submatrix. 
            // This should really be a tensor...
            jac.create(3*n, 12, CV_64FC1);
            Mat jacMat = jac.getMat();

            for (int ind(0); ind < n; ind++)
            {
                // use ROI's to cut off the matrix accordingly.
                Mat DptdG1(dABdA.row(ind).colRange(0, 12));
                Mat DptdG2(dABdA.row(ind+n).colRange(0, 12));
                Mat DptdG3(dABdA.row(ind+2*n).colRange(0, 12));

                Mat subJac1(jacMat.row(ind*3).colRange(0, 12));
                Mat subJac2(jacMat.row(ind*3+1).colRange(0, 12));
                Mat subJac3(jacMat.row(ind*3+2).colRange(0, 12));

                DptdG1.copyTo(subJac1);
                DptdG2.copyTo(subJac2);
                DptdG3.copyTo(subJac3);
            }
        }
        return G*points;
}
};  // namespace cv_projective

