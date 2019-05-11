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

#include "cwru_opencv_common/projective_geometry.h"
#include "cwru_opencv_common/ellipse_modeling_numerical.h"

using namespace cv;
using namespace cv_local;

namespace cv_ellipse_num
{


double circleEnergy(const cv::Mat &segmentedImage, const cv::Mat &P, const cv::Mat &G_co, const cv::Point3d &center, double rad, int segments, cv::OutputArray jac)
{
	// The circle energy is maximizd when the segmented image is dark where the circle is dark and the gradient around image
	// matches the circle's edge.
	Mat pt_o(4, 1, CV_64FC1);
	
	
	//create the cirlcle
	std::vector< std::vector<Point> > imagePts;
	
	imagePts.clear();
	imagePts.resize(1);

	Rect imageROI(projectCirclePoints(imagePts[0], P, G_co, center, rad, segments));


	// now create a convex hull.
	Rect baseRect(Point(0, 0), segmentedImage.size());


	// @todo. the intersection may be zero.
    Rect finalROI(imageROI & baseRect);

    if (finalROI.tl().x <= 0)
    {
        finalROI.width -= (1 - finalROI.x);
        finalROI.x = 1;
        if (finalROI.width <= 0)
        {
            return -1.0;
        }
    }

    if (finalROI.br().x >= segmentedImage.size().width) 
	{
		finalROI.width = (segmentedImage.size().width - 1 - finalROI.x); 
		if (finalROI.width <= 0)
		{
			return -1.0;
		}
	}
	
	if (finalROI.tl().y <= 0) 
	{
		finalROI.height -= (1-finalROI.y);
		finalROI.y = 1;
		if (finalROI.height <= 0)
		{
			return -1.0;
		}	
	}

	if (finalROI.br().y >= segmentedImage.size().height) 
	{
		finalROI.height = (segmentedImage.size().height - 1 - finalROI.y); 
		if (finalROI.height <= 0)
		{
			return -1.0;
		}
	}

	if (finalROI.area() <= 0)
	{
		//error return
		return -1.0;
	}



	Mat edgeImage(finalROI.height, finalROI.width, CV_32FC1);
	Mat fillImage(finalROI.height, finalROI.width, CV_32FC1);
	edgeImage.setTo(0);
	fillImage.setTo(0);

	drawContours(edgeImage, imagePts, 0, Scalar(255, 255, 255), 3, 8, noArray(), INT_MAX, finalROI.tl()*-1);
	drawContours(fillImage, imagePts, 0, Scalar(255, 255, 255), -1, 8, noArray(), INT_MAX, finalROI.tl()*-1);


	
	//get the ROI of the base image.
	if (jac.needed())
	{
		finalROI -= Point(1,1);
		finalROI += Size(2,2);
		if (finalROI.tl().x < 0 || finalROI.tl().y < 0 || finalROI.br().x > segmentedImage.size().width || finalROI.br().y > segmentedImage.size().height)
		{
			std::cout << finalROI << std::endl;
			return 0;
		}
		
	}

	Mat subImage(segmentedImage(finalROI));
	
	Mat subImagef;

	subImage.convertTo(subImagef,CV_32F);
	
	// make derivative magnitude images.
	Mat derivX, derivY;
	
	Scharr(subImage, derivX, CV_32F, 1, 0);
	Scharr(subImage, derivY, CV_32F, 0, 1);

	Mat dMagS(derivX.mul(derivX)+derivY.mul(derivY));
	Mat dMag;
	sqrt(dMagS, dMag);
	
	// use cross correlation for the fill match, as well as the edge match.
	// Note: there are several different matching methods: 
	// http://docs.opencv.org/2.4/modules/imgproc/doc/object_detection.html?highlight=matchtemplate#matchtemplate
	// edge image:
	Mat edgeMatch;
	matchTemplate(dMag, edgeImage, edgeMatch, CV_TM_CCORR_NORMED);

	// fill image
	Mat fillMatch;
	matchTemplate(subImagef, fillImage, fillMatch, CV_TM_CCORR_NORMED);

	float energy(0.0);
	// compute the rms energy:
	if ( jac.needed() )
	{
		//std::cout << edgeMatch.size() << std::endl;
		energy = edgeMatch.at<float>(1,1)*edgeMatch.at<float>(1,1) + fillMatch.at<float>(1,1)*fillMatch.at<float>(1,1);
	}
	else
	{
		
		//std::cout << edgeMatch.size() << std::endl;
		energy = edgeMatch.at<float>(0,0)*edgeMatch.at<float>(0,0) + fillMatch.at<float>(0,0)*fillMatch.at<float>(0,0);
	}

	if ( jac.needed() )
	{
		// compute the derivative of the match w.r.t x and y
		Mat edgeDx, edgeDy;
		Scharr(edgeMatch, edgeDx, CV_32F, 1, 0);
		Scharr(edgeMatch, edgeDy, CV_32F, 0, 1);

		Mat fillDx, fillDy;
		Scharr(fillMatch, fillDx, CV_32F, 1, 0);
		Scharr(fillMatch, fillDy, CV_32F, 0, 1);

		// derivative w.r.t the actual circle center.
		Mat grad(1,2, CV_64FC1);
		grad.at<double>(0,0) = static_cast<double> (2*edgeMatch.at<float>(1,1)*edgeDx.at<float>(1,1)+2*fillMatch.at<float>(1,1)*fillDx.at<float>(1,1));
		grad.at<double>(0,1) = static_cast<double> (2*edgeMatch.at<float>(1,1)*edgeDy.at<float>(1,1)+2*fillMatch.at<float>(1,1)*fillDy.at<float>(1,1));

		// compute the jacobian w.r.t to G_co (the 6 dof form)
		Mat ptJac, imgPoints;
		Mat spacialPoints(3, 1, CV_64FC1);
		
		spacialPoints.at<double>(0) = center.x;
		spacialPoints.at<double>(1) = center.y;
		spacialPoints.at<double>(2) = center.z;

		cv_projective::reprojectPointsSE3(spacialPoints, imgPoints, P, G_co, ptJac);
		
		jac.create(1,12,CV_64FC1);
        Mat jacMat = jac.getMat();

        Mat resultJac = grad*ptJac;

        resultJac.copyTo(jacMat);

	}

	return energy;
}



cv::Rect projectCirclePoints(std::vector<Point> & pointList, const cv::Mat &P, const cv::Mat &G_co, const cv::Point3d &center, double rad, int segments)
{
	// circle points in object frame. 
	Mat pt_o(4, 1, CV_64FC1);
	//create the cirlcle
	pointList.clear();

	Rect imageROI(-1, -1, 0, 0);
	Point oldPt(-1,-1);
	for (int ind(0); ind < segments; ind++)
	{
		// compute the point:
		pt_o.at<double>(0) = cos(2*ind*3.141/segments)*rad+center.x;
		pt_o.at<double>(1) = sin(2*ind*3.141/segments)*rad+center.y;
		pt_o.at<double>(2) = center.z;
		pt_o.at<double>(3) = 1.0;
		
		Mat pt_c = G_co*pt_o;
		Point3d result;
		result.x = pt_c.at<double>(0);
		result.y = pt_c.at<double>(1);
		result.z = pt_c.at<double>(2);
		cv::Point2d ptLoc(cv_projective::reprojectPoint(result, P));
		
		cv::Point newPt(ptLoc.x, ptLoc.y);
		
		if (norm(newPt-oldPt) > 0)	
		{
			pointList.push_back(newPt);
			
			if (imageROI.x > 0)
			{
				imageROI |= Rect(newPt+Point(-5, -5), newPt+Point(5, 5));
			}
			else imageROI = Rect(newPt+Point(-5, -5), newPt+Point(5, 5));
		}
		oldPt = newPt;
	}
	return imageROI;
}

};  // namespace cv_ellipse_num
