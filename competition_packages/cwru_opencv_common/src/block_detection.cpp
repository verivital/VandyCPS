/*FiducialBase.cpp
 *Case Western Reserve Mechatronics Lab -- Professor Murat Cavusoglu
 *Author: Eddie E. Massey III
 *A class to determine the location of a fiducial in an image using 
 * thresholding and blob detection Simple: and expandable.
 * Dec 2012 
 */

 //Standard library

#include "cwru_opencv_common/block_detection.h"
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <ros/ros.h>

using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Point;
using cv::Size;

void detectBlock(const cv::Mat &inputImg, cv::Point seedPt, std::vector<cv::Point2f> &corners, bool display)
{
    bool localDisplay = true;

    int newMaskVal = 255;
    cv::Scalar newVal = cv::Scalar( 120, 120, 120 );

    int connectivity = 8;
    int flags = connectivity | (newMaskVal << 8 ) | cv::FLOODFILL_FIXED_RANGE | cv::FLOODFILL_MASK_ONLY;

    int lo = 20;
    int up = 20;

    cv::Mat mask2 = cv::Mat::zeros( inputImg.rows + 2, inputImg.cols + 2, CV_8UC1 );
    cv::floodFill( inputImg, mask2, Point(seedPt.x,seedPt.y), newVal, 0, cv::Scalar( lo, lo, lo ), cv::Scalar( up, up, up), flags );
    cv::Mat mask = mask2( cv::Range( 1, mask2.rows - 1 ), cv::Range( 1, mask2.cols - 1 ) );


    cv::Mat elementOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat elementClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    Mat openMask;
    Mat closeMask;

    cv::morphologyEx(mask, openMask, cv::MORPH_OPEN, elementOpen);
    cv::morphologyEx(openMask, closeMask, cv::MORPH_CLOSE, elementClose);

    //outputImg = closeMask.clone();
    if(display)
    {
        cv::imshow( "TempImage", openMask);
        cv::waitKey(0);
        cv::destroyWindow("TempImage");
    }


    std::vector< std::vector< cv::Point> > contours;

    cv::findContours(closeMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    if(display)
    {
        ROS_INFO("There are  %d total contours",(int) contours.size());
    }

    sort(contours.begin(),contours.end(), cv_local::contourCompare);

    if(display)
    {
        ROS_INFO("The largest contour is %d.", (int) contours[0].size());
    }

    cv::RotatedRect minRect = cv::minAreaRect(contours[0]);
    Point2f ptArray[4];

    minRect.points(ptArray);

    std::vector<cv::Point2f> outputPts(ptArray,ptArray+sizeof(ptArray)/sizeof(Point2f));

    if(display)
    {
        ROS_INFO("Finished finding the RotatedRect and its associated point array");
        for(int ind=0; ind <4; ind++)
        {
            ROS_INFO_STREAM(outputPts[ind]);
        }
    }

    refineBlock(inputImg, outputPts, display);
    corners = outputPts;
}

void sortPtGrid(std::vector<cv::Point2f> & corners, const cv::Size &gridInfo, bool xDir)
{
    // before running, verify the sizes match:
    if (corners.size() != gridInfo.area())
    {
        return;
    }

    std::vector<cv::Point2f> tempCorners = corners;

    /* Point layout depends on the direction bool. 
     *     1      0
     *     2      3
     */

     // y is also sorted from low to high
    std::sort(tempCorners.begin(), tempCorners.end(), cv_local::pointYSortingLH<float>);

    for (int ind(0); ind < gridInfo.height; ind++)
    {
    	int offset1(gridInfo.width*ind);
    	int offset2(offset1+gridInfo.width);
    	if (xDir)  // sort x from high to low
    	{
    		std::sort(tempCorners.begin()+offset1, tempCorners.begin() + offset2, cv_local::pointXSortingHL<float>);
    	}
    	else // sort x from low to high
    	{
    		std::sort(tempCorners.begin()+offset1, tempCorners.begin() + offset2, cv_local::pointXSortingLH<float>);
    	}
	}
}


int refineBlock(const cv::Mat& inputImg, std::vector<cv::Point2f> & points,bool display)
{
    if(display) ROS_INFO("started refinement");
    // sort all of the y vectors from low to high
    std::vector<cv::Point2f> tempPoints = points;


    std::sort(tempPoints.begin(),tempPoints.end(),cv_local::pointYSortingLH<float>);
    //first row from High to Low
    std::sort(tempPoints.begin(),tempPoints.begin()+2,cv_local::pointXSortingHL<float>);
    //second row from Low to High
    std::sort(tempPoints.begin()+2,tempPoints.begin()+4,cv_local::pointXSortingLH<float>);

     /* Point layout (rough)
         *     1      0
         *     2      3
        */
    //Now spread the points by 2 pixels each.
    float spread = 4.0;
    tempPoints[0] += Point2f(spread,-spread);
    tempPoints[1] += Point2f(-spread,-spread);
    tempPoints[2] += Point2f(-spread,spread);
    tempPoints[3] += Point2f(spread,spread);

    if(display) ROS_INFO("Completed iterating the points");

    std::vector< Point2f > oldPoints = points;

    // Done going through the contour Now update the point information:
    cornerSubPix(inputImg,points, Size(spread, spread), Size(-1, -1),
    	cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 3, 0.01));

    int n = points.size();

    for (int indi = 0; indi < n; indi++)
    {
        for (int indj=0; indj < indi; indj++)
        {
            double dist = norm(points[indi]-points[indj]);
            if (dist < 3)
            {
                points = oldPoints;
                return -1;
            }
        }
    }
    return 1;
}


void exportPointFile(const char* filePath, const std::vector< std::vector< Point3f > > &inputPoints)
{
    FILE* pFile = fopen(filePath, "w");

    if (pFile == NULL)
    {
        printf("Unable to open the ellipse file.\n");
        return;
    }

    for (int ind = 0; ind < inputPoints.size(); ind++)
    {
        fprintf(pFile, "%d", ind);
        for (int jnd = 0; jnd < inputPoints[ind].size(); jnd++)
        {
            fprintf(pFile, ", %lf, %lf,%lf"
								,inputPoints[ind][jnd].x
								,inputPoints[ind][jnd].y
                                ,inputPoints[ind][jnd].z);
        }
        fprintf(pFile, "\n");
	}
	fclose(pFile);
}

