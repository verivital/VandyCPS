/* 
* opencv.cpp:
* 
* Created By Connor Balin
* Modified By Russell Jackson 12/2012
*/


/*This file relies on the following external libraries:
OpenCV (2.3.1)
*/

//This file defines common useful functions (based on opencv) that are used throughout the
//program. 


#include "cwru_opencv_common/opencv_local.h"



//This function compute the error in hue
//The ints are assumed to be byte (0-255);

using namespace cv;
using namespace cv_local;

namespace cv_local
{







int byteError(int a,int b){

	int  hueError1 = (256-a)%256+b;
	int  hueError2 = (256-b)%256+a;
	int  hueError3 = abs(a-b);
	return min(min(hueError1,hueError2),hueError3);
}


bool contourCompare(std::vector<Point> contour1, std::vector<Point> contour2)
{
	//Right now the comparison is number of points
	//A beeter comparison might be area:
	double size1 = contour1.size();
	double size2 = contour2.size();

	//Descending size:
	if(size1 >  size2) return true;
	else return false;

}



int countOutputChannels(int conversion){
	switch(conversion){
		case CV_BGR2BGRA:
		case CV_RGB2BGRA:
		case CV_BGRA2RGBA:
		case CV_BGR5652BGRA:
		case CV_BGR5552BGRA:
		case CV_BGR5652RGBA:
		case CV_BGR5552RGBA:
		case CV_GRAY2BGRA:
			return 4;
			break;

		case CV_BGR2YCrCb:
		case CV_RGB2YCrCb:
		case CV_BGR2XYZ:
		case CV_RGB2XYZ:
		case CV_BGR2HSV:
		case CV_RGB2HSV:
		case CV_BGR2Lab:
		case CV_RGB2Lab:
		case CV_BGR2Luv:
		case CV_RGB2Luv:
		case CV_BGR2HLS:
		case CV_RGB2HLS:
			return 3;
			break;

		case CV_BayerBG2BGR:
		case CV_BayerGB2BGR:
		case CV_BayerRG2BGR:
		case CV_BayerGR2BGR:

		case CV_BGRA2BGR:
		case CV_RGBA2BGR:
		case CV_RGB2BGR:
		case CV_BGR5652BGR:
		case CV_BGR5552BGR:
		case CV_BGR5652RGB:
		case CV_BGR5552RGB:
		case CV_GRAY2BGR:

		case CV_YCrCb2BGR:
		case CV_YCrCb2RGB:
		case CV_XYZ2BGR:
		case CV_XYZ2RGB:
		case CV_HSV2BGR:
		case CV_HSV2RGB:
		case CV_Lab2BGR:
		case CV_Lab2RGB:
		case CV_Luv2BGR:
		case CV_Luv2RGB:
		case CV_HLS2BGR:
		case CV_HLS2RGB:
			return 3;
			break;

		case CV_BGR2BGR565:
		case CV_BGR2BGR555:
		case CV_RGB2BGR565:
		case CV_RGB2BGR555:
		case CV_BGRA2BGR565:
		case CV_BGRA2BGR555:
		case CV_RGBA2BGR565:
		case CV_RGBA2BGR555:
		case CV_GRAY2BGR565:
		case CV_GRAY2BGR555:
			return 2;
			break;

		case CV_BGR2GRAY:
		case CV_BGRA2GRAY:
		case CV_RGB2GRAY:
		case CV_RGBA2GRAY:
		case CV_BGR5652GRAY:
		case CV_BGR5552GRAY:
			return 1;
			break;
		default:
			CV_Error( CV_StsBadFlag, "Unknown/unsupported color conversion code" );
			return -1;
	}
}

 


};




