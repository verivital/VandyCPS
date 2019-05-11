/* 
* opencv_ui.cpp:
* 
* Created By Connor Balin
* Modified By Russell Jackson 12/2012
*/


/*This file relies on the following external libraries:
OpenCV (2.3.1)
*/

//This file defines common useful functions (based on opencv) that are used throughout the
//program. 


#include "cwru_opencv_common/opencv_ui.h"



//This function compute the error in hue
//The ints are assumed to be byte (0-255);

using namespace cv;
using namespace cv_local;

namespace cv_ui{


void defaultWindowLocation(int &lx, int &ly, int &rx, int &ry){
	FILE* pFile = fopen ("localFiles/windowLocation.local.txt","r");
	fscanf(pFile,"%i, %i, %i, %i",&lx,&ly,&rx,&ry);
}


void getCoordinates(int event,int x,int y,int flags,void * param){
	if(event != CV_EVENT_LBUTTONDOWN)	return;
	//if(param==NULL){
	//std::cout<<Point(x,y)<<std::endl;
	//}
	else{
		Point *pt = (Point *) param;
		pt->x = x;
		pt->y = y;
	}
}

void displayPixel(int event,int x,int y,int flags,void * param){
	Mat *mat = (Mat *) param;
	if(event != CV_EVENT_LBUTTONDOWN)	return;
	std::cout<< "xPos: " << x;
	std::cout<< ", yPos: " << y;
	std::cout<< " color Value: "  << mat->row(y).col(x)<<std::endl;
}

void createContours(int event, int x, int y, int flags, void *param){
	/*
	generates a contour for each time mouse is dragged while holding left button and deletes a contour when right clicked
	*/
	ContourList *selections = (ContourList *) param;
	switch(event){

		case CV_EVENT_RBUTTONDOWN:
			for(unsigned int i=0;i<selections->size();i++){
				if((*selections)[i].size()==0)	continue;
				bool inside = pointPolygonTest(Mat((*selections)[i]),Point(x,y),false)>=0.0;
				if(inside){
					(*selections)[i].clear();
				}
			}
			break;

		case CV_EVENT_LBUTTONDOWN:
				selections->push_back(std::vector<Point>());
				break;

		case CV_EVENT_MOUSEMOVE:
			if(flags&CV_EVENT_FLAG_LBUTTON){	//dragged
				if(selections->empty())	selections->push_back(std::vector<Point>());
				(*selections)[selections->size()-1].push_back(Point(x,y));
			}
			break;
	}
}


stereoCorrespondence ptClickedIn(stereoImage& _sceneIn){

			// open an image and prompt for a click:
			stereoCorrespondence stereoData;

			Point leftPt =  Point(-1,-1);
			namedWindow("leftInput");
			cvSetMouseCallback("leftInput",getCoordinates,&leftPt);
			imshow("leftInput",_sceneIn[0]);

			while(leftPt.x == -1){
				waitKey(40);
			}
			stereoData[0] = Point2f(leftPt.x,leftPt.y);

			destroyWindow("leftInput");

			Point rightPt = Point(-1,-1);
			namedWindow("rightInput");
			cvSetMouseCallback("rightInput",getCoordinates,&rightPt);
			imshow("rightInput",_sceneIn[1]);

			while(rightPt.x == -1){
				waitKey(40);
			}
			destroyWindow("rightInput");

			stereoData[1] = Point2f(rightPt.x,rightPt.y);


			return stereoData;
		
}

std::vector<stereoCorrespondence> mulPtClickedIn(stereoImage& _sceneIn,int inputCount){

			// open an image and prompt for a click:
			std::vector<stereoCorrespondence> inputPointList;
			if(inputCount > 0) inputPointList.resize(inputCount);
			else
			{
				inputPointList.clear();
				return inputPointList;
			}
			
			Point tempPt =  Point(-1,-1);
			namedWindow("InputImage");
			cvSetMouseCallback("InputImage",getCoordinates,&tempPt);

			//grab the ImagePoints. all left, then all right.
			for(int lr = 0; lr < 2; lr++)
			{
				imshow("InputImage",_sceneIn[lr]);
				for(int index = 0; index < inputCount; index++)
				{
					while(tempPt.x == -1){
						waitKey(40);
					}
					inputPointList[index][lr] = Point2f(tempPt.x,tempPt.y);
					tempPt = Point(-1,-1);
				}
			}
			destroyWindow("InputImage");

			return inputPointList;
		
}



};




