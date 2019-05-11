/*rotation_operations.cpp
*Case Western Reserve Mechatronics Lab -- Professor Murat Cavusoglu
*Author: Russell Jackson, Viraj Desai
* 3/4/2015
*/

/*This file relies on the following external libraries:

OpenCV (2.3.1)
Eigen  (?????)
*/

//This file generates the function prototypes that will become common to multiple projects.
//The library will rely on opencv to define the basic math, but will be expanded to include
//other material, ie screws, twists, D-H params, and Quaternions.

#include "cwru_opencv_common/cv_rotations.h"
#include <math.h> 
#include <iostream> 


namespace cv_rot
{

cv::Mat QuatToMatrix(Quaternion inputQ, cv::OutputArray jac){
	Quaternion inputQn = QuatNormalize(inputQ);
	//cv::Mat_<double> outputM = cv::Mat::zeros(3,3,CV_32F);
	cv::Mat outputM = cv::Mat::zeros(3,3,CV_64FC1);
	outputM.at<double>(0,0) =  pow(inputQn.data[0],2) + pow(inputQn.data[1],2) - pow(inputQn.data[2],2) - pow(inputQn.data[3],2);
	outputM.at<double>(1,0) =  2*(inputQn.data[1]*inputQn.data[2] + inputQn.data[0]*inputQn.data[3]);
	outputM.at<double>(2,0) =  2*(inputQn.data[1]*inputQn.data[3] - inputQn.data[0]*inputQn.data[2]);
	outputM.at<double>(0,1) =  2*(inputQn.data[1]*inputQn.data[2] - inputQn.data[0]*inputQn.data[3]);
	outputM.at<double>(1,1) =  pow(inputQn.data[0],2) - pow(inputQn.data[1],2) + pow(inputQn.data[2],2) - pow(inputQn.data[3],2);
	outputM.at<double>(2,1) =  2*(inputQn.data[2]*inputQn.data[3] + inputQn.data[0]*inputQn.data[1]);
	outputM.at<double>(0,2) =  2*(inputQn.data[1]*inputQn.data[3] + inputQn.data[0]*inputQn.data[2]);
	outputM.at<double>(1,2) =  2*(inputQn.data[2]*inputQn.data[3] - inputQn.data[0]*inputQn.data[1]);
	outputM.at<double>(2,2) =  pow(inputQn.data[0],2) - pow(inputQn.data[1],2) - pow(inputQn.data[2],2) + pow(inputQn.data[3],2);

	if (jac.needed())
	{
		// @ TODO
		// add the output array 
		// Please add eigen support....
		jac.create(9, 4, CV_64FC1);

		cv::Mat jacMat(jac.getMat());

		jacMat = 0.0;

		// zeroth row...
		jacMat.at<double> (0, 0) = 2.0*inputQn.data[0];
		jacMat.at<double> (0, 1) = 2.0*inputQn.data[1];
		jacMat.at<double> (0, 2) = -2.0*inputQn.data[2];
		jacMat.at<double> (0, 3) = -2.0*inputQn.data[3];

		// first row: index (1, 0)
		jacMat.at<double>(1, 0) = 2*(inputQn.data[3]);
		jacMat.at<double>(1, 1) = 2*(inputQn.data[2]);
		jacMat.at<double>(1, 2) = 2*(inputQn.data[1]);
		jacMat.at<double>(1, 3) = 2*(inputQn.data[0]);

		// second row: index (2, 0)
		jacMat.at<double>(2, 0) =  -2*(inputQn.data[2]);
		jacMat.at<double>(2, 1) =   2*(inputQn.data[3]);
		jacMat.at<double>(2, 2) =  -2*(inputQn.data[0]);
		jacMat.at<double>(2, 3) =   2*(inputQn.data[1]);

		// third row: index (0, 1)
		jacMat.at<double>(3, 0) =  -2*(inputQn.data[3]);
		jacMat.at<double>(3, 1) =   2*(inputQn.data[2]);
		jacMat.at<double>(3, 2) =   2*(inputQn.data[1]);
		jacMat.at<double>(3, 3) =  -2*(inputQn.data[0]);

		// fourth row: index (1, 1)
		jacMat.at<double>(4, 0) =   2*inputQn.data[0];
		jacMat.at<double>(4, 1) =  -2*inputQn.data[1];
		jacMat.at<double>(4, 2) =   2*inputQn.data[2];
		jacMat.at<double>(4, 3) =  -2*inputQn.data[3];

		// fifth row: index (2, 1)
		jacMat.at<double>(5, 0) =  2*(inputQn.data[1]);
		jacMat.at<double>(5, 1) =  2*(inputQn.data[1]);
		jacMat.at<double>(5, 2) =  2*(inputQn.data[3]);
		jacMat.at<double>(5, 3) =  2*(inputQn.data[2]);

		// sixth row: index (0, 2)
		jacMat.at<double>(6, 0) =  2*(inputQn.data[2]);
		jacMat.at<double>(6, 1) =  2*(inputQn.data[3]);
		jacMat.at<double>(6, 2) =  2*(inputQn.data[0]);
		jacMat.at<double>(6, 3) =  2*(inputQn.data[3]);

		// seventh row: index (1, 2)
		jacMat.at<double>(7,0) =  -2*(inputQn.data[1]);
		jacMat.at<double>(7,1) =  -2*(inputQn.data[0]);
		jacMat.at<double>(7,2) =   2*(inputQn.data[3]);
		jacMat.at<double>(7,3) =   2*(inputQn.data[2]);

		// eigth row: index (2, 2)
		jacMat.at<double>(8,0) =   2*inputQn.data[0];
		jacMat.at<double>(8,0) =  -2*inputQn.data[1];
		jacMat.at<double>(8,0) =  -2*inputQn.data[2];
		jacMat.at<double>(8,0) =   2*inputQn.data[3];

	}

	return outputM;
};


Quaternion QuatNormalize(Quaternion inputQ){
	Quaternion outputQ;
	double inputQ_Modulus = sqrt(inputQ.data[0]*inputQ.data[0] + inputQ.data[1]*inputQ.data[1] + inputQ.data[2]*inputQ.data[2] + inputQ.data[3]*inputQ.data[3]); 
	outputQ.data[0] = inputQ.data[0]/inputQ_Modulus;
	outputQ.data[1] = inputQ.data[1]/inputQ_Modulus;
	outputQ.data[2] = inputQ.data[2]/inputQ_Modulus;
	outputQ.data[3] = inputQ.data[3]/inputQ_Modulus;
	return outputQ;
};






Quaternion MatrixToQuat(cv::Mat& matInput){

	Quaternion outputQ;
	for(int i = 0; i < 4; i++)
	{
		outputQ.data[i] = 0.0;
	}

	cv::Size matSize = matInput.size();

	cv::Mat matExtracted;
	cv::Mat matDiag;
	if(matSize.height == matSize.width && (matSize.height == 3 || matSize.height == 4)){
		matExtracted = matInput(cv::Range(0,3),cv::Range(0,3)).clone();
	}
	else 
	{
    	return outputQ;
	}


    // compare the diagonal items and find out the maximum 
    double max_dia = matExtracted.at<double>(0,0);
    int index_max_dia = 0;

    if (matExtracted.at<double>(1,1) > max_dia)
    {
    	max_dia = matExtracted.at<double>(1,1);
    	index_max_dia = 1;
    }

    if (matExtracted.at<double>(2,2) > max_dia)
    {
    	max_dia = matExtracted.at<double>(2,2);
    	index_max_dia = 2;
    }

    int u, v, w;

     switch (index_max_dia)
     {
     	case 0:
     	     u = 0;
     	     v = 1;
     	     w = 2;
     	     break;
     	case 1:
     	     u = 1;
     	     v = 2;
     	     w = 0;
     	     break;
     	case 2:
     	     u = 2;
     	     v = 0;
     	     w = 1;
     	     break;
     }

     double r = sqrt(1 + matExtracted.at<double>(u,u) - matExtracted.at<double>(v,v) - matExtracted.at<double>(w,w));

     if (r < 1.0e-12){
     	outputQ.data[0] = 1.0;
     	outputQ.data[1] = 0.0;
     	outputQ.data[2] = 0.0;
     	outputQ.data[3] = 0.0;

        matExtracted.release();
     	return outputQ;
     }
     else{
     	double q1 = (matExtracted.at<double>(w,v) - matExtracted.at<double>(v,w))/(2*r);
     	double qu = r/2;
     	double qv = (matExtracted.at<double>(u,v) + matExtracted.at<double>(v,u))/(2*r);
     	double qw = (matExtracted.at<double>(w,u) + matExtracted.at<double>(u,w))/(2*r);
        
        double length_quaternion = sqrt(pow(q1,2) + pow(qu,2) + pow(qv,2) + pow(qw,2));
        
        // implicitly constrain the output quaternion angle to be between -pi/2 and pi/2.

        if (q1 < 0){
        	outputQ.data[0] = -q1/length_quaternion;
     	    outputQ.data[u+1] = -qu/length_quaternion;
     	    outputQ.data[v+1] = -qv/length_quaternion;
     	    outputQ.data[w+1] = -qw/length_quaternion;
        }
        else{
        	outputQ.data[0] = q1/length_quaternion;
     	    outputQ.data[u+1] = qu/length_quaternion;
     	    outputQ.data[v+1] = qv/length_quaternion;
     	    outputQ.data[w+1] = qw/length_quaternion;
        }

	    matExtracted.release();

        return outputQ;
     }
 }


/*
Quaternion MatrixToQuat(cv::Mat& matInput){

	Quaternion outputQ;
	for(int i = 0; i < 4; i++)
	{
		outputQ.data[i] = 0.0;
	}

	cv::Size matSize = matInput.size();

	cv::Mat matExtracted;
	cv::Mat matDiag;
	if(matSize.height == matSize.width && (matSize.height == 3 || matSize.height == 4)){
		matExtracted = matInput(cv::Range(0,3),cv::Range(0,3)).clone();
	}
	else 
	{
		return outputQ;
	}

	cv::Scalar scalarTrace = cv::trace(matExtracted);

	double aa = matExtracted.at<double>(0,0);
	double sqtrp1,sqdip1;
	if (scalarTrace.val[0] > 0)
	{
		sqtrp1 = sqrt( scalarTrace.val[0] + 1.0 );

		outputQ.data[0] = 0.5*sqtrp1; 

		outputQ.data[1] = (matExtracted.at<double>(2, 1) - matExtracted.at<double>(1, 2))/(2.0*sqtrp1);
		outputQ.data[2] = (matExtracted.at<double>(0, 2) - matExtracted.at<double>(2, 0))/(2.0*sqtrp1);
		outputQ.data[3] = (matExtracted.at<double>(1, 0) - matExtracted.at<double>(0, 1))/(2.0*sqtrp1);
	}
	else
	{		

		matDiag = matExtracted.diag().clone(); // I don't know why this didn't work.
		matDiag.at<double>(0,0) = matExtracted.at<double>(0, 0); // force diag 
		matDiag.at<double>(1,0) = matExtracted.at<double>(1, 1);
		matDiag.at<double>(2,0) = matExtracted.at<double>(2, 2);
		double a = matDiag.at<double>(0,0);
		double b = matDiag.at<double>(1,0);
		double c = matDiag.at<double>(2,0);
		if ((matDiag.at<double>(1,0) > matDiag.at<double>(0,0)) && (matDiag.at<double>(1,0) > matDiag.at<double>(2,0)))
		{
			sqdip1 = sqrt(matDiag.at<double>(1,0) - matDiag.at<double>(0,0) - matDiag.at<double>(2,0) + 1.0 );

			outputQ.data[2] = 0.5*sqdip1; 

			if ( sqdip1 != 0 )
			{
				sqdip1 = 0.5/sqdip1;
			}			

			outputQ.data[0] = (matExtracted.at<double>(0, 2) - matExtracted.at<double>(2, 0))*sqdip1; 
			outputQ.data[1] = (matExtracted.at<double>(1, 0) + matExtracted.at<double>(0, 1))*sqdip1; 
			outputQ.data[3] = (matExtracted.at<double>(2, 1) + matExtracted.at<double>(1, 2))*sqdip1; 
		}
		else if (matDiag.at<double>(2,0) > matDiag.at<double>(0,0))
		{

			sqdip1 = sqrt(matDiag.at<double>(2,0) - matDiag.at<double>(0,0) - matDiag.at<double>(1,0) + 1.0 );

			outputQ.data[3] = 0.5*sqdip1;

			if ( sqdip1 != 0 )
			{
				sqdip1 = 0.5/sqdip1;
			}

			outputQ.data[0] = (matExtracted.at<double>(1, 0) - matExtracted.at<double>(0, 1))*sqdip1; 
			outputQ.data[1] = (matExtracted.at<double>(0, 2) + matExtracted.at<double>(2, 0))*sqdip1; 
			outputQ.data[2] = (matExtracted.at<double>(2, 1) + matExtracted.at<double>(1, 2))*sqdip1; 
		}
		else
		{						
			sqdip1 = sqrt(matDiag.at<double>(0,0) - matDiag.at<double>(1,0) - matDiag.at<double>(2,0) + 1.0);

			outputQ.data[1] = 0.5*sqdip1; 

			if ( sqdip1 != 0 )
			{
				sqdip1 = 0.5/sqdip1;
			}

			outputQ.data[0] = (matExtracted.at<double>(2, 1) - matExtracted.at<double>(1, 2))*sqdip1;
			outputQ.data[2] = (matExtracted.at<double>(1, 0) + matExtracted.at<double>(0, 1))*sqdip1; 
			outputQ.data[3] = (matExtracted.at<double>(0, 2) + matExtracted.at<double>(2, 0))*sqdip1; 
		}

	}
	matExtracted.release();
	matDiag.release();
	return outputQ;
};
*/

cv::Point3d MatrixToEulerXYZ(cv::Mat inputMat){
	//cv::Mat_<double> outputM = cv::Mat::zeros(3,3,CV_32F);
	//The matrix element 2,0 determines the -sin of the angle theta;
	//This function makes the following Quadrant assumptions:
	//the angle phi should be in some region centered at -90 degrees.
 	//the angle theta is ideally 0.
	//the andle psi is also centered at 0.
	cv::Point3d outputAngles;

	double nSinTheta =  inputMat.at<double>(2,0);

	double sinTheta = -1*nSinTheta;
	outputAngles.y = asin(sinTheta);
	
	double tanPsiN = inputMat.at<double>(1,0);
	double tanPsiD = inputMat.at<double>(0,0);

	//
	double psi = atan2(tanPsiN,tanPsiD);

	double tanPhiN = inputMat.at<double>(2,1);
	double tanPhiD = inputMat.at<double>(2,2);

	double phi = atan2(tanPhiN,tanPhiD);

	outputAngles.x = phi;
	outputAngles.z = psi;

	return outputAngles;
};



//Assume everything is in radians.
cv::Mat EulerXYZToMatrix(cv::Point3d inputEXYZ){
	
	//cv::Mat_<double> outputM = cv::Mat::zeros(3,3,CV_32F);
	double psi, theta, phi;
	phi = inputEXYZ.x;
	theta = inputEXYZ.y;
	psi = inputEXYZ.z;
	cv::Mat outputM = cv::Mat::zeros(3,3,CV_64FC1);
	//First collum
	outputM.at<double>(0,0) = (double) cos(psi)*cos(theta);	
	outputM.at<double>(1,0) = (double) sin(psi)*cos(theta);
	outputM.at<double>(2,0) = (double) -sin(theta);
	//second collum
	outputM.at<double>(0,1) = (double) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
	outputM.at<double>(1,1) = (double) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
	outputM.at<double>(2,1) = (double) cos(theta)*sin(psi);
	//Third collum
	outputM.at<double>(0,2) = (double) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
	outputM.at<double>(1,2) = (double) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi); 
	outputM.at<double>(2,2) = (double) cos(theta)*cos(phi);
	
	return outputM;
};




// Add by DLC for rotate a vector by quaternion 04_21_2014 
cv::Vec3d QuatRotVec(Quaternion q, cv::Vec3d v1){
	
	Quaternion q1, v1q, tempq;
	cv::Vec3d v2;

	double q_Modulus = sqrt(q.data[0]*q.data[0] + q.data[1]*q.data[1] + q.data[2]*q.data[2] + q.data[3]*q.data[3]); 
	
	q1.data[0] = q.data[0]/q_Modulus;
	q1.data[1] = -q.data[1]/q_Modulus;
	q1.data[2] = -q.data[2]/q_Modulus;
	q1.data[3] = -q.data[3]/q_Modulus;

	q.data[0] = q.data[0]/q_Modulus;
	q.data[1] = q.data[1]/q_Modulus;
	q.data[2] = q.data[2]/q_Modulus;
	q.data[3] = q.data[3]/q_Modulus;


	double v1_Modulus = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]); 
	
	v1q.data[0] = 0.0;
	v1q.data[1] = v1[0]/v1_Modulus;
	v1q.data[2] = v1[1]/v1_Modulus;
	v1q.data[3] = v1[2]/v1_Modulus;

	// multquat (v1q, ABBq)
	tempq.data[0] = -v1q.data[1]*q.data[1] - v1q.data[2]*q.data[2] - v1q.data[3]*q.data[3] + v1q.data[0]*q.data[0];

	tempq.data[1] = v1q.data[1]*q.data[0] + v1q.data[2]*q.data[3] - v1q.data[3]*q.data[2] + v1q.data[0]*q.data[1]; 
	tempq.data[2] = -v1q.data[1]*q.data[3] + v1q.data[2]*q.data[0] + v1q.data[3]*q.data[1] + v1q.data[0]*q.data[2]; 
	tempq.data[3] = v1q.data[1]*q.data[2] - v1q.data[2]*q.data[1] + v1q.data[3]*q.data[0] + v1q.data[0]*q.data[3]; 

	// multquat (Abbq_conj,tempq)
	v2[0] = q1.data[1]*tempq.data[0] + q1.data[2]*tempq.data[3] - q1.data[3]*tempq.data[2] + q1.data[0]*tempq.data[1]; 
	v2[1] = -q1.data[1]*tempq.data[3] + q1.data[2]*tempq.data[0] + q1.data[3]*tempq.data[1] + q1.data[0]*tempq.data[2]; 
	v2[2] = q1.data[1]*tempq.data[2] - q1.data[2]*tempq.data[1] + q1.data[3]*tempq.data[0] + q1.data[0]*tempq.data[3];
		

	return v2;
};


double QuatError(Quaternion inputQ1,Quaternion inputQ2){
	
	Quaternion errorQuat =  inputQ2*inputQ1.invert();

	Quaternion Qen(QuatNormalize(errorQuat));

	// the result here is between 0 and 2*pi
	double angleError(2*acos(Qen.data[0]));

	// remap to [-pi, pi].
    if ( abs(angleError) > CV_PI)
	{
	    if ( abs(angleError - 2*CV_PI ) < abs(angleError))
	    {
            angleError -= 2*CV_PI;
		}
	}

	return angleError;
};


cv::Mat InvRotMatrix(cv::Mat inputM){
	
	cv::Mat outputM = cv::Mat::zeros(3,3,CV_64FC1);
	outputM.at<double>(0,0) = inputM.at<double>(0,0);	
	outputM.at<double>(1,0) = inputM.at<double>(0,1);
	outputM.at<double>(2,0) = inputM.at<double>(0,2);
	outputM.at<double>(0,1) = inputM.at<double>(1,0);
	outputM.at<double>(1,1) = inputM.at<double>(1,1);
	outputM.at<double>(2,1) = inputM.at<double>(1,2);
	outputM.at<double>(0,2) = inputM.at<double>(2,0);
	outputM.at<double>(1,2) = inputM.at<double>(2,1);
	outputM.at<double>(2,2) = inputM.at<double>(2,2);

	return outputM;
};

cv::Point3d QuatToEulerXYZ(Quaternion inputQ){
	Quaternion inputQn = QuatNormalize(inputQ);
	
	cv::Mat rotMat = QuatToMatrix(inputQn);

	cv::Point3d eulerXYZOut =  MatrixToEulerXYZ(rotMat);

	return eulerXYZOut;
};



Quaternion EulerXYZToQuat(cv::Point3d inputEXYZ){
	Quaternion outputQ;
	//cv::Mat_<double> outputM = cv::Mat::zeros(3,3,CV_32F);
	cv::Mat tempMat = EulerXYZToMatrix(inputEXYZ);
	outputQ = MatrixToQuat(tempMat);
	return outputQ;

};

cv::Mat Ginv(cv::Mat G){
	cv::Mat G_temp = cv::Mat::zeros(4,4,CV_64FC1);
	for (int i = 0; i<3; i++){
		for (int j = 0; j<3; j++){
			G_temp.at<double>(i,j) = G.at<double>(j,i);
		};
	};
	for (int k = 0; k<3; k++){
		G_temp.at<double>(k,3) = -(G_temp.at<double>(k,0)*G.at<double>(0,3) + G_temp.at<double>(k,1)*G.at<double>(1,3) + G_temp.at<double>(k,2)*G.at<double>(2,3));
	};
	G_temp.at<double>(3,3) = 1;
	return G_temp;
};


};
