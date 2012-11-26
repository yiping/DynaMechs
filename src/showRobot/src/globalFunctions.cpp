//  globalFunctions.cpp


#include "globalFunctions.h"
#include <iostream>
using namespace std;

//Exponential map from so(3) to SO(3)
//transformation from axis-angle representation of rotations to rotation matrices.
void matrixExpOmegaCross(const Vector3F & omega,Matrix3F & R) 
{
	//Rodrigues' rotation formula
	Float theta = omega.norm();
	Matrix3F omegaHat = cr3(omega/theta);
	
	//R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
	R.setIdentity();
	R+=omegaHat*(sin(theta)*Matrix3F::Identity() + omegaHat*(1-cos(theta)));
}


//Log map from SO(3) to so(3)
//To retrieve the axis-angle representation of a rotation matrix calculate the angle of rotation:
void matrixLogRot(const Matrix3F & R, Vector3F & omega) {
	// theta = acos( (Trace(R) - 1)/2 )
	Float theta;
	Float tmp = (R(0,0) + R(1,1) + R(2,2) - 1) / 2;
	if (tmp >=1.) {
		theta = 0;
	}
	else if (tmp <=-1.) {
		theta = M_PI;
	}
	else {
		theta = acos(tmp);
	}

	//normalized axis:
	//   1 / (2 * sin(theta)) * [R(2,1)-R(1,2) , R(0,2)-R(2,0), R(1,0)-R(0,1)]'
	
	//Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
	//crossExtract(omegaHat,omega);

	omega << R(2,1)-R(1,2) , R(0,2)-R(2,0),R(1,0)-R(0,1); 	
	if (theta > 10e-5) {
		omega*=theta / (2*sin(theta));
	}
	else {
		omega/=2;
	}
}

void copyRtoMat(const CartesianTensor R, Matrix3F & Rmat) {
	Rmat << R[0][0] , R[0][1], R[0][2] , 
	R[1][0] , R[1][1], R[1][2] ,
	R[2][0] , R[2][1], R[2][2] ;
}

double timeDiff(const dmTimespec & t1, const dmTimespec & t2) {
	return ((double) t2.tv_sec - t1.tv_sec) + (1.0e-9*((double) t2.tv_nsec - t1.tv_nsec));
}


void transformToZMP(Vector6F & fZMP, Vector3F & pZMP) {
	const Float nx = fZMP(0), ny = fZMP(1), fz = fZMP(5);
	
	pZMP(0) = - ny/fz;
	pZMP(1) =   nx/fz;
	pZMP(2) = 0;
	
	fZMP(0) = 0;
	fZMP(1) = 0;
	fZMP(2)+= fZMP(4)*pZMP(0) - fZMP(3)*pZMP(1);
}


//! Be careful, this function only makes sense when used after dmArticulation::computeCandG()
void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector6F & a) 
{
	// assuming dmArticulation::computeCandG() has been called first
	Vector6F tmp;
	tmp = infoStruct.a + infoStruct.ag;		// ag (in local coor.)  
	ClassicAcceleration(tmp,infoStruct.v, a);	// in dm.h
	// ( aSpatial, vSpatial --> aClassic )	 [6D]
	a.swap(tmp);
	Float * pa = a.data();
	Float * ptmp = tmp.data();
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa);
	ptmp+=3;
	pa+=3;
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa);
}







