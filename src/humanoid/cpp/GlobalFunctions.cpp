/*
 *  GlobalFunctions.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalFunctions.h"
#include <iostream>
using namespace std;

void matrixExpOmegaCross(const Vector3F & omega,Matrix3F & R) {
	Float theta = omega.norm();
	R.setIdentity();
	
	if (theta > 1e-9) {
		Matrix3F omegaHat = cr3(omega/theta);
		//R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
		R+=omegaHat*(sin(theta)*Matrix3F::Identity() + omegaHat*(1-cos(theta)));
	}
}

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
	fZMP(2)-= fZMP(4)*pZMP(0) - fZMP(3)*pZMP(1);
}

void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector6F & a) {
	Vector6F tmp;
	tmp = infoStruct.a + infoStruct.ag;
	ClassicAcceleration(tmp,infoStruct.v, a);
	

	a.swap(tmp); // after this, tmp has classical acceelration
	Float * pa = a.data();
	Float * ptmp = tmp.data();
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa); // Transform rotational acceleration to global coords
	ptmp+=3;
	pa+=3;
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa); // Transform cartesian acceleration to global coords
}

void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector3F & pInterest, Vector6F & a) {
	Vector6F tmp;
	tmp = infoStruct.a + infoStruct.ag;
	
	// add omega cross r to get acceleration of interest point
	tmp.tail(3) += cr3(tmp.head(3))*pInterest;
	
	Vector6F vInterest = infoStruct.v;
	// add omega cross r to get velocity of interest point
	vInterest.tail(3) += cr3(vInterest.head(3))*pInterest;
							 
	ClassicAcceleration(tmp,vInterest, a);
	
	
	a.swap(tmp); // after this, tmp has classical acceelration of interest point
	Float * pa = a.data();
	Float * ptmp = tmp.data();
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa); // Transform rotational acceleration to global coords
	ptmp+=3;
	pa+=3;
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa); // Transform cartesian acceleration to global coords
}



// teach how wxString use streams (operator<<) 
ostream & operator<<(ostream & out, const wxString & str) 
{ 
    out << str.c_str(); 
    return out; 
} 
