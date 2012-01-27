/*****************************************************************************
 * DynaMechs: A Multibody Dynamic Simulation Library
 *
 * Copyright (C) 1994-2001  Scott McMillan   All Rights Reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *****************************************************************************
 *     File: dm.h
 *   Author: Scott McMillan
 *  Created: ?/?/1996
 *  Summary: All typedefs, constants and functions needed for DynaMechs
 *****************************************************************************/

/**
 *  \brief   All typedefs, constants and inline functions needed for DynaMechs, etc. file: dm.h
 *  \author  Scott McMillan (founder)
 *  \author  Patrick Wensing 
 *  \author  Yiping Liu
 *  \date    01/25/2012 
*/


#ifndef _DM_H
#define _DM_H

#define DM_MAJOR_VERSION 5
#define DM_MINOR_VERSION 0

#define DM_DOUBLE_PRECISION
//#define DM_HYDRODYNAMICS

#if defined(WIN32) && defined(_DLL)
// The next define will come from the makefile for archive objects.
#ifdef dm_DLL_FILE
#define DM_DLL_API __declspec(dllexport)
#else
#define DM_DLL_API __declspec(dllimport)
#endif
#else
#define DM_DLL_API
#endif

#if defined(WIN32)
#pragma warning (disable : 4786 4251 4661)
#include <windows.h>
#endif

// a bunch of hacks to select standard conforming iostream stuff if available
// on the platform

#if defined(WIN32) || (defined(sgi) && defined(_STANDARD_C_PLUS_PLUS)) || (defined(__GNUC__) && (__GNUC__>=2) && (__GNUC_MINOR__>=91))
#include <iostream>
#include <iomanip>
#include <fstream>
#else
#include <iostream>
#include <iomanip>
#include <fstream>
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>

// ----------------------------------------
// v5.0
#include <Eigen/Core>
#include <vector>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


//-----------------------------------------
// v5.0
#define CROSS_PRODUCT(a,b,c) \
c[0] = a[1]*b[2]-a[2]*b[1]; \
c[1] = -a[0]*b[2]+a[2]*b[0]; \
c[2] = a[0]*b[1]-a[1]*b[0];

#define ADD_CARTESIAN_VECTOR(a,b) \
a[0]+=b[0]; \
a[1]+=b[1]; \
a[2]+=b[2];

using namespace std;
using namespace Eigen;

#ifdef DM_DOUBLE_PRECISION
typedef double Float;   // changing this one line between float and double
                        // toggles the entire lib b/w single/double precision
#else
typedef float Float;
#endif

typedef Float SpatialTensor[6][6];
typedef Float SpatialVector[6];
typedef Float CartesianTensor[3][3];
typedef Float CartesianVector[3];
typedef Float EulerAngles[3];
typedef Float dmQuaternion[4];
typedef Float RotationMatrix[3][3];
typedef Float HomogeneousTransformationMatrix[4][4];

//-----------------------------------------
// v5.0
typedef Eigen::Matrix<Float, 6, 6 > Matrix6F;
typedef Eigen::Matrix<Float, 6, 1 > Vector6F;
typedef Eigen::Matrix<Float, 3, 3 > Matrix3F;
typedef Eigen::Matrix<Float, 3, 1 > Vector3F;

typedef Matrix<Float, Dynamic , 1 > VectorXF;
typedef Matrix<Float, Dynamic, Dynamic> MatrixXF;
typedef Matrix<Float, Dynamic,6> MatrixX6F;
typedef Matrix<Float, 6, Dynamic> Matrix6XF;

typedef struct CRBI {
	CartesianTensor IBar;
	CartesianVector h;
	Float m;
} CrbInertia;


//----------------------------------------------------------------------------
// Summary: struct containing variables used in ABForwardKinematics for a
//          single link.
//----------------------------------------------------------------------------
struct dmABForKinStruct
{
   RotationMatrix  R_ICS;     // pose of links wrt ICS - ^{ICS}R_{i}
   CartesianVector p_ICS;     // pos. of links wrt ICS - ^{ICS}p_{i}
   SpatialVector   v;         // velocity of link wrt to i.

#ifdef DM_HYDRODYNAMICS
   CartesianVector v_f;       // velocity of fluid wrt to i.
   CartesianVector a_fg;      // a_f - a_g wrt to i.
#endif
};

// Use the std namespace. To do this we must first guarantee that it exists.
#if defined(__sgi) || defined(__WIN32_) || defined(WIN32)
namespace std {}
using namespace std;
#endif

//----------------------------------------------------------------------------
inline void normalizeQuat(dmQuaternion quat)
{
   Float len = sqrt(quat[0]*quat[0] +
                    quat[1]*quat[1] +
                    quat[2]*quat[2] +
                    quat[3]*quat[3]);
   //cerr << len << endl;
   if (len > 0.000001)
   {
      quat[0] /= len;
      quat[1] /= len;
      quat[2] /= len;
      quat[3] /= len;
   }
   else
   {
      // This is an error condition
      cerr << "Warning: normalizing zero quaternion: [" << quat[0] << ", "
           << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << endl;
      quat[0] = quat[1] = quat[2] = 0.0;
      quat[3] = 1.0;
   }
}

// ---------------------------------------------------------------------

inline void buildRotMat(dmQuaternion quat, RotationMatrix R)
{
   // set the transformation matrix
   static Float q1,q2,q3, q1q1,q2q2,q3q3, q1q2,q1q3,q2q3, q1q4,q2q4,q3q4;

   q1 = 2*quat[0];
   q2 = 2*quat[1];
   q3 = 2*quat[2];

   q1q1 = q1*quat[0];
   q2q2 = q2*quat[1];
   q3q3 = q3*quat[2];

   q1q2 = q1*quat[1];
   q1q3 = q1*quat[2];
   q2q3 = q2*quat[2];

   q1q4 = q1*quat[3];
   q2q4 = q2*quat[3];
   q3q4 = q3*quat[3];

   R[0][0] = 1.0 - (q3q3 + q2q2);
   R[1][0] = q1q2 - q3q4;
   R[2][0] = q1q3 + q2q4;

   R[0][1] = q1q2 + q3q4;
   R[1][1] = 1.0 - (q3q3 + q1q1);
   R[2][1] = q2q3 - q1q4;

   R[0][2] = q1q3 - q2q4;
   R[1][2] = q2q3 + q1q4;
   R[2][2] = 1.0 - (q2q2 + q1q1);
}

//----------------------------------------------------------------------------
inline void buildQuaternion(RotationMatrix R, dmQuaternion q)
{
   Float trace = R[0][0] + R[1][1] + R[2][2];
   if (trace > 0.)
   {
      Float s = (Float)sqrt(trace + 1.);
      q[3] = (Float)(s*0.5);
      s = 0.5/s;
      q[0] = (Float)((R[2][1] - R[1][2])*s);
      q[1] = (Float)((R[0][2] - R[2][0])*s);
      q[2] = (Float)((R[1][0] - R[0][1])*s);
   }
   else // negative or zero trace
   {
      unsigned int ix = 0;
      if (R[1][1] > R[0][0]) ix = 1;
      if (R[2][2] > R[1][1]) ix = 2;
      unsigned int iy = (ix+1)%3;
      unsigned int iz = (iy+1)%3;

      Float s = (Float)sqrt((R[ix][ix] - (R[iy][iy] + R[iz][iz])) + 1.);
      q[ix] = (Float)(s*0.5);

      if (s != 0.) s = 0.5/s;
      q[3]  = (Float)((R[iz][iy] - R[iy][iz])*s);
      q[iy] = (Float)((R[iy][ix] + R[ix][iy])*s);
      q[iz] = (Float)((R[iz][ix] + R[ix][iz])*s);
   }
}

// ---------------------------------------------------------------------
// Function : crossproduct
// Purpose  : perform the Cartesian vector crossproduct a x b = c
// Inputs   : a, b
// Outputs  : c
// ---------------------------------------------------------------------
inline void crossproduct(const CartesianVector a,
                         const CartesianVector b,
                         CartesianVector c)
{
   c[0] = a[1]*b[2] - a[2]*b[1];
   c[1] = a[2]*b[0] - a[0]*b[2];
   c[2] = a[0]*b[1] - a[1]*b[0];
}

//----------------------------------------------------------------------------
inline Float normalize(CartesianVector v)
{
    float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    if (norm > 0.0)
    {
       v[0] /= norm;
       v[1] /= norm;
       v[2] /= norm;
    }

    return norm;
}





//-----------------------------------------------------------------------------
// v5.0


//-----
inline void doubleCrossProdMat(const CartesianVector p1, const CartesianVector p2, CartesianTensor pCross)
{
	const Float p10 = p1[0], p11 = p1[1], p12 = p1[2];
	const Float p20 = p2[0], p21 = p2[1], p22 = p2[2];
	
	pCross[0][0] = -p11*p21-p12*p22;	pCross[0][1] = p11*p20;				pCross[0][2] = p12*p20;
	pCross[1][0] = p10*p21;				pCross[1][1] = -p10*p20-p12*p22;	pCross[1][2] = p12*p21;
	pCross[2][0] = p10*p22;				pCross[2][1] = p11*p22;				pCross[2][2] = -p10*p20-p11*p21;
}

//------
inline void CrbCopy(CrbInertia &A, CrbInertia &B) {
	B.m = A.m;
	for (int i=0; i<3; i++) {
		B.h[i] = A.h[i];
		for (int j =0; j<3; j++) {
			B.IBar[i][j] = A.IBar[i][j];
		}
	}
}

//-----
inline void CrbAdd(CrbInertia & a, CrbInertia & b)
{
	a.m += b.m;
	for(int i=0 ; i<3;i++) {
		a.h[i] += b.h[i];
		for (int j=0; j<3; j++) {
			a.IBar[i][j] += b.IBar[i][j];
		}
	}
}


/*! 
Spatial cross product for forces.
*/
//! DM 5.0 function 
/*!
\f$ { \dot{\bf f} } = {\bf v} \times^* {\bf f}\f$
*/
/*!
\param v the spatial velocity of the coordinate.
\param F1 the force vector.
\param F2   the derivative of F1 (output)
\sa CrossSpV()
*/
inline void CrossSpF(Vector6F &v, Vector6F &F1, Vector6F &F2) {
	Float * f1 = F1.data();
	Float * f2 = F2.data();
	Float * vOf = v.data();
	Float * vv  = vOf + 3;
	
	CartesianVector tmp; //type def in dm.h

	Float * f12 = f1+3;
	
	CROSS_PRODUCT(vOf,f1,tmp);
	CROSS_PRODUCT(vv,f12,f2);
	ADD_CARTESIAN_VECTOR(f2,tmp);
	
	f2+=3;
	
	CROSS_PRODUCT(vOf,f12,f2);
}

/*! 
Spatial cross product for motions.
*/
//! DM 5.0 function 
/*!
\f$ { \dot{\bf m} } = {\bf v} \times {\bf m}\f$
*/
/**
\param v  the spatial velocity of the coordinate.
\param M1  the motion vector.
\param M2  the derivative of M1 (output)
\sa CrossSpF()
*/
inline void CrossSpV(Vector6F &v, Vector6F & M1, Vector6F &M2) {
	Float * m1 = M1.data();
	Float * m2 = M2.data();
	Float * vOm = v.data();
	Float * vv  = vOm + 3;
	
	CartesianVector tmp;
	
	CROSS_PRODUCT(vOm,m1,m2);
	m2+=3;
	Float * m12 = m1+3;
	
	CROSS_PRODUCT(vv,m1,tmp);
	CROSS_PRODUCT(vOm,m12,m2);
	ADD_CARTESIAN_VECTOR(m2,tmp);
}





/*!
 crm  spatial cross-product operator (motion).
 crm(v) calculates the 6x6 matrix such that the expression crm(v)*m is the
 cross product of the spatial motion vectors v and m.
*/
//! DM 5.0 function 
/**
\param v  spatial motion vector.
\return the 6x6 matrix
\sa crf()
*/

inline Matrix6F crm(Vector6F &v)
{
	Matrix6F vcross;  
	vcross <<   0,    -v(2),  v(1),    0,     0,     0,    
		  v(2),      0,  -v(0),    0,     0,     0,    
		-v(1),     v(0),    0,     0,     0,     0,
		    0,    -v(5),  v(4),    0,  -v(2),   v(1),
		  v(5),      0,  -v(3),  v(2),    0,   -v(0),
		 -v(4),    v(3),    0,  -v(1),  v(0),     0;

	return vcross;
}



/*!
 crf  spatial cross-product operator (force).
 crf(v) calculates the 6x6 matrix such that the expression crf(v)*f is the
 cross product of the spatial motion vector v with the spatial force vector f.
*/
//! DM 5.0 function 
/**
\param v  spatial force vector.
\return the 6x6 matrix
\sa crm()
*/
inline Matrix6F crf(Vector6F &v)
{
	Matrix6F vcross;
	vcross = -crm(v).transpose();
	return vcross;
}


/*!
 cr3  cross-product operator for 3d vector.
*/
//! DM 5.0 function 
/**
\param v  3d vector.
\return the 3x3 matrix (Eigen matrix)
\sa crm()
*/
inline Matrix3F cr3(Vector3F &v)
{
	Matrix3F vcross;  
	vcross <<   0,    -v(2),  v(1),    
		  v(2),      0,  -v(0),       
		-v(1),     v(0),    0;
	return vcross;
}


#endif
