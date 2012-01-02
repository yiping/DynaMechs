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

#ifndef _DM_H
#define _DM_H

#define DM_MAJOR_VERSION 4
#define DM_MINOR_VERSION 0

//#define DM_DOUBLE_PRECISION
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

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
typedef Float Quaternion[4];
typedef Float RotationMatrix[3][3];
typedef Float HomogeneousTransformationMatrix[4][4];

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
inline void normalizeQuat(Quaternion quat)
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

inline void buildRotMat(Quaternion quat, RotationMatrix R)
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
inline void buildQuaternion(RotationMatrix R, Quaternion q)
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

#endif
