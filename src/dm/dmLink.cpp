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
 *     File: dmLink.cpp
 *   Author: Scott McMillan
 *  Summary: Class implementation of Link (abstract base class for all links)
 *****************************************************************************/

#include "dm.h"
#include "dmObject.hpp"
#include "dmLink.hpp"

//============================================================================
// class dmLink
//============================================================================

//----------------------------------------------------------------------------
//    Summary: default class constructor
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmLink::dmLink()
      : dmObject(),
        m_joint_limit_flag(false),
        m_joint_limit_spring(0.0),
        m_joint_limit_damper(0.0),
        m_joint_friction(0.0)
{
   m_p[0] = m_p[1] = m_p[2] = 0.0;
}

//----------------------------------------------------------------------------
//    Summary: dmLink class destructor
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmLink::~dmLink()
{
}

//----------------------------------------------------------------------------
//    Summary: perform one step of a forward kinematics recursion to compute
//             the position and orientation of the link in the ICS
// Parameters: link_val_inboard - struct containing the position (p_ICS) and
//                                orientation (R_ICS) of the parent (inboard)
//                                link.
//    Returns: link_val_curr - struct containing the computed position (p_ICS)
//                                and orientation (R_ICS) of this link.
//----------------------------------------------------------------------------
void dmLink::forwardKinematics(dmABForKinStruct &link_val_inboard,
                               dmABForKinStruct &link_val_curr) const
{
// Compute R_ICS and p_ICS for this link's coordinate system.
   for (int i = 0; i < 3; i++)
   {
      link_val_curr.p_ICS[i] = link_val_inboard.p_ICS[i];
      for (int j = 0; j < 3; j++)
      {
         link_val_curr.p_ICS[i] += link_val_inboard.R_ICS[i][j]*m_p[j];
      }

      rtxFromInboard(&(link_val_inboard.R_ICS[i][0]),
                     &(link_val_curr.R_ICS[i][0]));
   }
}



// ---------------------------------------------------------------------
// Function : rtxToInboard
// Purpose  : rotate a 3d vector from this link's CS to inboard CS
// Inputs   : curr - 3d vector in this link's CS
// Outputs  : prev - 3d vector in inboard link's CS
// ---------------------------------------------------------------------
void dmLink::rtxToInboard(const CartesianVector curr,
                          CartesianVector prev) const
{
   prev[0] = m_R[0][0]*curr[0] + m_R[1][0]*curr[1] + m_R[2][0]*curr[2];
   prev[1] = m_R[0][1]*curr[0] + m_R[1][1]*curr[1] + m_R[2][1]*curr[2];
   prev[2] = m_R[0][2]*curr[0] + m_R[1][2]*curr[1] + m_R[2][2]*curr[2];
}


// ---------------------------------------------------------------------
// Function : rtxFromInboard
// Purpose  : rotate a 3d vector from inboard link's CS to this CS
// Inputs   : prev - 3d vector in inboard CS
// Outputs  : curr - 3d vector in this CS
// ---------------------------------------------------------------------
void dmLink::rtxFromInboard(const CartesianVector prev,
                            CartesianVector curr) const
{
   curr[0] = m_R[0][0]*prev[0] + m_R[0][1]*prev[1] + m_R[0][2]*prev[2];
   curr[1] = m_R[1][0]*prev[0] + m_R[1][1]*prev[1] + m_R[1][2]*prev[2];
   curr[2] = m_R[2][0]*prev[0] + m_R[2][1]*prev[1] + m_R[2][2]*prev[2];
}


// ---------------------------------------------------------------------
// Function : stxToInboard
// Purpose  : Spatial transform of 6d vector to inboard CS
// Inputs   : curr - 6d vector in this link's CS
// Outputs  : prev - 6d vector in inboard link's CS
// ---------------------------------------------------------------------
void dmLink::stxToInboard(const SpatialVector curr,
                          SpatialVector prev) const
{
   rtxToInboard(&curr[3], &prev[3]);

   CartesianVector temp;
   rtxToInboard(&curr[0], temp);
   prev[0] = temp[0] - m_p[2]*prev[4] + m_p[1]*prev[5];
   prev[1] = temp[1] + m_p[2]*prev[3] - m_p[0]*prev[5];
   prev[2] = temp[2] - m_p[1]*prev[3] + m_p[0]*prev[4];
}


// ---------------------------------------------------------------------
// Function : stxFromInboard
// Purpose  : Spatial transform of 6d vector from inboard CS
// Inputs   : prev - 6d vector wrt inboard link's CS
// Outputs  : curr - 6d vector wrt this link's CS
// ---------------------------------------------------------------------
void dmLink::stxFromInboard(const SpatialVector prev,
                            SpatialVector curr) const
{
   rtxFromInboard(&prev[0], &curr[0]);

   CartesianVector temp;
   temp[0] = + m_p[2]*prev[1] - m_p[1]*prev[2] + prev[3];
   temp[1] = - m_p[2]*prev[0] + m_p[0]*prev[2] + prev[4];
   temp[2] = + m_p[1]*prev[0] - m_p[0]*prev[1] + prev[5];
   rtxFromInboard(temp, &curr[3]);
}


// ---------------------------------------------------------------------
// Function : rcongtxToInboardSym
// Purpose  : Congruence transform of 3x3 sym matric to inboard link CS
// Inputs   : Curr - 3x3 symmetric matrix wrt this CS
// Outputs  : Prev - 3x3 symmetric matrix wrt inboard CS.
// ---------------------------------------------------------------------
void dmLink::rcongtxToInboardSym(const CartesianTensor Curr,
                                 CartesianTensor Prev) const
{
   register unsigned int i, j;
   CartesianTensor Temp;

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         Temp[i][j] = ( Curr[i][0]*m_R[0][j] + Curr[i][1]*m_R[1][j] +
                        Curr[i][2]*m_R[2][j] );

   for (i = 0; i < 3; i++)
      for (j = i; j < 3; j++)
         Prev[i][j] = Prev[j][i] = ( m_R[0][i]*Temp[0][j] +
                                     m_R[1][i]*Temp[1][j] +
                                     m_R[2][i]*Temp[2][j] );
}


// ---------------------------------------------------------------------
// Function : rcongtxToInboardGen
// Purpose  : Congruence transform of general 3x3 matrix to inboard CS.
// Inputs   : Curr - 3x3 general matrix in this coordinate system
// Outputs  : Prev - 3x3 general matrix in inboard CS
// ---------------------------------------------------------------------
void dmLink::rcongtxToInboardGen(const CartesianTensor Curr,
                                 CartesianTensor Prev) const
{
   register unsigned int i, j;
   CartesianTensor Temp;

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         Temp[i][j] = ( Curr[i][0]*m_R[0][j] + Curr[i][1]*m_R[1][j] +
                        Curr[i][2]*m_R[2][j] );

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         Prev[i][j] = ( m_R[0][i]*Temp[0][j] + m_R[1][i]*Temp[1][j] +
                        m_R[2][i]*Temp[2][j] );
}


//----------------------------------------------------------------------------
//    Summary: perform the spatial congruence tranformation to the inboard link
// Parameters: Curr - AB inertia reflected across the joint
//    Returns: Prev - Curr transformed to inboard link's CS.
//----------------------------------------------------------------------------
void dmLink::scongtxToInboardIrefl(const SpatialTensor Curr,
                                   SpatialTensor Prev) const
{
   register unsigned int i, j;

   //  N = [ N11  N21^T
   //        N21  N22  ]

   CartesianTensor N11, N21, N22, N11_r, N21_r, N22_r;

   for (i = 0; i < 3; i++)
      for (j = i; j < 3; j++)
         N11[i][j] = N11[j][i] = Curr[i][j];
   rcongtxToInboardSym(N11, N11_r);  // N11 is symmetric

   for (i = 0; i < 3; i++)
      for (j = i; j < 3; j++)
         N22[i][j] = N22[j][i] = Curr[i + 3][j + 3];
   rcongtxToInboardSym(N22, N22_r);  // N22 is symmetric

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         N21[i][j] =  Curr[i + 3][j];
   rcongtxToInboardGen(N21, N21_r);  // N21 is NOT symmetric


   CartesianTensor rtilde_N21r;
   for (j = 0; j < 3; j++) {
      rtilde_N21r[0][j] = - m_p[2]*N21_r[1][j] + m_p[1]*N21_r[2][j];
      rtilde_N21r[1][j] = + m_p[2]*N21_r[0][j] - m_p[0]*N21_r[2][j];
      rtilde_N21r[2][j] = - m_p[1]*N21_r[0][j] + m_p[0]*N21_r[1][j];
   }

   CartesianTensor N22r_rtildeT;
   for (i = 0; i < 3; i++) {
      N22r_rtildeT[i][0] = - ( + N22_r[i][1]*m_p[2] - N22_r[i][2]*m_p[1] );
      N22r_rtildeT[i][1] = - ( - N22_r[i][0]*m_p[2] + N22_r[i][2]*m_p[0] );
      N22r_rtildeT[i][2] = - ( + N22_r[i][0]*m_p[1] - N22_r[i][1]*m_p[0] );
   }

   // Only need to compute diagonal and below.
   CartesianTensor rtilde_N22r_rtildeT;
   rtilde_N22r_rtildeT[0][0] = ( - m_p[2]*N22r_rtildeT[1][0]
                                 + m_p[1]*N22r_rtildeT[2][0] );
   rtilde_N22r_rtildeT[1][0] = ( + m_p[2]*N22r_rtildeT[0][0]
                                 - m_p[0]*N22r_rtildeT[2][0] );
   rtilde_N22r_rtildeT[2][0] = ( - m_p[1]*N22r_rtildeT[0][0]
                                 + m_p[0]*N22r_rtildeT[1][0] );
   rtilde_N22r_rtildeT[1][1] = ( + m_p[2]*N22r_rtildeT[0][1]
                                 - m_p[0]*N22r_rtildeT[2][1] );
   rtilde_N22r_rtildeT[2][1] = ( - m_p[1]*N22r_rtildeT[0][1]
                                 + m_p[0]*N22r_rtildeT[1][1] );
   rtilde_N22r_rtildeT[2][2] = ( - m_p[1]*N22r_rtildeT[0][2]
                                 + m_p[0]*N22r_rtildeT[1][2] );


   for (i = 0; i < 3; i++) {
      for (j = 0; j <= i; j++) {
         Prev[i][j] = ( N11_r[i][j] + rtilde_N21r[i][j] + rtilde_N21r[j][i]
                        + rtilde_N22r_rtildeT[i][j] );
         Prev[i+3][j+3] = N22_r[i][j];
      }

      for (j = 0; j < 3; j++)
         Prev[i+3][j] = N21_r[i][j] + N22r_rtildeT[i][j];
   }

   // Fill out upper diagonal using symmetry.
   for (i = 0; i < 5; i++)
      for (j = i+1; j < 6; j++)
         Prev[i][j] = Prev[j][i];
}


//---------------------------------------------------------
void dmLink::scongxToInboardIcomp(const CrbInertia & IC_curr, CrbInertia & IC_prev) const
{
	IC_prev.m = IC_curr.m;
	
	rcongtxToInboardSym(IC_curr.IBar, IC_prev.IBar);
	CartesianVector RTh;
	rtxToInboard(IC_curr.h, RTh);
	CartesianTensor C1, C2;
	
	//cout << "RTH\tmp" << endl;
	for (int i=0; i<3; i++) {
		//cout << RTh[i] << "\t" <<  cur.m * X.pi_p_i[i] << endl;
		IC_prev.h[i] = RTh[i] + IC_curr.m * m_p[i];
	}
	
	doubleCrossProdMat(m_p, RTh, C1);
	doubleCrossProdMat(m_p, m_p, C2);
	
	for (int i =0 ; i < 3 ; i++) {
		for (int j=0; j<3; j++) {
			IC_prev.IBar[i][j] -= C1[i][j] + C1[j][i] + IC_curr.m*C2[i][j];
		}
	}
}

//-------------------------
void dmLink::stxToInboardMat(const Matrix6XF& curr, Matrix6XF & prev) const
{
	const int c = curr.cols();
	prev.resize(6,curr.cols());
	
	const Float * cPtr = curr.data();
	Float * pPtr = prev.data();
	for (int i=0; i<c; i++) {
		stxToInboard(cPtr,pPtr);
		cPtr +=6;
		pPtr +=6;
	}
}

//--
void dmLink::initializeCrbInertia(CrbInertia & IC_curr) const
{
	IC_curr.m =0;
	for (int i=0; i<3; i++) {
		IC_curr.h[i]=0;
		for (int j=0; j<3; j++) {
			IC_curr.IBar[i][j]=0;
		}
	}
}


//-------------------------------------------------------------
Matrix6F dmLink::get_X_FromParent_Motion()
{
    //dummy implementation
    Matrix6F X = Matrix6F::Zero();
    return X;
}

Matrix6XF dmLink::jcalc()
{
    //dummy implementation
    Matrix6XF  m(6,1);
    m << 0, 0, 1, 0, 0, 0;
    return m;
}

//-------------------------------------------------------------


void dmLink::RNEAOutwardFKID(dmRNEAStruct &link_val2_curr, 
                                      dmRNEAStruct &link_val2_inboard,
                                                     bool ExtForceFlag)
{
        //dummy
}

void dmLink::RNEAOutwardFKIDFirst(dmRNEAStruct &link_val2_curr, 
					  CartesianVector  p_ref_ICS,  
                                          RotationMatrix  R_ref_ICS, 
                                          Vector6F a_ini, Vector6F v_ini,
                                                     bool ExtForceFlag)
{
	//dummy
}

void dmLink::RNEAInwardID( dmRNEAStruct &link_val2_curr,
                            dmRNEAStruct &link_val2_inboard)
{
	//dummy
}

//--------------------------------------------------------------------
void dmLink::compute_AccBias_First(dmRNEAStruct &link_val2_curr)
{
	//dummy

}

//--------------------------------------------------------------------
void dmLink::compute_AccBias(dmRNEAStruct &link_val2_curr,
                                         dmRNEAStruct &link_val2_inboard)
{
	//dummy
}

//---------------------------------------------------------------------
//! DM v5.0 function,
void dmLink::computeSpatialVelAndICSPoseFirst(  dmRNEAStruct &link_val2_curr,
                                       CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
                                       RotationMatrix  R_ref_ICS,
                                          Vector6F a_ini)
{
	//dummy
}


//----------------------------------------------------------------------
void dmLink::computeSpatialVelAndICSPose(  dmRNEAStruct &link_val2_curr,
                                         dmRNEAStruct &link_val2_inboard)
{
	//dummy
}
