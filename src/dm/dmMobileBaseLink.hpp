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
 *     File: dmMobileBaseLink.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for a 6dof link (used to be the
 *           dmDynamicRefMember class.
 *****************************************************************************/

#ifndef _DM_MOBILE_BASE_LINK_HPP
#define _DM_MOBILE_BASE_LINK_HPP

#include "dm.h"
#include "dmRigidBody.hpp"

//======================================================================

/**

The  dmMobileBaseLinks  objects are links with six degrees of freedom, and
hence, the class is derived from the  dmRigidBody (for the dynamic
parameters) and, through that, the  dmLink  base class (required by
dmArticulation  for Articulated-Body simulation functions.  The default
constructor instantiates a generic mobile base with some valid inertial
parameters and zero state variables, but subsequent calls to this and the
dmRigidBody  member functions are needed to set the desired values.

Note that joint inputs and friction are currently ignored for this type of
link.

Nearly all of the remainder of the functions are described in the
dmLink  reference pages and are implemented in this class for the specific
case of links with a full 6 DOF.

See also:  dmRigidBody ,  dmLink , \b dmLoadFile_dm */

class DM_DLL_API dmMobileBaseLink : public dmRigidBody
{
public:
   enum {NUM_TRUE_DOFS = 6, NUM_VARS = 7};

public:
   ///
   dmMobileBaseLink();
   ///
   virtual ~dmMobileBaseLink();

   ///
   virtual int getNumDOFs() const { return NUM_VARS; }
	inline int getTrueNumDOFs() const { return NUM_TRUE_DOFS; }
	
   ///
   virtual void setState(Float q[], Float qd[]);
   ///
   virtual void getState(Float q[], Float qd[]) const;
   ///
   virtual void getPose(RotationMatrix R, CartesianVector p) const;

   ///
   void setJointInput(Float[]) {};

   ///
   void getDeriv(Float rdy[13]);

// Link-to-Link transformation functions:
   /// *
   void rtxToInboard(const CartesianVector curr, CartesianVector prev) const;
   /// *
   void rtxFromInboard(const CartesianVector prev, CartesianVector curr) const;
   /// not implemented
   void stxToInboard(const SpatialVector curr, SpatialVector prev) const;
   /// not implemented
   void stxFromInboard(const SpatialVector prev, SpatialVector curr) const;
   /// not implemented
   void rcongtxToInboardSym(const CartesianTensor Curr,
                            CartesianTensor Prev) const;
   /// not implemented
   void rcongtxToInboardGen(const CartesianTensor Curr,
                            CartesianTensor Prev) const;
   /// not implemented
   void scongtxToInboardIrefl(const SpatialTensor N, SpatialTensor I) const;
   ///
   void XikToInboard(Float **Xik_curr,
                     Float **Xik_prev,
                     int columns_Xik) const;
   ///
   void BToInboard(Float **Bkn,
                   Float **Xik, int cols_Xik,
                   Float **Xin, int cols_Xin) const;
   ///
   void xformZetak(Float *zetak,
                   Float **Xik, int cols_Xik) const;

   //! v5.0 function
   void jcalc(Matrix6XF &);


// Articulated-Body (AB) algorithm functions:
   ///
   virtual void ABForwardKinematics(Float q[],
                                    Float qd[],
                                    const dmABForKinStruct &link_val_inboard,
                                    dmABForKinStruct &link_val_curr);

   ///
   virtual void ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
                                   SpatialVector f_star_curr,
                                   SpatialTensor N_refl_curr,
                                   SpatialVector f_star_inboard,
                                   SpatialTensor N_refl_inboard);
   ///
   virtual void ABBackwardDynamicsN(const dmABForKinStruct &link_val_curr,
                                    SpatialVector f_star_inboard,
                                    SpatialTensor N_refl_inboard);

   ///
   virtual void ABForwardAccelerations(SpatialVector a_inboard,
                                       SpatialVector a_curr,
                                       Float qd[],
                                       Float qdd[]);
   ///
   void ABForwardAccelerations(SpatialVector a_inboard,
                               unsigned int *LB,
                               unsigned int num_elements_LB,
                               Float ***Xik,
                               Float **constraint_forces,
                               unsigned int *num_constraints,
                               SpatialVector a_curr,
                               Float qd[],
                               Float qdd[]);
	
	//------- v5.0 functions
	void RNEAOutwardFKID(  dmRNEAStruct &link_val2_curr, 
										   dmRNEAStruct &link_val2_inboard,
										   bool ExtForceFlag);
	
	void RNEAOutwardFKIDFirst(  dmRNEAStruct &link_val2_curr,
												CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
												RotationMatrix  R_ref_ICS,  
												const Vector6F &a_ini, 
												const Vector6F &v_ini,
							  bool ExtForceFlag);
	
	/*void RNEAInwardID(dmRNEAStruct &link_val2_curr,
					  dmRNEAStruct &link_val2_inboard);*/
	

// rendering functions:
   ///
   void draw() const;
   void drawSkeleton(bool isTip) const;
private:
   // not implemented
   dmMobileBaseLink(const dmMobileBaseLink &);
   dmMobileBaseLink &operator=(const dmMobileBaseLink &);

   void initABVars() {};
   inline void setJointPos(dmQuaternion q, CartesianVector p);

private:
   dmQuaternion      m_quat;     // orientation quaternion
                               //   rotation matrix, ^i{\bf R}_{i-1} and
                               //   position stored in dmLink::m_p

   SpatialVector   m_vel;      // velocity state variables
   //SpatialVector   m_acc;      // rate of change of velocity state variables
};

//----------------------------------------------------------------------------
//    Summary: 3D rotational transformation from link CS to inboard CS
// Parameters: curr - 3-vector expressed in link CS
//    Returns: prev - 3-vector expressed in inboard link CS
//----------------------------------------------------------------------------
inline void dmMobileBaseLink::rtxToInboard(const CartesianVector curr,
                                           CartesianVector prev) const
{
   prev[0] = m_R[0][0]*curr[0] + m_R[1][0]*curr[1] + m_R[2][0]*curr[2];
   prev[1] = m_R[0][1]*curr[0] + m_R[1][1]*curr[1] + m_R[2][1]*curr[2];
   prev[2] = m_R[0][2]*curr[0] + m_R[1][2]*curr[1] + m_R[2][2]*curr[2];
}

//----------------------------------------------------------------------------
//    Summary: 3D rotational transformation from inboard link CS to link CS
// Parameters: prev - 3-vector expressed in the inboard link CS
//    Returns: curr - 3-vector expressed in the link CS
//----------------------------------------------------------------------------
inline void dmMobileBaseLink::rtxFromInboard(const CartesianVector prev,
                                             CartesianVector curr) const
{
   curr[0] = m_R[0][0]*prev[0] + m_R[0][1]*prev[1] + m_R[0][2]*prev[2];
   curr[1] = m_R[1][0]*prev[0] + m_R[1][1]*prev[1] + m_R[1][2]*prev[2];
   curr[2] = m_R[2][0]*prev[0] + m_R[2][1]*prev[1] + m_R[2][2]*prev[2];
}

#endif
