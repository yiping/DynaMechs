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
 *     File: dmQuaternionLink.hpp
 *   Author: Duane Marhefka
 *  Created: September 2000
 *  Summary: Class definitions for links with (quaternion) ball-joints
 *****************************************************************************/

#ifndef _DM_QUATERNION_LINK_HPP
#define _DM_QUATERNION_LINK_HPP

#include "dm.h"
#include "dmRigidBody.hpp"

//======================================================================

/**
The {\tt dmQuaternionLink} objects are links with three rotational degrees of
freedom, and hence, the class is derived from the {\tt dmRigidBody} (for the
dynamic parameters) and, through that, the {\tt dmLink} base class (required by
{\tt dmArticulation} for Articulated-Body simulation functions.  The use of
quaternions to specify orientation means that there are no singularity issues
with these links.

The {\tt setJointOffset} function is used to set a quaternion link's origin
relative to its predecessor.  This function takes as an argument the
position of the link origin expressed in predecessor link coordinates.  The
{\tt setJointInput} function sets the torque input at the joint.  This function
takes an array of {\tt Float} of length three, with the elements corresponding
to the torque applied through the joint about the link x, y, and z axes.

There are currently no joint limits implemented for this type of joint.

Nearly all of the remainder of the functions are described in the {\tt
dmLink} reference pages and are implemented in this class for the specific
case of links with a full 6 DOF.

See also: {\tt dmRigidBody}, {\tt dmLink}, {\tt dmLoadFile\_dm}  */

class DM_DLL_API dmQuaternionLink : public dmRigidBody
{
public:
   enum {NUM_VARS = 4};

public:
   ///
   dmQuaternionLink();
   ///
   virtual ~dmQuaternionLink() {};

   ///
   inline void setJointOffset(CartesianVector pos)
      {
         for (int i=0; i<3; i++)
            m_p[i] = pos[i];
      }

   ///
   inline int getNumDOFs() const {return NUM_VARS;}
   ///
   void setState(Float q[], Float qd[]);
   ///
   void getState(Float q[], Float qd[]) const;
   ///
   void getPose(RotationMatrix R, CartesianVector p) const;

   ///
   void setJointFriction(Float u_c) {m_joint_friction = u_c;}

   ///
   inline void setJointInput(Float joint_input[])
      {
         for (int i=0; i<3; i++)
            m_joint_input[i] = joint_input[i];
      }

#if 0
// Link-to-link transformation functions:
   ///
   void rtxToInboard(const CartesianVector curr, CartesianVector prev) const;
   ///
   void rtxFromInboard(const CartesianVector prev, CartesianVector curr) const;
   ///
   void stxToInboard(const SpatialVector curr, SpatialVector prev) const;
   ///
   void stxFromInboard(const SpatialVector prev, SpatialVector curr) const;
   ///
   void rcongtxToInboardSym(const CartesianTensor Curr,
                            CartesianTensor Prev) const;
   ///
   void rcongtxToInboardGen(const CartesianTensor Curr,
                            CartesianTensor Prev) const;
#endif
   ///
   void scongtxToInboardIrefl(const SpatialTensor N,
                              SpatialTensor I) const;

   ///
   void XikToInboard(Float **Xik_curr,
                     Float **Xik_prev,
                     int columns_Xik) const;
   ///
   void BToInboard(Float **Bkn,
                   Float **Xik, int cols_Xik,
                   Float **Xin, int cols_Xin ) const;
   ///
   void xformZetak(Float *zetak,
                   Float **Xik, int cols_Xik) const;

// Articulated-Body (AB) algorithm functions:
   ///
   void ABForwardKinematics(Float q[],
                            Float qd[],
                            const dmABForKinStruct &link_val_inboard,
                            dmABForKinStruct &link_val_curr);

   ///
   void ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
                           SpatialVector f_star_curr,
                           SpatialTensor N_refl_curr,
                           SpatialVector f_star_inboard,
                           SpatialTensor N_refl_inboard);
   ///
   void ABBackwardDynamicsN(const dmABForKinStruct &link_val_curr,
                            SpatialVector f_star_inboard,
                            SpatialTensor N_refl_inboard);

   ///
   void ABForwardAccelerations(SpatialVector a_inboard,
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

// Rendering functions:
   ///
   void draw() const;

private:
   // not implemented
   dmQuaternionLink(const dmQuaternionLink &);
   dmQuaternionLink &operator=(dmQuaternionLink &);

   void initABVars();

   inline void matrixInverse3PD(CartesianTensor A,
                                CartesianTensor A_inv);
   inline void setJointPos(dmQuaternion q);

private:
   dmQuaternion m_q;           // orientation quaternion
   Float m_qd[3];            // relative angular velocity wrt i coords.
   Float m_qdd[3];           // acceleration.

   Float m_joint_input[3];

// vars for the AB algorithm member functions.
   Float m_minv[3][3];
   Float m_n_minv[6][3];
   Float m_tau_star[3];
};

#endif
