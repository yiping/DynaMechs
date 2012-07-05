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
 *     File: dmSphericalLink.hpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for links with 3dof ball-joints
 *****************************************************************************/

#ifndef _DM_SPHERICAL_LINK_HPP
#define _DM_SPHERICAL_LINK_HPP

#include "dm.h"
#include "dmRigidBody.hpp"
#include "dmLink.hpp"

//======================================================================
/**

This class is a concrete link class (previously called the BallnSocketLink
class) that models a rigid body attached to an articulation with a ball joint
(three rotational degrees of freedom).  This class is derived from
the {\tt dmRigidBody} (for the dynamic parameters) and, through that, the {\tt
dmLink} base class (required by {\tt dmSystem} for Articulated-Body simulation
functions).

The kinematics (the point of motion) of these links can be defined with a
single cartesian vector that specifies the constant offset from the origin of
the previous coordinate system (expressed with respect to the previous
coordinate system).  This can be set by passing the desired offset to the {\tt
setJointOffset} member function.  Note that currently the default orientation
of this link's coordinate system in zero position (all joint angles set to
zero) is parallel to the orientation of the previous coordinate system.

The orientation of this link with respect to the inboard coordinate system is
specified with three Euler angles. {\em I am considering converting this to a
quaternion just like I did to the {\tt dmRefMember} class.}  The values of
these angles can be set and retrieved with the {\tt setState} and {\tt
getState} member functions where the {\tt q} parameter contains the angles
\f$\phi, \theta, \psi\f$ order.  They are applied, however, in the following order
from the inboard to this coordinate system:
\f[
\begin{tabular}{lcl}
$\psi$ & ~~ & angle in radians about the inboard $z$ axis, \\
$\theta$  & & angle in radians about the rotated $y$ axis, and \\
$\phi$    & & angle in radians about the twice rotated $x$ axis (same as \\
          & & this link's $x$ axis.
\end{tabular}
\f]
In dynamic simulation, all Euler angles suffer from a singularity when the
middle angle rotates to the point where the inner and outer axes line up. {\em
...which is why I am considering the conversion to a quaternion representation
in the near future.}

The {\tt getState} and {\tt setState} functions also set and retrieve the
link's angular velocity (in the {\tt qd} parameter).  For this link the
velocity is the angular velocity of the link relative to the inboard coordinate
system and expressed in this link's coordinate system (in radians/sec).  Note
that it is not the time derivatives of the Euler angles.  Note that the size of
both the {\tt q} and {\tt qd} arrays are dimension three, which is the value
returned by the call to {\tt getNumDOFs}.

Specifying joint limits for this type of joint still needs some further
development.  Currently, rudimentary joint limits can be specified with a call
to {\tt setJointLimits}.  The first parameter, {\tt axis\_limits}, contains
three maximum angles between the inboard and outboard coordinate axes in
question (i.e., the first corresponds to the angle between the x-axes, the
second y, and the third z).  A zero value in any position corresponds to no
restriction for the corresponding axis.   If the limit is exceeded, a
restorative torque equal to:\\
\f[
\tau \: = \: K_{limit} \: \Delta q
\f]
where \f$\Delta q\f$ is the amount by which the angle limit is exceeded and is
applied about an axis normal to the axes in question.  In addition the a
dampening term is included that opposes the velocity along the other {\em two}
axes as follows:\\
\f[
\tau \: = \: - B_{limit} \: \dot{q} 
\f]
where \f$K_{limit}\f$ and \f$B_{limit}\f$ correspond to the last two parameters of the {\tt
setJointLimits} member function.

The coulomb friction parameter, \f$\mu\f$, is set with the {\tt setJointFriction}
function and models a dampening force:\\
\f[
{\bf \tau} \: = \: -u \: {\bf \omega}
\f]
which is a 3D torque in a "direction" opposite the rotation (angular velocity).
Finally, an external torque to be applied to link can be specified with the
{\tt setJointInput} function where the parameter is a 3D torque with respect
the link's coordinate system.

The remainder of the functions are described in the {\tt dmLink} reference
pages and are implemented in this class for the specific case of ball joints.
Note again that {\tt getNumDOFs} returns 3.  As such the joint variables {\tt
q}, {\tt qd}, {\tt qdd}, and {\tt joint\_input} are arrays of three elements.

See also: {\tt dmLink}, {\tt dmRigidBody}, {\tt dmLoadFile\_dm}.

 */

class DM_DLL_API dmSphericalLink : public dmRigidBody
{
public:
   enum {NUM_DOFS = 3};

public:
   ///
   dmSphericalLink();
   ///
   virtual ~dmSphericalLink() {};

   ///
   inline void setJointOffset(CartesianVector pos)
                           { for (int i=0; i<3; i++) m_p[i] = pos[i]; }

   ///
   inline int getNumDOFs() const {return NUM_DOFS;}
	inline int getTrueNumDOFs() const {return NUM_DOFS;}
   ///
   void setState(Float q[], Float qd[]);
   ///
   void getState(Float q[], Float qd[]) const;
   ///
   void getPose(RotationMatrix R, CartesianVector p) const;

   ///
   void setJointLimits(CartesianVector axis_limits,
                       Float K_limit, Float B_limit);
   ///
   void setJointFriction(Float u_c) {m_joint_friction = u_c;}

   ///
   inline void setJointInput(Float joint_input[])
   {
      for (int i=0; i<NUM_DOFS; i++)
      {
         m_joint_input[i] = joint_input[i];
      }
   }

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
                   Float **Xin, int cols_Xin) const;
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
	
	
	//! DM v5.0 Function, 
	void RNEAOutwardFKID(dmRNEAStruct &link_val2_curr, 
						 dmRNEAStruct &link_val2_inboard,
						 bool ExtForceFlag = false); // should make the second param const
	
	//! DM v5.0 Function,  for the first link. 
	void RNEAOutwardFKIDFirst(dmRNEAStruct &link_val2_curr, 
							  CartesianVector p_ref_ICS,  
							  RotationMatrix  R_ref_ICS, 
							  const Vector6F& a_ini,
							  const Vector6F& v_ini = Vector6F::Zero(), 
							  bool ExtForceFlag = false
							  );
	
	void jcalc(Matrix6XF &);
	

// Rendering functions:
   ///
   void draw() const;

private:
   // not implemented
   dmSphericalLink(const dmSphericalLink &);
   dmSphericalLink &operator=(dmSphericalLink &);

   void initABVars();

   inline void matrixInverse3PD(CartesianTensor A,
                                CartesianTensor A_inv);
   inline void setJointPos(EulerAngles q);

private:
   EulerAngles m_q;
   Float m_qd[NUM_DOFS];       // angular velocity
   Float m_qdd[NUM_DOFS];      // acceleration.
   Float m_cphi, m_sphi, m_ctheta, m_stheta, m_cpsi, m_spsi;

   Float m_joint_limit[NUM_DOFS]; // angles axes and prev link's.
   Float m_tau_limit[NUM_DOFS];   // torque on link due to limit
   Float m_joint_input[NUM_DOFS];

// vars for the AB algorithm member functions.
   Float m_minv[NUM_DOFS][NUM_DOFS];
   Float m_n_minv[6][NUM_DOFS];
   Float m_tau_star[NUM_DOFS];
};

#endif
