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
 *     File: dmSecondarySphericalJoint.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Class definitions for spherical secondary joints.
 * ***************************************************************************/

#ifndef _DM_SECONDARY_SPHERICAL_JOINT_HPP
#define _DM_SECONDARY_SPHERICAL_JOINT_HPP

#include "dm.h"
#include "dmObject.hpp"
#include "dmSecondaryJoint.hpp"

//======================================================================

/**

The {\tt dmSecondarySphericalJoint} class can model loop-closing
joints with three rotational degrees of freedom.  The class is derived
from the {\tt dmSecondaryJoint} class.  The joint consists of
translations ($d_x$, $d_y$, and $d_z$) along the $\hat{x}$, $\hat{y}$,
and $\hat{z}$ axes of link A's outboard coordinate system to link B's
outboard coordinate system, followed by a unconstrained rotation.  The
vector of constrained joint variables is given by
$[d_x~d_y~d_z]^T$. Note that the simulation algorithm drives the
constrained joint variables to zero.  No Euler angles are computed, so
no singularity problems exist.  A call to {\tt getFreeState} returns
the rotation matrix in a vector format for the state and the relative
angular velocity across the joint (w.r.t. link B outboard
coordinates).  Likewise, the joint input is the torque applied through
the joint to link B at it's outboard coordinate system and expressed
w.r.t. those coordinates.

The function, {\tt setConstraintParams}, sets parameters for
implementing the joint constraints.  The meaning and use of the
parameters depends on the type of secondary joint.  For compliant
(soft) joints, the terms are the linear springa and dampers which
implement the joint constraints.  For hard joints with spring/damper
stabiliation, the terms are the springs and dampers which implement
the constraint stabilization.  For hard joints with Baumgarte
stabilization, the terms are interpreted as the Baumgarte
coefficients.  The remainder of the functions are described in the
{\tt dmSecondaryJoint} reference pages and are implemented in this
class for the specific case of spherical secondary joints.

A configuration file reader, {\tt dmLoadFile\_dm} is being supplied
with the dmutils library that can be used to instantiate and
initialize these secondary joint objects.
 */

class DM_DLL_API dmSecondarySphericalJoint :  public dmSecondaryJoint
{
public:
   enum {NUM_DOFS = 3};

public:
   ///
   dmSecondarySphericalJoint();
   ///
   ~dmSecondarySphericalJoint() {};

   ///
   int getNumDOFs() const {return NUM_DOFS;}
   ///
   void initXik(Float **Xik, int link_index, int root_index) const;
   ///
   void getZeta(Float *zeta) const;

   ///
   void getFreeState(Float q[], Float qd[]) const;
   ///
   void getConstrainedState(Float q[], Float qd[]) const;
   ///
   inline void setJointInput(Float joint_input[])
      {
         for (int i=0; i<NUM_DOFS; i++)
         {
            m_joint_input[i] = joint_input[i];
         }
      }

   ///
   void computeState();
   ///
   void computeEtas();
   ///
   void computeAppliedForce();
   ///
   void applyPenaltyForce();
   ///
   void computeStabilizationForce(Float force[]);

   ///
   void setConstraintParams(Float linear_spring, Float linear_damper);

private:   // not implemented
   dmSecondarySphericalJoint(const dmSecondarySphericalJoint &);
   dmSecondarySphericalJoint &operator=(const dmSecondarySphericalJoint &);

protected:
   Float m_pos_spring;
   Float m_pos_damper;

   Float m_joint_input[NUM_DOFS];
};

#endif
