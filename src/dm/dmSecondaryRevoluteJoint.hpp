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
 *     File: dmSecondaryRevoluteJoint.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Class definitions for revolute secondary joints.
 *****************************************************************************/

#ifndef _DM_SECONDARY_REVOLUTE_JOINT_HPP
#define _DM_SECONDARY_REVOLUTE_JOINT_HPP

#include "dm.h"
#include "dmObject.hpp"
#include "dmSecondaryJoint.hpp"

//======================================================================

/**

The {\tt dmSecondaryRevoluteJoint} class can model loop-closing joints
with one rotational degree of freedom.  The class is derived from the
{\tt dmSecondaryJoint} class.  The state of the joint is computed
based on a six-degree of freedom joint from the outboard coordinate
system of link A to that of link B.  The joint consists of
translations along the $\hat{x}$, $\hat{y}$, and $\hat{z}$ axes of
link A's outboard coordinate system ($d_x$, $d_y$, and $d_z$),
followed by XYZ Euler angles ($\alpha$, $\beta$, and $\gamma$).  The
free joint variable is the Z Euler angle, $\gamma$, while the vector
of constrained joint variables is given by
$[\alpha$~$\beta$~d_x~d_y~d_z]^T$.  Note that the simulation algorithm
drives the constrained joint variables to zero.

The function, {\tt setConstraintParams}, sets parameters for
implementing the joint constraints.  The meaning and use of the
parameters depends on the type of secondary joint.  For compliant
(soft) joints, the terms are the linear and angular springs and
dampers which implement the joint constraints.  For hard joints with
spring/damper stabiliation, the terms are the springs and dampers
which implement the constraint stabilization.  For hard joints with
Baumgarte stabilization, the terms are interpreted as the Baumgarte
coefficients.  The remainder of the functions are described in the
{\tt dmSecondaryJoint} reference pages and are implemented in this
class for the specific case of revolute secondary joints.

A configuration file reader, {\tt dmLoadFile\_dm} is being supplied
with the dmutils library that can be used to instantiate and
initialize these secondary joint objects.
 */


class DM_DLL_API dmSecondaryRevoluteJoint :  public dmSecondaryJoint
{
public:
   enum {NUM_DOFS = 1};

public:
   ///
   dmSecondaryRevoluteJoint();
   ///
   ~dmSecondaryRevoluteJoint() {};

   ///
   int getNumDOFs() const {return NUM_DOFS;}
   ///
   void initXik(Float **Xik, int link_index, int root_index) const;
   ///
   void getZeta(Float *zeta) const;

   ///
   inline void setJointInput(Float joint_input[])
      {
         m_joint_input = joint_input[0];
      }

   ///
   void getFreeState(Float q[], Float qd[]) const;
   ///
   void getConstrainedState(Float q[], Float qd[]) const;
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
   void setConstraintParams(Float linear_spring, Float linear_damper,
                            Float angular_spring, Float angular_damper);

private:   // not implemented
   dmSecondaryRevoluteJoint(const dmSecondaryRevoluteJoint &);
   dmSecondaryRevoluteJoint &operator=(const dmSecondaryRevoluteJoint &);

protected:
   Float m_pos_spring;
   Float m_pos_damper;
   Float m_euler_spring;
   Float m_euler_damper;

   CartesianVector m_euler;            // orientation sv's
   CartesianVector m_euler_dot;        // derivatives of orientation sv's
   Float m_c2, m_s2, m_c1, m_s1, m_t1; // trig. fctns. of the euler angles

   Float m_joint_input;
};

#endif
