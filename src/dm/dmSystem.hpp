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
 *     File: dmSystem.hpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for dmSystem - top level DynaMechs structure
 *****************************************************************************/

#ifndef _DM_SYSTEM_HPP
#define _DM_SYSTEM_HPP

#include "dm.h"
#include "dmObject.hpp"

//======================================================================

/**

The {\tt dmSystem} is the abstract base class for all types of objects that
need to be simulated by a {\tt dmIntegrator} class.  As such this class defines
the interface needed by the dmIntegrator classes, and is the base class
for robotic systems (dmArticulation and dmClosedArticulation), dynamic
environments (dmTreadmill through dmEnvironment), and (in the future) control
systems.  By adding all the dynamic systems to a single integrator, it will be
ensured that all the systems will be numerically integrated in synchrony with
one another.  This is especially important when multi-step and/or variable
stepsize numerical integrators are used, where the system dynamics are
calculated at more then one point across the interval, where some computations
may be rejected due to accuracy constraints, and the interval may be subdivided
into smaller steps.

The {\tt setState} and {\tt getState} functions are used to set and query joint
states in the entire tree.  {\tt setState} sets the state of the DOFs in the
articulation.  The two arguments are both packed arrays (like {\tt
joint\_input}) containing the joint positions and velocities of the links.  The
reverse operation, {\tt getState}, fills packed arrays with the joint positions
velocities.  A convenience function, {\tt getNumDOFs}, returns the total number
of degrees of freedom in the articulation and can be used to determine the
appropriate size of the above arrays.

The {\tt initSimVars} function initializes the state and derivative of state
vectors with the current state of the system.  The parameters, {\tt qy} and
{\tt qdy}, are the arrays with 2*{\tt getNumDOFs()} elements.  All necessary
calls to {\tt initSimVars} are done automatically by the {\tt dmIntegrator}
class when a {\tt dmSystem} object is added to or removed from it.  Note that
{\tt initSimVars} is different from the {\tt getState} function that returns
just the position and velocity state vectors in two separate vectors ({\tt q}
and {\tt qd}) of {\tt getNumDOFs()} each.  {\bf IMPORTANT}: The {\tt
dmIntegrator} class cannot detect subsequent changes that would affect the
number of DOFs (e.g., addition/removal of links); therefore, the entire system
must be built before it is assigned to the {\tt dmIntegrator} object and no
changes can be made after this point.

Specification of an inertial coordinate system other than the origin can be
specified by calling {\tt setRefSystem} and passing the position and
orientation (relative to the origin) of the new coordinate system.
This ability is not necessary or even computationally efficient, but it makes
it much more flexible in specifying systems.  The functions {\tt getRefSystem}
and {\tt getPose} return this position and orientation -- the latter in two
different formats: quaternion or rotation matrix.  The functions, {\tt
rtxFromICS} and {\tt rtxToICS}, are used to rotate free vectors from one system
to the other.

The {\tt dynamics} function is the entry point for computation of the dynamics
of the system.  It takes, as input, the state vector {\tt qy} and computes the
derivative of state vector, {\tt qdy}.  This is the derivative function that
the {\tt dmIntegrator} class uses to perform the numerical integration.

Two functions are provided for use by adaptive-stepsize integrators which may
need to back out of a rejected integration step.  This is required because some
{\tt dmForce} objects attached to links may retain state information.  The {\tt
pushForceStates} function is used to save the state information of all force
objects attached to links in the system, while the {\tt popForceStates}
function is used if necessary to restore the state to the stored values.

Another two functions are used for retrieving the energy of the system.  The
{\tt getPotentialEnergy} function returns the potential energy.  Note that the
potential energy zero is defined by the plane which passes through the origin
and has the gravity vector as its normal.  The {\tt getKineticEnergy} function
returns the sum of translational and rotational kinetic energy of the system.

See also: {\tt dmArticulation, dmEnvironment, dmIntegrator}.  */

//======================================================================

class DM_DLL_API dmSystem : public dmObject
{
public:
   ///
   dmSystem();
   ///
   virtual ~dmSystem();

   ///
   virtual unsigned int getNumDOFs() const = 0;
   ///
   virtual void setState(Float q[], Float qd[]) = 0;
   /// qy and qdy are arrays with getNumDOFs() elements
   virtual void getState(Float q[], Float qd[]) const = 0;

   /// qy and qdy are arrays with 2*getNumDOFs() elements
   void initSimVars(Float *qy, Float *qdy);

   // transformation functions
   ///
   void setRefSystem(dmQuaternion quat, CartesianVector pos);
   ///
   void getRefSystem(dmQuaternion quat, CartesianVector pos) const;
   ///
   void getPose(RotationMatrix R, CartesianVector pos) const;

   ///
   inline void rtxFromICS(const CartesianVector p_ICS,
                          CartesianVector p_ref) const;
   ///
   inline void   rtxToICS(const CartesianVector p_ref,
                          CartesianVector p_ICS) const;

   ///
   virtual void pushForceStates() = 0;
   ///
   virtual void popForceStates() = 0;
   ///
   virtual Float getPotentialEnergy() const = 0;
   ///
   virtual Float getKineticEnergy() const = 0;

   // dynamics algorithm
   ///
   virtual void dynamics(Float *qy, Float *qdy) = 0;

   virtual void draw() const = 0;

protected:
   // not implemented
   dmSystem(const dmSystem &);
   dmSystem &operator=(const dmSystem &);

   // forward kinematics helper routine
   void forwardKinematics(dmABForKinStruct &fk) const;

protected:
   // description of transform to reference system
   dmQuaternion      m_quat_ICS;
   RotationMatrix  m_R_ICS;     // pose of links wrt ICS - ^{ICS}R_{i}
   CartesianVector m_p_ICS;     // pos. of links wrt ICS - ^{ICS}p_{i}
};

//----------------------------------------------------------------------------
//    Summary: 3D rotational transformation from ICS to ref mem CS
// Parameters: p_ICS - 3-vector expressed in the ICS
//    Returns: p_ref - 3-vector expressed in ref mem's CS
//----------------------------------------------------------------------------
inline void dmSystem::rtxFromICS(const CartesianVector p_ICS,
                                 CartesianVector p_ref) const
{
   p_ref[0] = m_R_ICS[0][0]*p_ICS[0] + m_R_ICS[0][1]*p_ICS[1] +
      m_R_ICS[0][2]*p_ICS[2];
   p_ref[1] = m_R_ICS[1][0]*p_ICS[0] + m_R_ICS[1][1]*p_ICS[1] +
      m_R_ICS[1][2]*p_ICS[2];
   p_ref[2] = m_R_ICS[2][0]*p_ICS[0] + m_R_ICS[2][1]*p_ICS[1] +
      m_R_ICS[2][2]*p_ICS[2];
}

//----------------------------------------------------------------------------
//    Summary: 3D rotational transformation from ref mem CS to ICS
// Parameters: p_ref - 3-vector expressed in ref mem's CS
//    Returns: p_ICS - 3-vector expressed in the ICS
//----------------------------------------------------------------------------
inline void dmSystem::rtxToICS(const CartesianVector p_ref,
                               CartesianVector p_ICS) const
{
   p_ICS[0] = m_R_ICS[0][0]*p_ref[0] + m_R_ICS[1][0]*p_ref[1] +
      m_R_ICS[2][0]*p_ref[2];
   p_ICS[1] = m_R_ICS[0][1]*p_ref[0] + m_R_ICS[1][1]*p_ref[1] +
      m_R_ICS[2][1]*p_ref[2];
   p_ICS[2] = m_R_ICS[0][2]*p_ref[0] + m_R_ICS[1][2]*p_ref[1] +
      m_R_ICS[2][2]*p_ref[2];
}

#endif
