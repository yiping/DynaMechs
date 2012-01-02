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
 *     File: dmSecondaryJoint.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Abstract base class declaration for secondary joints.
 *****************************************************************************/

#ifndef _DM_SECONDARY_JOINT_HPP
#define _DM_SECONDARY_JOINT_HPP

#include "dm.h"
#include "dmObject.hpp"
#include "dmArticulation.hpp"
#include "dmLink.hpp"

//============================================================================
/** 
The {\tt dmSecondaryJoint} class is an abstract base class for all
derived (loop-closing) secondary joint classes.  Its main purpose is
to support polymorphism across all secondary joint classes by
providing the virtual interfaces for all the functions that the {\tt
dmClosedArticulation} class calls in the course of simulating the
system.  As such, many functions specified in the public section
are pure virtual functions that must be defined in the derived
secondary joint classes (currently, {\tt dmSecondaryPrismaticJoint},
{\tt dmSecondaryRevoluteJoint}, and {\tt dmSecondarySphericalJoint}).

The following describes functions used to initialize all secondary
joint objects.  The {\tt setArticulation} function establishes which
articulation the links connected by the secondary joint belong to.
After setting the articulation, the {\tt setLinkA} and {\tt setLinkB}
functions may be used to set the links connected by the joint, while
the {\tt getLinkAIndex} and {\tt getLinkBIndex} functions may be used
at a later time to retrieve the indices of these links in the
articulation.  The {\tt setKinematics} function is used to locate the
outboard coordinate systems of the connected links (used by the
secondary joint) relative to their respective link coordinate systems.
This function requires as arguments the locations of the outboard
coordinate systems, with respect to link coordinates, as well as the
rotation matrices from the outboard coordinate systems to link
coordinates.

A coulomb friction parameter can also be specified and retrieved using
the {\tt setJointFriction} and {\tt getJointFriction} functions.  This
scalar parameters multiplies the joint velocity to produce a
force/torque opposing the direction of motion as follows:
\begin{eqnarray*}
\tau \: = \: \mu_c \: \dot{q}.
\end{eqnarray*}

When a secondary joint is added to a {\tt dmClosedArticulation} as a
`hard' joint, three constraint stabilization options exist.  These
include Baumgarte stabilization, spring and damper stabilization, and
no stabilization.  The {\tt StabilizationEnum} enumeration type
defined in this class has three values representing these options:
{\tt BAUMGARTE}, {\tt SPRING_DAMPER}, and {\tt NONE}.  The {\tt
setStabilizationType} function takes a {\tt StabilizationEnum} as an
argument to choose the stabilization method, whereas the {\tt
getStabilizationType} function returns the current stabilization
method.

The state of the secondary joints is determined by the state of the
joints in the system spanning tree.  Because the state of the joints
in the spanning tree are not at all times consistent with the
constraints imposed by the secondary joints, a secondary joint has
joint variables in both the free and the constrained modes of the
joint.  The virtual {\tt computeState} function, called by {\tt
dmClosedArticulation::ABForwardKinematics()}, is implemented in the
derived classes to compute the free and constrained states for a
specific type of joint. Note that the {\tt computeState} function
defined in the base class is called by the derived classes in order to
compute state information common to all secondary joints.

The {\tt getFreeState} function retrieves the free variables of the
joint, where the {\tt q} and {\tt qd} arguments correspond to position
and velocity, respectively.  They are arrays containing $n$ elements
where $n$ is the number of degrees of freedom for this particular type
of joint (spherical joints are an exception).  This number is
returned when the {\tt getNumDOFs} function is called.  The joint
input vector for the free modes can be set by calling {\tt
setJointInput}.  The parameter is also an array of {\tt getNumDOFs}
elements containing the desired values.  The joint variables in the
constrained modes of the joint are retrieved with the function {\tt
getConstrainedState}, where the {\tt q} and {\tt qd} arguments are now
of length $6-n$.

The pure virtual {\tt computeAppliedForce} function is called by {\tt
dmClosedArticulation::ABForwardKinematics()} to compute the resultant
force and moment applied by link A to link B at the outboard
coordinate system of link B due to the joint input and friction.  For
`soft' secondary joints, the constraint forces at the joint are known
as a function of the constrained joint variables and their
derivatives, and {\tt dmClosedArticulation::ABForwardKinematics()}
calls the pure virtual {\tt applyPenaltyForce} function to compute
these constraint forces and add them to the applied forces at the
joint.  The {\tt getAppliedAForce} and {\tt getAppliedBForce}
functions, used by the {\tt dmSecondaryForce} objects, return the
above joint forces reflected onto the A and B links and expressed in
the respective link's coordinates.

The following functions are pure virtual and are implemented for every
derived class.  The {\tt initXik} function initializes the the X_ik's
for the joint's A link, B link, and the root link of the loop closed
by the joint.  Note that Xik is the coefficient of loop k's constraint
forces in link i's force balance equation, and its transpose also
appears in loop k's constraint equations.  The {\tt computeEtas}
function computes the eta_k1 and eta_k2 acceleration bias terms for
the joint coordinate system (Marhefka dissertation Eqs. 4.67 and
4.69), while the {\tt getZeta} function computes and returns the $6-n$
Zeta_k acceleration bias term appearing in the loop-closure constraint
equation (Marhefka dissertation Eq. 4.75).  The above three functions
are called by {\tt dmClosedArticulation::ABForwardKinematics()}.
Finally, the {\tt computeStabilizationForce} function is used for
spring and damper constraint stabilization (not to be confused with
'soft' constraints).  It returns the $6-n$ vector of additional
constrained mode joint inputs to be added to those computed by the
unstabilized constraint-based algorithm, and is called during the
backward dynamics recursion.
 */

class DM_DLL_API dmSecondaryJoint : public dmObject
{
public:
   enum StabilizationEnum
   {
      NONE,
      BAUMGARTE,
      SPRING_DAMPER
   };  // Hard constraint stabilization methods.

public:
   ///
   dmSecondaryJoint();
   ///
   ~dmSecondaryJoint() {};
   ///
   void setArticulation(dmArticulation *art);
   ///
   void setLinkA(dmLink *link);
   ///
   void setLinkB(dmLink *link);
   ///
   int getLinkAIndex() {return m_link_A_index;}
   ///
   int getLinkBIndex() {return m_link_B_index;}
   ///
   void setKinematics(const CartesianVector pos_a,
                      const CartesianVector pos_b,
                      const RotationMatrix rot_a,
                      const RotationMatrix rot_b);
   ///
   void setStabilizationType(StabilizationEnum stab) {m_stabilization = stab;}
   ///
   inline StabilizationEnum getStabilizationType() { return m_stabilization; }

   ///
   virtual int getNumDOFs() const = 0;
   ///
   virtual void initXik(Float **Xik, int link_index,
                        int root_index) const = 0;
   ///
   virtual void getZeta(Float *zeta) const = 0;

   ///
   virtual void getFreeState(Float q[], Float qd[]) const = 0;
   ///
   virtual void getConstrainedState(Float q[], Float qd[]) const = 0;
   ///
   virtual void setJointInput(Float joint_input[]) = 0;

   ///
   Float getJointFriction() const {return m_joint_friction;}
   ///
   void setJointFriction(Float u_c) {m_joint_friction = u_c;}

   ///
   virtual void computeStabilizationForce(Float force[]) = 0;

   ///
   virtual void computeState();
   ///
   virtual void computeEtas() = 0;

   ///
   virtual void computeAppliedForce() = 0;
   ///
   virtual void applyPenaltyForce() = 0;
   ///
   void getAppliedAForce(SpatialVector force);
   ///
   void getAppliedBForce(SpatialVector force);

private:   // not implemented
   dmSecondaryJoint(const dmSecondaryJoint &);
   dmSecondaryJoint &operator=(const dmSecondaryJoint &);

protected:
   int               m_link_A_index;
   int               m_link_B_index;
   dmArticulation   *m_articulation;
   CartesianVector   m_a_p_oa;         // fixed
   RotationMatrix    m_a_R_oa;         // fixed
   CartesianVector   m_b_p_k;          // fixed
   RotationMatrix    m_b_R_k;          // fixed

   StabilizationEnum m_stabilization;  // only used for hard constraints

   RotationMatrix    m_a_R_k;          // function of state
   CartesianVector   m_a_p_k;          // function of state

   RotationMatrix    m_oa_R_k;         // function of state
   CartesianVector   m_k_w_rel;        // function of state
   CartesianVector   m_d;              // function of state
   CartesianVector   m_d_dot;          // function of state

   CartesianVector   m_oa_w_oa;
   CartesianVector   m_k_w_oa;

   SpatialVector     m_Eta_k1;         // used in sub classes
   SpatialVector     m_Eta_k2;         // used in sub classes

   SpatialVector     m_k_f_k;

   Float             m_joint_friction; // Coulomb(?) friction coefficient.
};

#endif
