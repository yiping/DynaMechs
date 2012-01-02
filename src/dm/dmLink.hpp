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
 *     File: dmLink.hpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for dmLink.
 *****************************************************************************/

#ifndef _DM_LINK_HPP
#define _DM_LINK_HPP

#include "dm.h"
#include "dmObject.hpp"

//============================================================================
/**

The {\tt dmLink} class is an abstract base class for all derived link classes.
Its main purpose is to support polymorphism across all link classes by
providing the virtual interfaces for all the functions that the {\tt
dmArticulation} class calls in the course of simulating the system.  As such,
nearly all functions specified in the public section are pure virtual functions
that must be defined in the derived link classes (currently, {\tt dmMDHLink},
{\tt dmSphericalLink}, {\tt dmQuaternionLink}, {\tt dmMobileBaseLink}, and {\tt
dmZScrewTxLink}).

The link's state variables (generalized position and velocity variables) can be
set and retrieved using the {\tt setState} and {\tt getState} functions.  The
{\tt q} and {\tt qd} arguments correspond to position and velocity,
respectively.  They are arrays containing $n$ elements where $n$ is the number
of degrees of freedom for this particular type of link (and determined by the
derived classes).  This number is returned when the {\tt getNumDOFs} function
is called.  The joint input vector (containing motor inputs where applicable,
but usually generalized joint forces) can be set by calling {\tt
setJointInput}.  The parameter is also an array of {\tt getNumDOFs} elements
containing the desired values.

A coulomb friction parameter can also be specified and retrieved using the {\tt
setJointFriction} and {\tt getJointFriction} functions.  This scalar parameters
multiplies the joint velocity to produce a force/torque opposing the direction
of motion as follows:
\begin{eqnarray*}
\tau \: = \: \mu_c \: \dot{q}.
\end{eqnarray*}

For dynamics simulation of the link using the AB algorithm, a number of
transformation functions are required of all link types.  Their functions are
listed as follows:
\begin{center}
\begin{tabular}{lcl}
{\tt rtxToInboard}  &  & $^{i-1}f_i = {^iR}^T_{i-1} \: f_i$ \\
{\tt rtxFromInboard}     && $^{i}\omega_{i-1} = {^iR}_{i-1} \: \omega_{i-1}$ \\
{\tt stxToInboard}       && $^{i-1}{\bf f}_i = {^iX}^T_{i-1} \: {\bf f}_i$ \\
{\tt stxFromInboard}     && $^i{\bf v}_{i-1} = {^iX}_{i-1} \: {\bf v}_{i-1}$ \\
{\tt rcongtxToInboardSym}&& $^{i-1}\bar{I}_i =
                      {^iR}^T_{i-1} \: \bar{I}_i \: {^iR}_{i-1}$ \\
{\tt rcongtxToInboardGen}&& $^{i-1}G_i =
                      {^iR}^T_{i-1} \: G_i \: {^iR}_{i-1}$ \\
{\tt scongtxToInboardIrefl} && $^{i-1}{\bf N}_i =
                      {^i{X}}^T_{i-1} \: {\bf N}_i \: {^i{X}}_{i-1}$ \\
\end{tabular}
\end{center}
The first two rotate Cartesian vectors between this link's and the inboard
(parent) link's coordinate systems, while the second two transform (rotate and
translate) spatial vectors.  The last three functions perform congruence
transformations on matrices to the inboard coordinate system.  {\tt
rcongtxToInboardSym} affects a rotational congruence transform on sysmmetric
$3\times 3$ matrices while its counterpart, {\tt rcongtxToInboardGen},
transforms general $3\times 3$ matrices.  The last function, {\tt
scongtxToInboardIrefl}, is a specialized function for the AB algorithm which
efficiently transforms $6\times 6$ reflected articulated-body inertia matrices
taking into account special structure of the matrix based on its joint axes.
This one function is the centerpiece of the efficiency of this implementation
of the AB algorithm.

The next four functions are used to perform the three recursions of the AB
simulation algorithm.  The first forward kinematics recursion is implemented
for each link with the {\tt ABForwardKinematics} function.  This function
should only be called by {\tt dmArticulation::ABForwardKinematics} {\em yeah,
this should be private and {\tt dmArticulation} should be made a friend}.
Again, the {\tt q} and {\tt qd} parameters are the joint position and velocity
vectors computed by the current interation of the numerical integration
algorithm and are used to set the new state of the link.  The {\tt
link\_val\_inboard} is the {\tt ABForKinStruct} (see {\tt dmArticulation}) that
has been filled with the values for the inboard link's state dependent
quantities.  Using these values, the {\tt link\_val\_curr} struct elements are
computed for this link.

The second backward recursion of the AB algorithm is implemented with two
different functions for reasons of efficiency.  If this link has no children,
it is a leaf node in the tree structure, and therefore, the {\tt
dmArticulation::ABBackwardDynamics} function will call the {\tt
ABBackwardDynamicsN} function which will begin the recursion which requires
less computation, by computing {\tt f\_star\_inboard} and {\tt
N\_refl\_inboard}, the AB bias force and inertia that are reflected across this
link's joint and transformed to the inboard link's coordinate system.  The
function {\tt ABBackwardDynamics} performs a full recursive step for a link
with children.  It takes the accumulated results of the child links in {\tt
f\_star\_curr} and {\tt N\_refl\_curr} and computes the same quantities as
before.  The {\tt ABForKinStruct} parameter is the {\tt link\_val\_curr}
computed in the first pass, and is used to call {\tt dmForce::computeForce()}
functions if they have been added to the associated dmRigidBody.

The third forward recursion of the AB algorithm is implemented in the {\tt
ABForwardAcceleration} function.  Its input parameter, {\tt a\_inboard} is the
spatial acceleration of the inboard link.  This function computes and outputs
the spatial acceleration of this link in {\tt a\_curr} and fills in the
derivative of state variables ({\tt qd} and {\tt qdd}).  Note that the velocity
vector, {\tt qd}, is not necessarily the same as that used in {\tt
ABForwardKinematics}.  Rather, it contains the time derivatives of {\tt q}.
See {\tt dmSphericalLink} for an example where they are not the same.

Another set of four functions are required for simulating systems with
kinematic loops.  Three of these function are used in the backward dynamics
recursion of the AB algorithm.  The {\tt XikToInboard} function is used when
eliminating a link from the force-balance equation of its predecessor.  This
function computes the update to the coefficient of the loop-closure constraint
forces of a loop in the predecessor link's force-balance equation (as a result
of the elimination).  The {\tt BtoInboard} function computes the new
coefficient of loop n's constraint forces in the loop-closure constraint
equation for loop k after elimination of the current link in favor of its
predecessor.  The {\tt xformZetak} function computes the new bias term
($\zeta_k$) in loop k's loop-closure constraint equation after elimination of
the link.  Finally, the last function, {\tt ABForwardAccelerations}, is used
in the forward accelerations recursion of the AB algorithm.  It is an
overloaded version of the same function used for tree topologies, requiring
additional parameters containing topology information and computed
loop-closure forces.

Two functions, {\tt pushForceStates} and {\tt popForceStates}, save and
restore the state of all {\tt dmForce} objects attached to the link.  These
functions are provided for adaptive-stepsize integrators which may need to
back out of an integration step.

Another two functions are provided for retrieving the energy of a link.  The
{\tt getPotentialEnergy} function returns the gravitational potential energy of
the link based on the supplied gravity vector and the {\tt ABForKinStruct}
parameter for the link.  The potential energy zero is defined by the plane
passing through the origin of the inertial coordinate system with the gravity
vector as its normal.  The {\tt getKineticEnergy} function returns the kinetic
energy of the link (translational + rotational) based on the supplied {\tt
ABForKinStruct} parameter.

The only function that is implemented in this class is the {\tt
forwardKinematics} function.  It is called by {\tt
dmArticulation::forwardKinematics} function and computes the current link's
position ({\tt link\_val\_curr->p\_ICS}) and orientation ({\tt
link\_val\_curr->R\_ICS}) with respect to the inertial coordinate frame given
the inboard link's position and orientation ({\tt link\_val\_inboard->p\_ICS}
and {\tt link\_val\_inboard->R\_ICS}).

See also: {\tt dmArticulation}, {\tt dmMDHLink}, {\tt dmSphericalLink},
          {\tt dmQuaternionLink}, {\tt dmZScrewTxLink}, {\tt dmMobileBaseLink}.

 */

class DM_DLL_API dmLink : public dmObject
{
public:
   ///
   dmLink();
   ///
   virtual ~dmLink();

   ///
   virtual int getNumDOFs() const = 0;
   ///
   virtual void setState(Float q[], Float qd[]) = 0;
   ///
   virtual void getState(Float q[], Float qd[]) const = 0;
   ///
   virtual void getPose(RotationMatrix R, CartesianVector p) const = 0;

   ///
   virtual void setJointInput(Float joint_input[]) = 0;

   ///
   Float getJointFriction() const {return m_joint_friction;}
   ///
   void setJointFriction(Float u_c) {m_joint_friction = u_c;}

// Link-to-link transformation functions:
   ///
   virtual void rtxToInboard(const CartesianVector curr,
                             CartesianVector prev) const;
   ///
   virtual void rtxFromInboard(const CartesianVector prev,
                               CartesianVector curr) const;
   ///
   virtual void stxToInboard(const SpatialVector curr,
                             SpatialVector prev) const;
   ///
   virtual void stxFromInboard(const SpatialVector prev,
                               SpatialVector curr) const;
   ///
   virtual void rcongtxToInboardSym(const CartesianTensor Curr,
                                    CartesianTensor Prev) const;
   ///
   virtual void rcongtxToInboardGen(const CartesianTensor Curr,
                                    CartesianTensor Prev) const;
   ///
   virtual void scongtxToInboardIrefl(const SpatialTensor N_curr,
                                      SpatialTensor N_prev) const;
   ///
   virtual void XikToInboard(Float **Xik_curr,
                             Float **Xik_prev,
                             int columns_Xik) const = 0;
   ///
   virtual void BToInboard(Float **Bkn,
                           Float **Xik, int cols_Xik,
                           Float **Xin, int cols_Xin) const = 0;
   ///
   virtual void xformZetak(Float *zetak,
                           Float **Xik, int cols_Xik) const = 0;


// Articulated-Body (AB) algorithm functions:
   ///
   virtual void ABForwardKinematics(Float q[],
                                    Float qd[],
                                    const dmABForKinStruct &link_val_inboard,
                                    dmABForKinStruct &link_val_curr) = 0;

   ///
   virtual void ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
                                   SpatialVector f_star_curr,
                                   SpatialTensor N_refl_curr,
                                   SpatialVector f_star_inboard,
                                   SpatialTensor N_refl_inboard) = 0;
   ///
   virtual void ABBackwardDynamicsN(const dmABForKinStruct &link_val_curr,
                                    SpatialVector f_star_inboard,
                                    SpatialTensor N_refl_inboard) = 0;

   ///
   virtual void ABForwardAccelerations(SpatialVector a_inboard,
                                       SpatialVector a_curr,
                                       Float qd[],
                                       Float qdd[]) = 0;
   ///
   virtual void ABForwardAccelerations(SpatialVector a_inboard,
                                       unsigned int *LB,
                                       unsigned int num_elements_LB,
                                       Float ***Xik,
                                       Float **constraint_forces,
                                       unsigned int *num_constraints,
                                       SpatialVector a_curr,
                                       Float qd[],
                                       Float qdd[]) = 0;

// Generic forward kinematics algorithm functions:
   ///
   void forwardKinematics(dmABForKinStruct &link_val_inboard,
                          dmABForKinStruct &link_val_curr) const;

   ///
   virtual void pushForceStates() {};
   ///
   virtual void popForceStates() {};
   ///
   virtual Float getPotentialEnergy(const dmABForKinStruct &link_val,
                                    CartesianVector a_gravity) const = 0;
   ///
   virtual Float getKineticEnergy(const dmABForKinStruct &link_val) const = 0;

// rendering functions:
   ///
   virtual void draw() const = 0;

private:   // not implemented
   dmLink(const dmLink &);
   dmLink &operator=(const dmLink &);

protected:
   CartesianVector m_p;   // position of this link's CS wrt inboard link.
   RotationMatrix  m_R;   // Rotation matrix ^i{\bf R}_{i-1}

   bool  m_joint_limit_flag;      // define dynamic behavior when joint
   Float m_joint_limit_spring;    // limits are reached.
   Float m_joint_limit_damper;
   Float m_joint_friction;        // Coulomb(?) friction coefficient.

   SpatialVector m_zeta;          // Additional variables for the AB algorithm
   SpatialTensor m_N_refl;        // member functions that are not
   SpatialVector m_gamma;         // dependent on the joint type.
};

#endif
