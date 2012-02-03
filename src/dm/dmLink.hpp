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

The dmLink class is an abstract base class for all derived link classes.
Its main purpose is to support polymorphism across all link classes by
providing the virtual interfaces for all the functions that the dmArticulation
class calls in the course of simulating the system.  As such,
nearly all functions specified in the public section are pure virtual functions
that must be defined in the derived link classes (currently, dmMDHLink,
dmSphericalLink, dmQuaternionLink, dmMobileBaseLink, and dmZScrewTxLink).

The link's state variables (generalized position and velocity variables) can be
set and retrieved using the \b setState and \b getState functions.  The
\b q and \b qd arguments correspond to position and velocity,
respectively.  They are arrays containing $n$ elements where $n$ is the number
of degrees of freedom for this particular type of link (and determined by the
derived classes).  This number is returned when the \b getNumDOFs function
is called.  The joint input vector (containing motor inputs where applicable,
but usually generalized joint forces) can be set by calling \b setJointInput.  
The parameter is also an array of \b getNumDOFs elements
containing the desired values.

A coulomb friction parameter can also be specified and retrieved using the \b 
setJointFriction and \b getJointFriction functions.  This scalar parameters
multiplies the joint velocity to produce a force/torque opposing the direction
of motion as follows:
\f[
\tau \: = \: \mu_c \: \dot{q}.
\f]

For dynamics simulation of the link using the AB algorithm, a number of
transformation functions are required of all link types.  Their functions are
listed as follows:
\f[
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
\f]

The first two rotate Cartesian vectors between this link's and the inboard
(parent) link's coordinate systems, while the second two transform (rotate and
translate) spatial vectors.  The last three functions perform congruence
transformations on matrices to the inboard coordinate system.  \b
rcongtxToInboardSym affects a rotational congruence transform on sysmmetric
$3\times 3$ matrices while its counterpart, \b rcongtxToInboardGen,
transforms general \f$3\times 3\f$ matrices.  The last function, \b
scongtxToInboardIrefl, is a specialized function for the AB algorithm which
efficiently transforms \f$6\times 6\f$ reflected articulated-body inertia matrices
taking into account special structure of the matrix based on its joint axes.
This one function is the centerpiece of the efficiency of this implementation
of the AB algorithm.

The next four functions are used to perform the three recursions of the AB
simulation algorithm.  The first forward kinematics recursion is implemented
for each link with the \b ABForwardKinematics function.  This function
should only be called by dmArticulation::ABForwardKinematics <em> yeah,
this should be private and dmArticulation should be made a friend</em>.
Again, the \b q and \b qd parameters are the joint position and velocity
vectors computed by the current interation of the numerical integration
algorithm and are used to set the new state of the link.  The \b
link_val_inboard is the \b ABForKinStruct (see dmArticulation) that
has been filled with the values for the inboard link's state dependent
quantities.  Using these values, the \b link_val_curr struct elements are
computed for this link.

The second backward recursion of the AB algorithm is implemented with two
different functions for reasons of efficiency.  If this link has no children,
it is a leaf node in the tree structure, and therefore, the 
dmArticulation::ABBackwardDynamics function will call the \b
ABBackwardDynamicsN function which will begin the recursion which requires
less computation, by computing \b f_star_inboard and \b
N_refl_inboard, the AB bias force and inertia that are reflected across this
link's joint and transformed to the inboard link's coordinate system.  The
function \b ABBackwardDynamics performs a full recursive step for a link
with children.  It takes the accumulated results of the child links in \b
f_star_curr and \b N_refl_curr and computes the same quantities as
before.  The \b ABForKinStruct parameter is the \b link_val_curr
computed in the first pass, and is used to call dmForce::computeForce()
functions if they have been added to the associated dmRigidBody.

The third forward recursion of the AB algorithm is implemented in the \b
ABForwardAcceleration function.  Its input parameter, \b a_inboard is the
spatial acceleration of the inboard link.  This function computes and outputs
the spatial acceleration of this link in \b a_curr and fills in the
derivative of state variables (\b qd and \b qdd).  Note that the velocity
vector, \b qd, is not necessarily the same as that used in \b
ABForwardKinematics.  Rather, it contains the time derivatives of \b q.
See dmSphericalLink for an example where they are not the same.

Another set of four functions are required for simulating systems with
kinematic loops.  Three of these function are used in the backward dynamics
recursion of the AB algorithm.  The \b XikToInboard function is used when
eliminating a link from the force-balance equation of its predecessor.  This
function computes the update to the coefficient of the loop-closure constraint
forces of a loop in the predecessor link's force-balance equation (as a result
of the elimination).  The \b BtoInboard function computes the new
coefficient of loop n's constraint forces in the loop-closure constraint
equation for loop k after elimination of the current link in favor of its
predecessor.  The \b xformZetak function computes the new bias term
($\zeta_k$) in loop k's loop-closure constraint equation after elimination of
the link.  Finally, the last function, \b ABForwardAccelerations, is used
in the forward accelerations recursion of the AB algorithm.  It is an
overloaded version of the same function used for tree topologies, requiring
additional parameters containing topology information and computed
loop-closure forces.

Two functions, \b pushForceStates and \b popForceStates, save and
restore the state of all dmForce objects attached to the link.  These
functions are provided for adaptive-stepsize integrators which may need to
back out of an integration step.

Another two functions are provided for retrieving the energy of a link.  The
\b getPotentialEnergy function returns the gravitational potential energy of
the link based on the supplied gravity vector and the \b ABForKinStruct
parameter for the link.  The potential energy zero is defined by the plane
passing through the origin of the inertial coordinate system with the gravity
vector as its normal.  The \b getKineticEnergy function returns the kinetic
energy of the link (translational + rotational) based on the supplied \b
ABForKinStruct parameter.

The only function that is implemented in this class is the \b
forwardKinematics function.  It is called by 
dmArticulation::forwardKinematics function and computes the current link's
position (\b link_val_curr->p_ICS) and orientation (\b
link_val_curr->R_ICS) with respect to the inertial coordinate frame given
the inboard link's position and orientation (\b link_val_inboard->p_ICS
and \b link_val_inboard->R_ICS).

See also: dmArticulation, dmMDHLink, dmSphericalLink,
          dmQuaternionLink, dmZScrewTxLink, dmMobileBaseLink.

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
	
	void stxToInboardMat(const Matrix6XF &curr,
							  Matrix6XF &prev) const;
	
	
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
	
	// CRB Functions
	void scongxToInboardIcomp(const CrbInertia & IC_curr, CrbInertia & IC_prev) const;
	
	virtual void initializeCrbInertia(CrbInertia & IC_curr) const;
	
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

   ///
   //! Return the 6x6 spatial transformation matrix (\f$ {}^i X_{i-1}\f$).
   /*!
      v5.0 function
      \b Should make it a pure virtual function, but that involves adding the implementation for 
       all link types. Let's worry about it later. 
   */
   virtual Matrix6F get_X_FromParent_Motion();
   
   //! Return the subspace vector of the joint
   /*!
      v5.0 function
      \b Should make it a pure virtual function, too, later.
   */
   virtual Matrix6XF jcalc();

//Inverse Dynamics Algorithm (Recursive NE) Functions 
   //! DM v5.0 Function, recursive NE algorithm, outward forward kinematics and inverse dynamics, later should be made a pure virtual. 
   virtual void RNEAOutwardFKID(dmRNEAStruct &link_val2_curr, 
                                      dmRNEAStruct &link_val2_inboard);

   //! DM v5.0 Function, recursive NE algorithm, outward forward kinematics and dynamics for the first link. Later should be made a pure virtual.
   virtual void RNEAOutwardFKIDFirst(dmRNEAStruct &link_val2_curr, 
					  CartesianVector  p_ref_ICS,  
                                          RotationMatrix  R_ref_ICS, 
                                          Vector6F a_ini, Vector6F v_ini = Vector6F::Zero());

   //! DM v5.0 Function, recursive NE algorithm, inward inverse dynamics, later should be made a pure virtual. 
   virtual void RNEAInwardID( dmRNEAStruct &link_val2_curr,
                            dmRNEAStruct &link_val2_inboard);



   //! DM v5.0 function, 


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
