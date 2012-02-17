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
 *     File: dmMDHLink.hpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for links with 1dof joints (Modified Denavit
 *         : Hartenberg notation.
 *****************************************************************************/

#ifndef _DM_MDH_LINK_HPP
#define _DM_MDH_LINK_HPP

#include "dm.h"
#include "dmRigidBody.hpp"
#include "dmLink.hpp"
#include "dmActuator.hpp"

//======================================================================

/**
The dmMDHLink class can model the class of links with one degree
of freedom (revolute and prismatic) joints.  The class is derived from the
dmRigidBody (for the dynamic parameters) class and, through that, from
the dmLink base class (required by dmArticulation for Articulated
Body simulation functions).  This, in turn, is an abstract base class for the
two kinds of single DOF link classes: dmRevoluteLink and 
dmPrismaticLink.

The kinematics (origin and axis of motion) of these links can be defined with
four scalar parameters.  The algorithm implemented in this library uses the
Modified Denavit-Hartenberg (MDH) parameters.  This convention assumes that the
link's coordinate systems have been placed according to the following rules:
\f[
\begin{tabular}{lcl}
$\hat{z}_i$ & ~~ & lies along the axis of motion of the joint, \\
$\hat{x}_i$ & ~~ & lies along the common normal between $\hat{z}_i$ and
                   $\hat{z}_{i+1}$, and \\
$\hat{y}_i$ & ~~ & completes the right-handed coordinate system.
\end{tabular}
\f]
Then the four parameters that define the transformation from the previous
body's coordinate system (\f$i-1\f$) to this link's (\f$i\f$) as follows:

\f[
\begin{tabular}{lcl}
$a_i$ & ~~  & distance along $\hat{x}_{i-1}$ (common normal) from
              $\hat{z}_{i-1}$ to $\hat{z}_{i}$, \\
$\alpha_i$ && angle (in radians) about $\hat{x}_{i-1}$ from $\hat{z}_{i-1}$ to
              $\hat{z}_{i}$, \\
$d_i$      && distance along $\hat{z}_{i}$ from $\hat{x}_{i-1}$ (the inboard
              common normal) \\
           && ~~~ to $\hat{x}_{i}$ (the outboard common normal). \\
$\theta_i$ && the angle (in radians) about $\hat{z}_{i}$ from $\hat{x}_{i-1}$
              to $\hat{x}_{i}$.
\end{tabular}
\f]
For revolute joints, \f$\theta_i\f$ is the joint position variable, and for
prismatic joints, \f$d_i\f$ is the variable.  These parameters (along with the
joint position) is specified with the \c setMDHParameters function.

The \c setJointLimits function is used to specify the range (min and max) of
motion for the joint position variable.  Should the range be violated,
DynaMechs provides a mechanism by which compliant joint limit forces can be
simulated with a spring-damper system specified by the \c k_spring and \c
b_damper constants.  The compliant limit force is computed as follows: 

\f[
\tau \: = \: K_{spring} \: \Delta q - B_{damper} \: \dot{q} 
\f]
where \f$\Delta q \f$ is the amount by which the joint position exceeds the joint
limits.  This is a force along the joint axis in the case of prismatic joints,
and is a torque about the joint axis in the case of revolute joints.

In some cases, actuator dynamics can also be simulated with single DOF 
MDHLink s.  Currently only DC motors for revolute joints have been provided
with this library (see dmRevDCMotor) which can be ``assigned'' to  
dmRevoluteLink s.  This assignment is accomplished with the \c setActuator
function.  Calling \c setActuator with a NULL argument effectively unsets
any existing actuator.  A pointer to an assigned actuator can be retrieved with
the \c getActuator member function.  If no actuator has been assigned it
returns NULL.

The remainder of the functions are described in the dmLink reference
pages and are implemented in this class for the specific case of MDH-type
joints.  The exceptions is \c scongtxToInboardIrefl which is implemented in
the two derived subclasses:  dmRevoluteLink , and   dmPrismaticLink .
Note that  \c getNumDOFs  returns 1 for this and the derived classes.  As such
the joint variables \c q, \c qd}, \c qdd, and \c joint_input are
arrays of one element.

A configuration file reader, \c dmLoadFile_dm is being supplied with the
dmutils library that can be used to instantiate and intialize these link
objects.  See the  dmRevoluteLink  and  dmPrismaticLink  classes for the
format for each type of link

See also  dmRigidBody ,  dmLink,  dmPrismaticLink , 
dmRevoluteLink},  dmRevDCMotor ,   dmLoadFile_dm.
*/

class DM_DLL_API dmMDHLink : public dmRigidBody
{
public:
   enum {NUM_DOFS = 1};

public:
   ///
   dmMDHLink();
   ///
   virtual ~dmMDHLink();

   ///
   void setMDHParameters(Float a, Float alpha, Float d, Float theta);
   ///
   void getMDHParameters(Float *a, Float *alpha,
                         Float *d, Float *theta) const;

   ///
   void setJointLimits(Float min, Float max, Float k_spring, Float b_damper);
   ///
   void getJointLimits(Float *min, Float *max,
                       Float *k_spring, Float *b_damper) const;

   ///
   void setActuator(dmActuator *actuator);
   ///
   dmActuator *getActuator() const {return m_actuator;}

   ///
   inline int getNumDOFs() const {return NUM_DOFS;}
   ///
   void setState(Float q[], Float qd[]);
   ///
   void getState(Float q[], Float qd[]) const;
   ///
   void getPose(RotationMatrix R, CartesianVector p) const;

   ///
   //! v5.0, generate the spatial transformation matrix. I probably should later move this implementation to dmRigidBody class. 
   Matrix6F get_X_FromParent_Motion();

   //! DM v5.0 Function, 
   void RNEAOutwardFKID(dmRNEAStruct &link_val2_curr, 
                           dmRNEAStruct &link_val2_inboard,
                                       bool ExtForceFlag = false); // should make the second param const

   //! DM v5.0 Function,  for the first link. 
   void RNEAOutwardFKIDFirst(dmRNEAStruct &link_val2_curr, 
				CartesianVector p_ref_ICS,  
                                RotationMatrix  R_ref_ICS, 
                                Vector6F a_ini,
                                Vector6F v_ini = Vector6F::Zero(), 
                                      bool ExtForceFlag = false
                                );
   //! DM v5.0 Function, 
   void RNEAInwardID(dmRNEAStruct &link_val2_curr,
                    dmRNEAStruct &link_val2_inboard);

   //! DM v5.0 Function
   void compute_AccBias_First(dmRNEAStruct &link_val2_curr);
   //! DM v5.0 Function
   void compute_AccBias(dmRNEAStruct &link_val2_curr,
                                 dmRNEAStruct &link_val2_inboard) ;
   //! DM v5.0 Function
   void computeSpatialVelAndICSPoseFirst(  dmRNEAStruct &link_val2_curr,
                                           CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
                                           RotationMatrix  R_ref_ICS,
                                           Vector6F a_ini);
   //! DM v5.0 Function
   void computeSpatialVelAndICSPose(  dmRNEAStruct &link_val2_curr,
                                                 dmRNEAStruct &link_val2_inboard);

   ///
   inline void setJointInput(Float joint_input[])
   {
      m_joint_input = joint_input[0];
   }

// Link-to-Link transformation functions:
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
   virtual void scongtxToInboardIrefl(const SpatialTensor N,
                                      SpatialTensor I) const = 0;
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

// Rendering functions:
   ///
   virtual void draw() const = 0;

protected:
   // not implemented
   dmMDHLink(const dmMDHLink &);
   dmMDHLink &operator=(const dmMDHLink &);

   void initABVars();

// Accessor routines:
   virtual void setJointPos(Float q) = 0;
   virtual Float getJointPos() const = 0;

   virtual void computeZeta(const CartesianVector omega_inboard,
                            const CartesianVector omega_curr,
                            SpatialVector zeta) = 0;

protected:
   dmActuator *m_actuator;    // a ptr to object to process joint input

   Float m_aMDH, m_alphaMDH, m_dMDH, m_thetaMDH; // MDH-params (Craig)
   Float m_qd, m_qdd;                            // joint vel. and accel.
   Float m_min_joint_pos, m_max_joint_pos;       // joint limits (d or theta)

   int m_joint_axis_index;                       // 2-revolute, 5-prismatic

   Float m_joint_input;
   Float m_tau_limit;
   Float m_salpha, m_calpha;
   Float m_stheta, m_ctheta;
   Float m_sasa, m_saca, m_cacamsasa, m_saca2;
   Float m_stst, m_stct, m_ctctmstst, m_stct2;

// vars for the AB algorithm member functions.
   Float         m_minv;            // inv(m_star) - was k_star;
   SpatialVector m_n_minv;          // n*inv(m_star)
   Float         m_tau_star;
};

#endif
