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
 *     File: dmMobileBaseLink.cpp
 *   Author: Scott McMillan
 *  Summary: Class implementation for mobile reference members.
 *****************************************************************************/

#include "dm.h"

#include "dmRigidBody.hpp"
#include "dmMobileBaseLink.hpp"
#include "dmEnvironment.hpp"

//======================================================================
// class dmMobileBaseLink: public dmRigidBody
//======================================================================

//----------------------------------------------------------------------------
//    Summary: Class constructor for 6 dof links
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmMobileBaseLink::dmMobileBaseLink() : dmRigidBody()
{
   for (int i = 0; i < 6; i++)
   {
      m_vel[i] = 0.0;
      // m_acc[i] = 0.0;
   }
}

//----------------------------------------------------------------------------
dmMobileBaseLink::~dmMobileBaseLink()
{
}

//----------------------------------------------------------------------------
//    Summary: sets the 6 dof joint position variables
// Parameters: q - orientation quaternion relative to predecessor link
//             p - position of link's c.s. wrt inboard link expressed
//                 in inboard link coordinates
//    Returns: none
//----------------------------------------------------------------------------
void dmMobileBaseLink::setJointPos(dmQuaternion q, CartesianVector p)
{
   // normalize the quaternion
   ::normalizeQuat(q);

   m_quat[0]  = q[0];
   m_quat[1]  = q[1];
   m_quat[2]  = q[2];
   m_quat[3]  = q[3];

   m_p[0]   = p[0];
   m_p[1]   = p[1];
   m_p[2]   = p[2];

   ::buildRotMat(m_quat, m_R);
}

//----------------------------------------------------------------------------
//    Summary: set the state of the 6 dof link
//             For EXTERNAL USE ONLY - should not be called in the normal
//             course of simulation because we do not want to reset the contact
//             model, except when the state is being overridden externally.
//
// Parameters: q = [quat pos]
//                  quat - normalized quaternion
//                  pos  - position of link's c.s. wrt inboard link
//                         expressed in inboard link coordinates
//             qd = vel  - velocity state variables (relative to inboard link)
//    Returns: none
//----------------------------------------------------------------------------
void dmMobileBaseLink::setState(Float q[], Float qd[])
{
   setJointPos(&q[0], &q[4]);

   if (qd)
   {
      for (int i=0; i<6; i++)
      {
         m_vel[i] = qd[i];      // angular velocity and  linear velocity
      }
   }

   /* FIXME - I don't think the following is quite right but it must be done
              with the current contact model when overriding the state. */
   // if (m_contact_model) m_contact_model->reset();

   // Now we must reset the force objects 'cause that is what the contact model
   // has become...
   for (unsigned int i=0; i<m_force.size(); i++)
   {
      m_force[i]->reset();
   }
}

//----------------------------------------------------------------------------
//    Summary: retrieve the state of the 6 dof link
// Parameters:
//    Returns: q = [quat pos]
//                  quat - normalized quaternion
//                  pos  - position of link's c.s. wrt inboard link
//                         expressed in inboard link coordinates
//             qd = vel  - velocity state variables (relative to inboard link)
//----------------------------------------------------------------------------
void dmMobileBaseLink::getState(Float q[], Float qd[]) const
{
   q[0] = m_quat[0];
   q[1] = m_quat[1];
   q[2] = m_quat[2];
   q[3] = m_quat[3];

   q[4] = m_p[0];
   q[5] = m_p[1];
   q[6] = m_p[2];

   if (qd)
   {
      for (int i=0; i<6; i++)
      {
         qd[i]  = m_vel[i];
      }
      qd[6] = 0;  // unused
   }
}


//----------------------------------------------------------------------------
//    Summary: retrieve the position and orientation matrix of the 6 dof link
// Parameters: none
//    Returns: R - orientation matrix relative to inboard link
//             p - position of link's c.s. wrt inboard link expressed in
//                 inboard link coordinates
//----------------------------------------------------------------------------
void dmMobileBaseLink::getPose(RotationMatrix R, CartesianVector p) const
{
   for (unsigned int i=0; i<3; i++)
   {
      for (unsigned int j=0; j<3; j++)
      {
         R[i][j] = m_R[i][j];
      }
      p[i] = m_p[i];
   }
}

// ---------------------------------------------------------------------
// Function : stxFromInboard
// Purpose  : Spatial transform of 6d vector from inboard CS
// Inputs   : prev - 6d vector wrt inboard link's CS
// Outputs  : curr - 6d vector wrt this link's CS
// ---------------------------------------------------------------------
void dmMobileBaseLink::stxFromInboard(const SpatialVector prev,
                                      SpatialVector curr) const
{
   rtxFromInboard(&prev[0], &curr[0]);

   CartesianVector temp;
   temp[0] =   m_p[2]*prev[1] - m_p[1]*prev[2] + prev[3];
   temp[1] = - m_p[2]*prev[0] + m_p[0]*prev[2] + prev[4];
   temp[2] =   m_p[1]*prev[0] - m_p[0]*prev[1] + prev[5];
   rtxFromInboard(temp, &curr[3]);
}

//----------------------------------------------------------------------------
void dmMobileBaseLink::stxToInboard(const SpatialVector ,
                                    SpatialVector ) const
{
   exit(1);
}

//----------------------------------------------------------------------------
void dmMobileBaseLink::rcongtxToInboardSym(const CartesianTensor ,
                                           CartesianTensor ) const
{
   exit(1);
}

//----------------------------------------------------------------------------
void dmMobileBaseLink::rcongtxToInboardGen(const CartesianTensor ,
                                           CartesianTensor ) const
{
   exit(1);
}

//----------------------------------------------------------------------------
void dmMobileBaseLink::scongtxToInboardIrefl(const SpatialTensor ,
                                             SpatialTensor ) const
{
   exit(1);
}

//----------------------------------------------------------------------------
//    Summary: beginning of AB's Forward Kinematics recursion, called by
//             dmSystem::ABDynamics function only, also computes any contact
//             forces and then m_beta (resultant bias force).
// Parameters: q, qd   - new state of body (provided by numerical integrator)
//             q[0..3] - normalized quaternion
//             q[4..6] - link position wrt inboard link (^{i-1}p_i)
//             qd - relative spatial velocity in inboard link coordinates
//             link_val_inboard - kinematic parameters of inboard link
//    Returns: link_val_curr - various kinematic parameters for the link.
//----------------------------------------------------------------------------
void dmMobileBaseLink::ABForwardKinematics(
   Float q[],  // float[7]
   Float qd[], // float[6]
   const dmABForKinStruct &link_val_inboard,
   dmABForKinStruct &link_val_curr)
{
   register int i, j;

   // set new state
   setJointPos(&q[0], &q[4]);

   for (i=0; i<6; i++)
   {
      m_vel[i] = qd[i];      // angular velocity and  linear velocity
   }

// Compute R_ICS and p_ICS for this link's coordinate system.
   for (i = 0; i < 3; i++)
   {
      link_val_curr.p_ICS[i] = link_val_inboard.p_ICS[i];
      for (j = 0; j < 3; j++)
      {
         link_val_curr.p_ICS[i] += link_val_inboard.R_ICS[i][j]*m_p[j];
      }
      rtxFromInboard(&link_val_inboard.R_ICS[i][0],
                     &link_val_curr.R_ICS[i][0]);
   }

   // Compute link angular velocity wrt link i
   CartesianVector omega_prev; // angular vel. of 6dof link wrt inboard link
   omega_prev[0] = link_val_inboard.v[0] + m_vel[0];
   omega_prev[1] = link_val_inboard.v[1] + m_vel[1];
   omega_prev[2] = link_val_inboard.v[2] + m_vel[2];
   rtxFromInboard( omega_prev, &link_val_curr.v[0] );

   // Compute link linear velocity wrt link i
   CartesianVector vel_prev; // linear velocity of 6dof link wrt inboard link
   CartesianVector w_cross_r;
   crossproduct( &link_val_inboard.v[0], m_p, w_cross_r );
   vel_prev[0] = link_val_inboard.v[3] + m_vel[3] + w_cross_r[0];
   vel_prev[1] = link_val_inboard.v[4] + m_vel[4] + w_cross_r[1];
   vel_prev[2] = link_val_inboard.v[5] + m_vel[5] + w_cross_r[2];
   rtxFromInboard( &vel_prev[0], &link_val_curr.v[3] );

   // compute zeta
   CartesianVector tem, wwr;
   crossproduct( &link_val_inboard.v[0], &m_vel[0], tem );
   rtxFromInboard( tem, &m_zeta[0] );

   crossproduct( &link_val_inboard.v[0], &m_vel[3], tem );
   crossproduct( &link_val_inboard.v[0], w_cross_r, wwr );
   tem[0] = 2.0*tem[0] + wwr[0];
   tem[1] = 2.0*tem[1] + wwr[1];
   tem[2] = 2.0*tem[2] + wwr[2];
   rtxFromInboard( tem, &m_zeta[3] );

#ifdef DM_HYDRODYNAMICS
   rtxFromInboard(link_val_inboard.v_f,  link_val_curr.v_f);
   rtxFromInboard(link_val_inboard.a_fg, link_val_curr.a_fg);
#endif

   // compute bias (velocity-dependent) force.
   computeBeta(link_val_curr, m_beta);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters: n_refl_curr - resultant AB inertia of all children combined
//                           and transformed to this link's CS
//             f_star_curr - resultant AB force exerted on this from
//                           all the outboard members (combined)
//    Returns:
//----------------------------------------------------------------------------
void dmMobileBaseLink::ABBackwardDynamics(
   const dmABForKinStruct &link_val_curr,
   SpatialVector f_star_curr,
   SpatialTensor N_refl_curr,
   SpatialVector f_star_inboard,
   SpatialTensor N_refl_inboard)
{
   register unsigned int i,j,k;

// compute force objects (if any)
   if (m_force.size())
   {
      SpatialVector force;
      for (j=0; j<m_force.size(); j++)
      {
         m_force[j]->computeForce(link_val_curr, force);

         for (i = 0; i < 6; i++)
         {
            m_beta[i] += force[i];
         }
      }
   }

   for (j = 0; j < 6; j++)
   {
      m_beta[j] += m_external_force[j];
   }

// Lower triangular portion of IStar required.
   for (j = 0; j < 6; j++)
   {
      m_beta_star[j] = m_beta[j] + f_star_curr[j];
      for (k = j; k < 6; k++)
      {
         m_I_star[j][k] = m_I_star[k][j] =
            N_refl_curr[j][k] + m_SpInertia[j][k];
      }
   }

   // a full spatial joint transmits no force or inertia across "joint"
   for (i=0; i<6; i++)
   {
      f_star_inboard[i] = 0.0;
      for (j=0; j<6; j++)
      {
         N_refl_inboard[i][j] = 0.0;
      }
   }

// Perform the root-free cholesky decomposition (IStar = L D L').
   register Float tem;
   for (k = 0; k < 5; k++)
   {
      for (i = 5; i > k; i--)
      {
         tem = m_I_star[i][k]/m_I_star[k][k];

         for (j = i; j > k; j--)
         {
            m_I_star[i][j] -= m_I_star[j][k]*tem;
         }
         m_I_star[i][k] = tem;
      }
   }

}

//----------------------------------------------------------------------------
void dmMobileBaseLink::ABBackwardDynamicsN(
   const dmABForKinStruct &link_val_curr,
   SpatialVector f_star_inboard,
   SpatialTensor N_refl_inboard)
{
   register unsigned int i,j,k;

// compute force objects (if any)
   if (m_force.size())
   {
      SpatialVector force;
      for (j=0; j<m_force.size(); j++)
      {
         m_force[j]->computeForce(link_val_curr, force);

         for (i = 0; i < 6; i++)
         {
            m_beta[i] += force[i];
         }
      }
   }

   for (j = 0; j < 6; j++)
   {
      m_beta[j] += m_external_force[j];
   }

   // Compute IStar
   for (j = 0; j < 6; j++)
   {
      m_beta_star[j] = m_beta[j];
      for (k = j; k < 6; k++)
      {
         m_I_star[j][k] = m_I_star[k][j] = m_SpInertia[j][k];
      }
   }

   // a full spatial joint transmits no force or inertia across "joint"
   for (i=0; i<6; i++)
   {
      f_star_inboard[i] = 0.0;
      for (j=0; j<6; j++)
      {
         N_refl_inboard[i][j] = 0.0;
      }
   }

// Perform the root-free cholesky decomposition (IStar = L D L').
   register Float tem;
   for (k = 0; k < 5; k++)
   {
      for (i = 5; i > k; i--)
      {
         tem = m_I_star[i][k]/m_I_star[k][k];

         for (j = i; j > k; j--)
         {
            m_I_star[i][j] -= m_I_star[j][k]*tem;
         }
         m_I_star[i][k] = tem;
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: end of second AB Backward Dynamics recursion AND beginning of
//             third AB Forward Accelerations recursion
// Parameters: a_inboard - spatial acceleration of inboard link
//             a_curr    - spatial acceleration of 6 dof link
//             qd        - rate of change of position state variables
//             qdd       - rate of change of velocity state variables
//----------------------------------------------------------------------------
void dmMobileBaseLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              SpatialVector a_curr,
                                              Float qd[],
                                              Float qdd[])
{
   register int i, k;

// Note that the root-free cholesky decomposition (IStar = L D L')
// is now performed at the end of  ABBackwardDynamics(N) because it is
// required for propagation of constraint forces.

// Calculate the right hand side of the system of equations.
   for (k = 0; k < 6; k++)
   {
      a_curr[k] = m_beta_star[k];
   }

   for (k = 0; k < 6; k++)
   {
      for (i = k + 1; i < 6; i++)
      {
         a_curr[i] -= m_I_star[i][k]*a_curr[k];
      }
   }

   for (k = 0; k < 6; k++)
   {
      a_curr[k] = a_curr[k]/m_I_star[k][k];
   }

   for (k = 5; k > -1; k--)
   {
      for (i = k - 1; i > -1; i--)
      {
         a_curr[i] -= m_I_star[k][i]*a_curr[k];
      }
   }

   // a_curr = ^ia_i
   SpatialVector temp;
   stxFromInboard( a_inboard, temp );

   for (i = 0; i < 6; i++)
      temp[i] = a_curr[i] - temp[i] - m_zeta[i];

   rtxToInboard( &temp[0], &qdd[0] );
   rtxToInboard( &temp[3], &qdd[3] );
   qdd[6] = 0.0;

// transfer results to derivative of state vector
   // convert vel[0..2] = to Quat. rates [vxd vyd vzd sd].
   qd[0] = 0.5*( m_vel[0]*m_quat[3]+ m_vel[1]*m_quat[2] - m_vel[2]*m_quat[1]);
   qd[1] = 0.5*(-m_vel[0]*m_quat[2]+ m_vel[1]*m_quat[3] + m_vel[2]*m_quat[0]);
   qd[2] = 0.5*( m_vel[0]*m_quat[1]- m_vel[1]*m_quat[0] + m_vel[2]*m_quat[3]);
   qd[3] =-0.5*(m_vel[0]*m_quat[0] + m_vel[1]*m_quat[1] + m_vel[2]*m_quat[2]);

   qd[4] = m_vel[3];
   qd[5] = m_vel[4];
   qd[6] = m_vel[5];
}


//----------------------------------------------------------------------------
//    Summary: The current link (i) is to be eliminated from the force-balance
//             equation of its predecessor.  This function computes the update
//             to the coefficient of loop k's constraint forces in the
//             predecessor link's force-balance equation as a result of the
//             elimination.
// Parameters: Xik_curr - coefficient of loop k's constraint forces in
//                        link i's force balance equation (ignored)
//             columns_Xik - number of columns in Xik
//    Returns: Xik_prev - update to the coefficient of loop k's constraint
//                        forces in the predecessor's force-balance equation
//                        as a result of the elimination
//----------------------------------------------------------------------------
void dmMobileBaseLink::XikToInboard(Float **,
                                    Float **Xik_prev,
                                    int columns_Xik) const
{
   register int r, c;

   for (r = 0; r < 6; r++)
      for (c = 0; c < columns_Xik; c++)
         Xik_prev[r][c] = 0.0;  // secondary joint doesn't constrain parent
   // through the 6 dof link
}



//----------------------------------------------------------------------------
//    Summary: Compute the new coefficient of loop n's constraint forces
//             in the loop-closure constraint equation for loop k after
//             elimination of the current link (i) in favor of its predecessor
// Parameters: Xik - coefficient of loop k's constraint forces in
//                   link i's force balance equation
//             cols_Xik - number of columns in Xik
//             Xin - coefficient of loop n's constraint forces in
//                   link i's force balance equation
//             cols_Xin - number of columns in Xin
//    Returns: Bkn - coefficient of loop n's constraint forces in the
//                   loop-closure constraint equation for loop k
//----------------------------------------------------------------------------
void dmMobileBaseLink::BToInboard( Float **Bkn,
                                   Float **Xik, int cols_Xik,
                                   Float **Xin, int cols_Xin ) const
{
   register int i, j, k;

// Note that the root-free cholesky decomposition (IStar = L D L')
// is already performed at the end of ABBackwardDynamics(N) and stored
// in m_I_star.

   Float invI_Xin[6][6];  // only need 6 x (cols_Xin)

   // Use Cholesky decomposition to columnwise compute inv(I_star)*X_in = ?,
   // using solution technique of computing I_star * ? = X_in.

   for (j = 0; j < cols_Xin; j++)
   {
      for (k = 0; k < 6; k++)
         invI_Xin[k][j] = Xin[k][j];

      for (k = 0; k < 6; k++)
         for (i = k + 1; i < 6; i++)
            invI_Xin[i][j] -= m_I_star[i][k]*invI_Xin[k][j];

      for (k = 0; k < 6; k++)
         invI_Xin[k][j] = invI_Xin[k][j]/m_I_star[k][k];

      for (k = 5; k > -1; k--)
         for (i = k - 1; i > -1; i--)
            invI_Xin[i][j] -= m_I_star[k][i]*invI_Xin[k][j];
   }

   // Compute Xik' * inv(I_star) * Xin and add to Bkn.
   for (i = 0; i < cols_Xik; i++)
      for (j = 0; j < cols_Xin; j++)
         for (k = 0; k < 6; k++)
            Bkn[i][j] += Xik[k][i]*invI_Xin[k][j];
}


//----------------------------------------------------------------------------
//    Summary: Compute the new bias term (zetak) in loop k's loop-closure
//             constraint equation by eliminating link i in favor of its
//             predecessor.
// Parameters: Xik - coefficient of loop k's constraint forces in current
//                   link i's force balance equation
//             cols_Xik - number of columns in Xik
//    Returns: zetak - bias term in loop k's loop-closure constraint
//                     equation after elimination of link i
//----------------------------------------------------------------------------
void dmMobileBaseLink::xformZetak( Float *zetak,
                                   Float **Xik, int cols_Xik) const
{
   register int i, k;

// Note that the root-free cholesky decomposition (IStar = L D L')
// is already performed at the end of ABBackwardDynamics(N) and stored
// in m_I_star.

   // Use Cholesky decomposition to columnwise compute inv(I_star)*beta = ?,
   // using solution technique of computing I_star * ? = beta.

   Float invI_beta[6];

   for (k = 0; k < 6; k++)
      invI_beta[k] = m_beta_star[k];

   for (k = 0; k < 6; k++)
      for (i = k + 1; i < 6; i++)
         invI_beta[i] -= m_I_star[i][k]*invI_beta[k];

   for (k = 0; k < 6; k++)
      invI_beta[k] = invI_beta[k]/m_I_star[k][k];

   for (k = 5; k > -1; k--)
      for (i = k - 1; i > -1; i--)
         invI_beta[i] -= m_I_star[k][i]*invI_beta[k];

   // Compute Xik' * inv(I_star) * beta and subtract from Zeta_k
   for (i = 0; i < cols_Xik; i++)
      for (k = 0; k < 6; k++)
         zetak[i] -= Xik[k][i]*invI_beta[k];
}


//----------------------------------------------------------------------------
//    Summary: third (final) forward recursion of the AB algorithm to compute
//             the link's state derivative (velocity and acceleration) for
//             links in systems containing loops
// Parameters: a_inboard - spatial acceleration of inboard link
//             LB - set of loops to which current link belongs
//             num_elements_LB - number of loops in LB
//             Xik - Xik[k] is the coefficient of loop k's constraint forces
//                   in the force-balance equation of link i
//             constraint_forces - constraint_forces[k][j] is the value
//                                 of the constraint force for the jth
//                                 constrained DOF at secondary joint k
//             num_constraints - array containing the number of constraints
//                               at each secondary joint
//    Returns: a_curr    - spatial accel of this link
//             qd - array whose first (and only) element is the time derivative
//                  of joint position - joint velocity
//             qdd - array whose first (and only) element is the time
//                  derivative of joint velocity - joint acceleration
//----------------------------------------------------------------------------
void dmMobileBaseLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              unsigned int *LB,
                                              unsigned int num_elements_LB,
                                              Float ***Xik,
                                              Float **constraint_forces,
                                              unsigned int *num_constraints,
                                              SpatialVector a_curr,
                                              Float qd[],
                                              Float qdd[])
{
   unsigned int i;
   Float forces_of_constraint[6] = {0,0,0,0,0,0};
   for (i = 0; i < num_elements_LB; i++)
   {
      unsigned int k = LB[i];
      for (unsigned int r = 0; r < 6; r++)
         for (unsigned int j = 0; j < num_constraints[k]; j++)
            forces_of_constraint[r] += Xik[k][r][j]*constraint_forces[k][j];
   }

// Note that the root-free cholesky decomposition (IStar = L D L')
// is now performed at the end of  ABBackwardDynamics(N) because it is
// required for propagation of constraint forces.

// Calculate the right hand side of the system of equations.
   register int k;
   for (k = 0; k < 6; k++)
   {
      a_curr[k] = m_beta_star[k] + forces_of_constraint[k];
   }

   for (k = 0; k < 6; k++)
   {
      for (int i = k + 1; i < 6; i++)
      {
         a_curr[i] -= m_I_star[i][k]*a_curr[k];
      }
   }

   for (k = 0; k < 6; k++)
   {
      a_curr[k] = a_curr[k]/m_I_star[k][k];
   }

   for (k = 5; k > -1; k--)
   {
      for (int i = k - 1; i > -1; i--)
      {
         a_curr[i] -= m_I_star[k][i]*a_curr[k];
      }
   }

   // a_curr = ^ia_i
   SpatialVector temp;
   stxFromInboard( a_inboard, temp );

   for (i = 0; i < 6; i++)
      temp[i] = a_curr[i] - temp[i] - m_zeta[i];

   rtxToInboard( &temp[0], &qdd[0] );
   rtxToInboard( &temp[3], &qdd[3] );
   qdd[6] = 0.0;

// transfer results to derivative of state vector
   // convert vel[0..2] = to Quat. rates [vxd vyd vzd sd].
   qd[0] = 0.5*( m_vel[0]*m_quat[3]+ m_vel[1]*m_quat[2] - m_vel[2]*m_quat[1]);
   qd[1] = 0.5*(-m_vel[0]*m_quat[2]+ m_vel[1]*m_quat[3] + m_vel[2]*m_quat[0]);
   qd[2] = 0.5*( m_vel[0]*m_quat[1]- m_vel[1]*m_quat[0] + m_vel[2]*m_quat[3]);
   qd[3] =-0.5*(m_vel[0]*m_quat[0] + m_vel[1]*m_quat[1] + m_vel[2]*m_quat[2]);

   qd[4] = m_vel[3];
   qd[5] = m_vel[4];
   qd[6] = m_vel[5];
}






//---------------------------------------------------
Matrix6XF dmMobileBaseLink::jcalc()
{
    Matrix6XF  S(6,6);
    S = Matrix6XF::Identity(6,6);
    return S;
}


//-------------------------------------------------------------------
/* The following codes are mostly written in Pure Eigen style. 
 Consider to gradually optimize the following code in the future 
 by switching to those more efficient, DynaMechs-native functions, such as stxToInboard, stxFromInboard etc. */
//! DM v5.0 function, 
void dmMobileBaseLink::RNEAOutwardFKID(  dmRNEAStruct &link_val2_curr, 
								dmRNEAStruct &link_val2_inboard,
								bool ExtForceFlag)
{
	exit(-1);
}


//-------------------------------------------------------------------
//! DM v5.0 function, 
void dmMobileBaseLink::RNEAOutwardFKIDFirst(  dmRNEAStruct &link_val2_curr,
									 CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
									 RotationMatrix  R_ref_ICS,  
									 Vector6F a_ini, 
									 Vector6F v_ini,
									 bool ExtForceFlag)
{
	// compute R_ICS and p_ICS)
	for (int i = 0; i < 3; i++)
	{
		link_val2_curr.p_ICS[i] = p_ref_ICS[i];
		for (int j = 0; j < 3; j++)
		{
			link_val2_curr.p_ICS[i] += R_ref_ICS[i][j] * m_p[j]; // position 
			rtxFromInboard(&(R_ref_ICS[i][0]),
						   &(link_val2_curr.R_ICS[i][0])); //orientation			
		}
	}
	

	Float q[7], qd[7];
	getState(q,qd);
	
	Vector6F vJ;;
	rtxFromInboard( qd, vJ.data() );
	rtxFromInboard( qd+3, vJ.data()+3);
	
	// qd for Floating Base contains a rogue zero as the last element
	
	
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = vJ;
	
	// Spatial acceleration here my friends
	link_val2_curr.a = X * a_ini + link_val2_curr.qdd;
	
	Matrix6F I = getSpatialInertiaMatrix();
	
	// Initialize f
	link_val2_curr.f = I *  link_val2_curr.a  + crf(link_val2_curr.v) * I * link_val2_curr.v;	
	
	// Trsuting Yipings code here
	if (ExtForceFlag != false)
	{
		for (int i = 0; i < m_force.size(); i++)// if there are external forces
		{
			// currently there is only ground contact force.
			SpatialVector ext_f;
			m_force[i]->computeForce( link_val2_curr, ext_f );//ext_f is ALREADY with respect to the body's coordinate system
			Vector6F Ext_f;
			Ext_f<< ext_f[0], ext_f[1], ext_f[2], ext_f[3], ext_f[4], ext_f[5];
			link_val2_curr.f -= Ext_f;
		}	
	}
}


/*//--------------------------------------------------------------------
//! DM v5.0 function, 
void dmMobileBaseLink::RNEAInwardID(dmRNEAStruct &link_val2_curr,
							 dmRNEAStruct &link_val2_inboard)
{
	cout << "f at mobile base " << link_val2_curr.f << endl;
	link_val2_curr.tau = link_val2_curr.f;
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_inboard.f += X.transpose() *  link_val2_curr.f;
}*/