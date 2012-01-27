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
 *     File: dmQuaternionLink.cpp
 *   Author: Duane Marhefka
 *  Created: September 2000
 *  Summary: Class implementation for links with (quaternion) ball-joints
 *****************************************************************************/

#include "dmQuaternionLink.hpp"

//============================================================================
// class dmQuaternionLink: public dmRigidBody
//============================================================================

//----------------------------------------------------------------------------
//    Summary: class constructor
// Parameters:
//    Returns: none
//----------------------------------------------------------------------------
dmQuaternionLink::dmQuaternionLink() : dmRigidBody()
{
   for (int i=0; i<3; i++)
   {
      m_qd[i] = 0.0;
      m_qdd[i] = 0.0;
      m_joint_input[i] = 0.0;
   }

   QuaternionDM q = {0.0, 0.0, 0.0, 1.0};
   setJointPos(q);
}

//----------------------------------------------------------------------------
void dmQuaternionLink::initABVars()
{
   register int i, j, k;
   CartesianTensor tem;

// 3.0.5: I_star
   for (j = 0; j < 6; j++) {
      for (k = 0; k < 6; k++) {
         m_I_star[j][k] = m_SpInertia[j][k];
         m_N_refl[j][k] = m_N_refl[k][j] = 0.0;
      }
   }

// 3.1, 3.2, 3.3, 3.4 (_eta[0..5][0..2] = m_I_star[0..5][0..2])
   for (j = 0; j < 3; j++) {
      for (i = 0; i < 3; i++) {
         tem[j][i] = m_I_star[j][i];
         m_n_minv[j][i] = 0.0;
      }
      m_n_minv[j][j] = 1.0;
   }

   matrixInverse3PD(tem, m_minv);

   for (j = 3; j < 6; j++) {
      for (i = 0; i < 3; i++) {
         m_n_minv[j][i] = m_I_star[j][0]*m_minv[0][i] +
            m_I_star[j][1]*m_minv[1][i] +
            m_I_star[j][2]*m_minv[2][i];
      }
   }

   for (j = 3; j < 6; j++)
      for (k = j; k < 6; k++) {
         m_N_refl[j][k] = m_N_refl[k][j] =
            m_I_star[j][k] - (m_n_minv[j][0]*m_I_star[k][0] +
                              m_n_minv[j][1]*m_I_star[k][1] +
                              m_n_minv[j][2]*m_I_star[k][2]);
      }
}

//----------------------------------------------------------------------------
//    Summary: compute the 3x3 inverse of the symmetric PD matrix using
//             Cholesky Decomposition
// Parameters: A - 3x3 symmetric PD matrix
//    Returns: A_inv - inverse of A
//----------------------------------------------------------------------------
inline void dmQuaternionLink::matrixInverse3PD(CartesianTensor A,
                                               CartesianTensor A_inv)
{
   register int i, j, k;
   register Float temp;
   CartesianTensor T;

   // transfer lower triangle to temp matrix.
   for (i = 0; i < 3; i++) {
      for (j = i; j < 3; j++) {
         A_inv[i][j] = A_inv[j][i] = 0.0;
         T[j][i] = A[i][j];
      }
      A_inv[i][i] = 1.0;
   }

// Perform the root-free cholesky decomposition (T = L D L').
   for (k = 0; k < 2; k++)
   {
      for (i = 2; i > k; i--)
      {
         temp = T[i][k]/T[k][k];

         for (j = i; j > k; j--)
            T[i][j] -= T[j][k]*temp;

         T[i][k] = temp;
      }
   }

// Calculate the right hand side of three systems of equations.
   for (j = 0; j < 3; j++)
   {
      for (k = 0; k < 3; k++)
         for (i = k + 1; i < 3; i++)
            A_inv[i][j] -= T[i][k]*A_inv[k][j];

      for (k = 0; k < 3; k++)
         A_inv[k][j] = A_inv[k][j]/T[k][k];

      for (k = 2; k > -1; k--)
         for (i = k - 1; i > -1; i--)
            A_inv[i][j] -= T[k][i]*A_inv[k][j];
   }
}

//----------------------------------------------------------------------------
//    Summary: Set the joint quaternion
// Parameters: q - quaternion
//    Returns: none
//----------------------------------------------------------------------------
inline void dmQuaternionLink::setJointPos(QuaternionDM q)
{
   // normalize the quaternion
   ::normalizeQuat(q);

   m_q[0] = q[0];
   m_q[1] = q[1];
   m_q[2] = q[2];
   m_q[3] = q[3];

   ::buildRotMat(m_q, m_R);
}


//----------------------------------------------------------------------------
//    Summary: set the state (position and velocity) variables and reset any
//             contacts in case there is any built up spring forces
// Parameters:  q - array of 4 elements defining joint position
//             qd - array of 3 elements defining joint velocity
//    Returns: none
//----------------------------------------------------------------------------
void dmQuaternionLink::setState(Float q[], Float qd[])
{
   for (int i = 0; i < 3; i++)
   {
      m_qd[i] = qd[i];
   }

   setJointPos(&q[0]);

   /* FIXME - I don't think the following is quite right but it must be done
      with the current contact model when overriding the state. */
   // So it doesn't try to stick to a previous point, This may be wrong:
   // if (m_contact_model) m_contact_model->reset();

   // Now we must reset the force objects 'cause that is what the contact model
   // has become...
   for (unsigned int j=0; j<m_force.size(); j++)
   {
      m_force[j]->reset();
   }
}

//----------------------------------------------------------------------------
//    Summary: retrieve the current state of the link (joint position and
//             velocity variables)
// Parameters:
//    Returns: q  - array of 4 joint position variables
//             qd - array of 3 joint velocity variables + 0
//----------------------------------------------------------------------------
void dmQuaternionLink::getState(Float q[], Float qd[]) const
{
   for (int i=0; i<3; i++)
   {
      qd[i] = m_qd[i];
      q[i] =  m_q[i];
   }
   q[3] = m_q[3];
   qd[3] = 0.0;  // Assumes qd[] is allocated to same dimensions as q[].
}

//----------------------------------------------------------------------------
void dmQuaternionLink::getPose(RotationMatrix R, CartesianVector p) const
{
   R[0][0] = m_R[0][0];
   R[1][0] = m_R[1][0];
   R[2][0] = m_R[2][0];

   R[0][1] = m_R[0][1];
   R[1][1] = m_R[1][1];
   R[2][1] = m_R[2][1];

   R[0][2] = m_R[0][2];
   R[1][2] = m_R[1][2];
   R[2][2] = m_R[2][2];

   p[0] = m_p[0];
   p[1] = m_p[1];
   p[2] = m_p[2];
}


//----------------------------------------------------------------------------
//    Summary: first forward kinematics recursion of the AB simulation alg.
// Parameters: q - current joint position computed by num. integration alg.
//             qd - current joint velocity computed by the num. int. alg.
//             link_val_inboard - struct containing kinematic (state dependent)
//                    quantities for the inboard (parent) link/body
//    Returns: link_val_curr - struct filled with kinematic quantities for this
//                    link.
//----------------------------------------------------------------------------
void dmQuaternionLink::ABForwardKinematics(
   Float q[],
   Float qd[],
   const dmABForKinStruct &link_val_inboard,
   dmABForKinStruct &link_val_curr)
{
   register int i, j;

   // set the new state
   for (i = 0; i < 3; i++)
   {
      m_qd[i] = qd[i];
   }
   setJointPos(&q[0]);

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

   // 2.2
   stxFromInboard(link_val_inboard.v, link_val_curr.v);
   link_val_curr.v[0] += m_qd[0];
   link_val_curr.v[1] += m_qd[1];
   link_val_curr.v[2] += m_qd[2];

   // 2.7 compute zeta
   CartesianVector tmp, tmp1;

   crossproduct(&link_val_inboard.v[0], m_p, tmp);
   crossproduct(&link_val_inboard.v[0], tmp, tmp1);
   rtxFromInboard(tmp1, &m_zeta[3]);

   crossproduct(&link_val_curr.v[0], &m_qd[0], &m_zeta[0]);

#ifdef DM_HYDRODYNAMICS
   rtxFromInboard(link_val_inboard.v_f,  link_val_curr.v_f);
   rtxFromInboard(link_val_inboard.a_fg, link_val_curr.a_fg);
#endif

   // 2.8
   computeBeta(link_val_curr, m_beta);
}

//----------------------------------------------------------------------------
//    Summary: the second backward dynamics recursion of the AB algorithm -
//               compute needed quantities for this link
// Parameters: f_star_curr - AB bias force reflected across and transformed
//                      from outboard link to this one
//             I_star_curr - AB spatial inertia reflected across and
//                      transformed  from outboard link to this one
//    Returns: f_star_inboard - AB bias force computed for this link, reflected
//                      across and transformed to the inboard link
//             I_star_inboard - AB spatial inertia computed for this link,
//                      reflected across and transformed to the inboard link
//----------------------------------------------------------------------------
void dmQuaternionLink::ABBackwardDynamics(
   const dmABForKinStruct &link_val_curr,
   SpatialVector f_star_curr,
   SpatialTensor N_refl_curr,
   SpatialVector f_star_inboard,
   SpatialTensor N_refl_inboard)
{
   register int j, k;
   CartesianTensor temp;

// compute forceobjects (if any)
   if (m_force.size())
   {
      SpatialVector force;
      for (unsigned int i=0; i<m_force.size(); i++)
      {
         m_force[i]->computeForce(link_val_curr, force);

         for (j = 0; j < 6; j++)
         {
            m_beta[j] += force[j];
         }
      }
   }

   for (j = 0; j < 6; j++)
   {
      m_beta[j] += m_external_force[j];
   }

// 3.0.5  and  3.0.8
   for (j = 0; j < 6; j++)
   {
      m_beta_star[j] = m_beta[j] + f_star_curr[j];
      for (k = j; k < 6; k++)
      {
         m_I_star[j][k] = m_I_star[k][j] =
            N_refl_curr[j][k] + m_SpInertia[j][k];
      }
   }

// 3.1, 3.2, 3.3
   for (j = 0; j < 3; j++)
   {
      for (k = 0; k < 3; k++)
      {
         temp[j][k] = m_I_star[j][k];
      }
   }

   matrixInverse3PD(temp, m_minv);

   // top half of n initialized to 1_3
   for (j = 3; j < 6; j++)
   {
      for (k = 0; k < 3; k++)
      {
         m_n_minv[j][k] = m_I_star[j][0]*m_minv[0][k] +
            m_I_star[j][1]*m_minv[1][k] +
            m_I_star[j][2]*m_minv[2][k];
      }
   }

// 3.4 - three blocks of m_N_refl initialized to 0_3x3 previously
   for (j = 3; j < 6; j++)
   {
      for (k = j; k < 6; k++)
      {
         m_N_refl[j][k] = m_N_refl[k][j] =
            m_I_star[j][k] - (m_n_minv[j][0]*m_I_star[k][0] +
                              m_n_minv[j][1]*m_I_star[k][1] +
                              m_n_minv[j][2]*m_I_star[k][2]);
      }
   }

// 3.6
   for (j = 0; j < 3; j++)
   {
      m_tau_star[j] = (m_beta_star[j] + m_joint_input[j] -
                       m_joint_friction*m_qd[j]);
   }

// 3.7
   m_gamma[0] = m_beta_star[0] - m_tau_star[0];
   m_gamma[1] = m_beta_star[1] - m_tau_star[1];
   m_gamma[2] = m_beta_star[2] - m_tau_star[2];
   for (j = 3; j < 6; j++)
   {
      m_gamma[j] = m_beta_star[j] -
         (m_n_minv[j][0]*m_tau_star[0] + m_N_refl[j][3]*m_zeta[3] +
          m_n_minv[j][1]*m_tau_star[1] + m_N_refl[j][4]*m_zeta[4] +
          m_n_minv[j][2]*m_tau_star[2] + m_N_refl[j][5]*m_zeta[5]);
   }

// 3.8 and 3.5
   stxToInboard(m_gamma, f_star_inboard);
   scongtxToInboardIrefl(m_N_refl, N_refl_inboard);
}


//----------------------------------------------------------------------------
//    Summary: the second backward dynamics recursion of the AB algorithm -
//               compute needed quantities for this link when this link is a
//               leaf node in the tree structure (this requires much less
//               computation than ABBackwardDynamics function)
// Parameters: none
//    Returns: f_star_inboard - AB bias force computed for this link, reflected
//                      across and transformed to the inboard link
//             I_star_inboard - AB spatial inertia computed for this link,
//                      reflected across and transformed to the inboard link
//----------------------------------------------------------------------------
void dmQuaternionLink::ABBackwardDynamicsN(
   const dmABForKinStruct &link_val_curr,
   SpatialVector f_star_inboard,
   SpatialTensor N_refl_inboard)
{
   register int j;

// compute force objects (if any)
   if (m_force.size())
   {
      SpatialVector force;
      for (unsigned int i=0; i<m_force.size(); i++)
      {
         m_force[i]->computeForce(link_val_curr, force);

         for (j = 0; j < 6; j++)
         {
            m_beta[j] += force[j];
         }
      }
   }

   for (j = 0; j < 6; j++)
   {
      m_beta[j] += m_external_force[j];
   }

// 3.6
   for (j = 0; j < 3; j++)
   {
      m_tau_star[j] = (m_beta[j] + m_joint_input[j] -
                       m_joint_friction*m_qd[j]);
   }

// 3.7
   m_gamma[0] = m_beta[0] - m_tau_star[0];
   m_gamma[1] = m_beta[1] - m_tau_star[1];
   m_gamma[2] = m_beta[2] - m_tau_star[2];

   for (j = 3; j < 6; j++)
   {
      m_gamma[j] = m_beta[j] -
         (m_n_minv[j][0]*m_tau_star[0] + m_N_refl[j][3]*m_zeta[3] +
          m_n_minv[j][1]*m_tau_star[1] + m_N_refl[j][4]*m_zeta[4] +
          m_n_minv[j][2]*m_tau_star[2] + m_N_refl[j][5]*m_zeta[5]);
   }

// 3.8 and 3.5
   stxToInboard(m_gamma, f_star_inboard);
   scongtxToInboardIrefl(m_N_refl, N_refl_inboard);
}


//----------------------------------------------------------------------------
//    Summary: third (final) forward recursion of the AB algorithm compute the
//             link's state derivative (velocity and acceleration)
// Parameters: a_inboard - spatial acceleration of inboard link
//    Returns: a_curr    - spatial accel of this link
//             qd - array whose 4 elements are the time derivative
//                  of joint position - quaternion rates
//             qdd - array whose 3 elements are the time derivative
//                  of joint velocity - angular acceleration wrt BCS
//----------------------------------------------------------------------------
void dmQuaternionLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              SpatialVector a_curr,
                                              Float qd[],
                                              Float qdd[])
{
   register int j;

// 5.1
   stxFromInboard(a_inboard, a_curr);
   for (j = 0; j < 6; j++)
   {
      a_curr[j] += m_zeta[j];
   }

// 5.2 (and transfer accels to 2nd half of derivative of state vector)
   for (j = 0; j < 3; j++)
   {
      qdd[j] = m_qdd[j] = -a_curr[j] +
         m_minv[j][0]*m_tau_star[0] - m_n_minv[3][j]*a_curr[3] +
         m_minv[j][1]*m_tau_star[1] - m_n_minv[4][j]*a_curr[4] +
         m_minv[j][2]*m_tau_star[2] - m_n_minv[5][j]*a_curr[5];
   }
   qdd[3] = 0.0;

// 5.3
   a_curr[0] += m_qdd[0];
   a_curr[1] += m_qdd[1];
   a_curr[2] += m_qdd[2];

// Compute quaternion rates.  Note that the quaternion rate equation
// requires the relative angular velocity to be expressed in i-1 coordinates.
   CartesianVector prev;
   rtxToInboard( &m_qd[0], prev);
   qd[0] = 0.5*( prev[0]*m_q[3]+ prev[1]*m_q[2] - prev[2]*m_q[1]);
   qd[1] = 0.5*(-prev[0]*m_q[2]+ prev[1]*m_q[3] + prev[2]*m_q[0]);
   qd[2] = 0.5*( prev[0]*m_q[1]- prev[1]*m_q[0] + prev[2]*m_q[3]);
   qd[3] =-0.5*( prev[0]*m_q[0]+ prev[1]*m_q[1] + prev[2]*m_q[2]);
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
//             qd - array whose 4 elements are the time derivative
//                  of joint position - quaternion rates
//             qdd - array whose 3 elements are the time derivative
//                  of joint velocity - angular acceleration wrt BCS
//----------------------------------------------------------------------------
void dmQuaternionLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              unsigned int *LB,
                                              unsigned int num_elements_LB,
                                              Float ***Xik,
                                              Float **constraint_forces,
                                              unsigned int *num_constraints,
                                              SpatialVector a_curr,
                                              Float qd[],
                                              Float qdd[])
{
   register unsigned int i, j, r;

// 5.1
   stxFromInboard(a_inboard, a_curr);
   for (j = 0; j < 6; j++)
   {
      a_curr[j] += m_zeta[j];
   }

   Float forces_of_constraint[3] = {0,0,0};
   for ( i = 0; i < num_elements_LB; i++)
   {
      unsigned int k = LB[i];

      for (r = 0; r < 3; r++)
         for (j = 0; j < num_constraints[k]; j++)
            forces_of_constraint[r] += Xik[k][r][j]*constraint_forces[k][j];
   }


// 5.2 (and transfer accels to 2nd half of derivative of state vector)
   for (j = 0; j < 3; j++)
   {
      qdd[j] = m_qdd[j] = -a_curr[j] +
         m_minv[j][0]*(m_tau_star[0] + forces_of_constraint[0]) +
         m_minv[j][1]*(m_tau_star[1] + forces_of_constraint[1]) +
         m_minv[j][2]*(m_tau_star[2] + forces_of_constraint[2]) -
         m_n_minv[3][j]*a_curr[3] -
         m_n_minv[4][j]*a_curr[4] -
         m_n_minv[5][j]*a_curr[5];
   }
   qdd[3] = 0.0;

// 5.3
   a_curr[0] += m_qdd[0];
   a_curr[1] += m_qdd[1];
   a_curr[2] += m_qdd[2];

// Compute quaternion rates.  Note that the quaternion rate equation
// requires the relative angular velocity to be expressed in i-1 coordinates.
   CartesianVector prev;
   rtxToInboard( &m_qd[0], prev);
   qd[0] = 0.5*( prev[0]*m_q[3]+ prev[1]*m_q[2] - prev[2]*m_q[1]);
   qd[1] = 0.5*(-prev[0]*m_q[2]+ prev[1]*m_q[3] + prev[2]*m_q[0]);
   qd[2] = 0.5*( prev[0]*m_q[1]- prev[1]*m_q[0] + prev[2]*m_q[3]);
   qd[3] =-0.5*( prev[0]*m_q[0]+ prev[1]*m_q[1] + prev[2]*m_q[2]);
}


//----------------------------------------------------------------------------
//    Summary: Spatial congruence transform of the 6x6 reflected AB inertia to
//             the inboard link's CS
// Parameters: N - 6x6 AB inertia reflected across this link's joint (input)
//    Returns: I - 6x6 reflected AB inertia transformed to inboard link's CS
//----------------------------------------------------------------------------
void dmQuaternionLink::scongtxToInboardIrefl(const SpatialTensor N,
                                             SpatialTensor I) const
{
// p.141 lab book #1.
   register int j, k;
   register Float tmp1, tmp2, tmp3;
   CartesianTensor Ta, Tb;

   for (k = 0; k < 3; k++) {
      for (j = k; j < 3; j++) {
         Ta[k][j] = Ta[j][k] = N[k + 3][j + 3];
      }
   }
   rcongtxToInboardSym(Ta, Tb);

// (2b) I22 = R'*N22*R;
   for (k = 0; k < 3; k++) {
      for (j = k; j < 3; j++) {
         I[k + 3][j + 3] = I[j + 3][k + 3] = Tb[k][j];
      }
   }

// (4)  I12 = (p x)*(R'*N22*R)
   I[0][3] = tmp1 = I[3][5]*m_p[1];
   I[1][4] = tmp2 = I[3][4]*m_p[2];
   I[2][5] = tmp3 = I[4][5]*m_p[0];
   I[3][0] = (I[0][3] -= tmp2);
   I[4][1] = (I[1][4] -= tmp3);
   I[5][2] = (I[2][5] -= tmp1);

   I[0][4] = I[4][0] = I[4][5]*m_p[1] - I[4][4]*m_p[2];
   I[0][5] = I[5][0] = I[5][5]*m_p[1] - I[4][5]*m_p[2];
   I[1][3] = I[3][1] = I[3][3]*m_p[2] - I[3][5]*m_p[0];
   I[1][5] = I[5][1] = I[3][5]*m_p[2] - I[5][5]*m_p[0];
   I[2][3] = I[3][2] = I[3][4]*m_p[0] - I[3][3]*m_p[1];
   I[2][4] = I[4][2] = I[4][4]*m_p[0] - I[3][4]*m_p[1];

// (5)  I11 = [(p x)*(R'*N22*R)]*(p x)'
   I[0][0] = I[0][5]*m_p[1] - I[0][4]*m_p[2];
   I[0][1] = I[1][0] = I[1][5]*m_p[1] - I[1][4]*m_p[2];
   I[0][2] = I[2][0] = I[2][5]*m_p[1] - I[2][4]*m_p[2];

   I[1][1] = I[1][3]*m_p[2] - I[1][5]*m_p[0];
   I[1][2] = I[2][1] = I[2][3]*m_p[2] - I[2][5]*m_p[0];

   I[2][2] = I[2][4]*m_p[0] - I[2][3]*m_p[1];
}


//----------------------------------------------------------------------------
//    Summary: The current link (i) is to be eliminated from the force-balance
//             equation of its predecessor.  This function computes the update
//             to the coefficient of loop k's constraint forces in the
//             predecessor link's force-balance equation as a result of the
//             elimination.
// Parameters: Xik_curr - coefficient of loop k's constraint forces in
//                        link i's force balance equation
//             columns_Xik - number of columns in Xik
//    Returns: Xik_prev - update to the coefficient of loop k's constraint
//                        forces in the predecessor's force-balance equation
//                        as a result of the elimination
//----------------------------------------------------------------------------
void dmQuaternionLink::XikToInboard(Float **Xik_curr,
                                    Float **Xik_prev,
                                    int columns_Xik) const
{
   register int i,j;

   Float tmp[3][6]; // Maximum dimensions of tmp (really tmp[3][columns_Xik]).

   for (i = 0; i < 3; i++)
      for (j = 0; j < columns_Xik; j++)
         tmp[i][j] = Xik_curr[3+i][j] - ( m_n_minv[3+i][0]*Xik_curr[0][j] +
                                          m_n_minv[3+i][1]*Xik_curr[1][j] +
                                          m_n_minv[3+i][2]*Xik_curr[2][j] );

   for (i = 0; i < 3; i++)
      for (j = 0; j < columns_Xik; j++)
         Xik_prev[i+3][j] = ( m_R[0][i]*tmp[0][j] +
                              m_R[1][i]*tmp[1][j] +
                              m_R[2][i]*tmp[2][j] );

   for (j = 0; j < columns_Xik; j++)
   {
      Xik_prev[0][j] = -m_p[2]*Xik_prev[4][j] + m_p[1]*Xik_prev[5][j];
      Xik_prev[1][j] =  m_p[2]*Xik_prev[3][j] - m_p[0]*Xik_prev[5][j];
      Xik_prev[2][j] = -m_p[1]*Xik_prev[3][j] + m_p[0]*Xik_prev[4][j];
   }
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
void dmQuaternionLink::BToInboard( Float **Bkn,
                                   Float **Xik, int cols_Xik,
                                   Float **Xin, int cols_Xin ) const
{
   Float tmp[3][6];
   int i, j;

   for (i = 0; i < 3; i++)
      for (j = 0; j < cols_Xin; j++)
         tmp[i][j] = ( m_minv[i][0]*Xin[0][j] +
                       m_minv[i][1]*Xin[1][j] +
                       m_minv[i][2]*Xin[2][j] );

   for (i = 0; i < cols_Xik; i++)
      for (j = 0; j < cols_Xin; j++)
         Bkn[i][j] += ( Xik[0][i]*tmp[0][j] +
                        Xik[1][i]*tmp[1][j] +
                        Xik[2][i]*tmp[2][j] );
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
void dmQuaternionLink::xformZetak(Float *zetak,
                                  Float **Xik, int cols_Xik) const
{
   register int i, j;

   Float tmp[3];
   tmp[0] = m_tau_star[0];
   tmp[1] = m_tau_star[1];
   tmp[2] = m_tau_star[2];

   for (i = 0; i < 3; i++)
      for (j = 0; j < 6; j++)
         tmp[i] -= m_I_star[i][j]*m_zeta[j];

   Float tmp1[3];

   tmp1[0] = m_minv[0][0]*tmp[0] + m_minv[0][1]*tmp[1] + m_minv[0][2]*tmp[2];
   tmp1[1] = m_minv[1][0]*tmp[0] + m_minv[1][1]*tmp[1] + m_minv[1][2]*tmp[2];
   tmp1[2] = m_minv[2][0]*tmp[0] + m_minv[2][1]*tmp[1] + m_minv[2][2]*tmp[2];

   SpatialVector tmpVec;
   tmpVec[0] = m_zeta[0] + tmp1[0];
   tmpVec[1] = m_zeta[1] + tmp1[1];
   tmpVec[2] = m_zeta[2] + tmp1[2];
   tmpVec[3] = m_zeta[3];
   tmpVec[4] = m_zeta[4];
   tmpVec[5] = m_zeta[5];

   for (i = 0; i < cols_Xik; i++)
      for (j = 0; j < 6; j++)
         zetak[i] -= Xik[j][i]*tmpVec[j];
}
