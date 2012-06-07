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
 *     File: dmSphericalLink.cpp
 *   Author: Scott McMillan
 *  Summary: Class implementation for links with ball-joints
 *****************************************************************************/

#include "dm.h"
#include "dmRigidBody.hpp"
#include "dmLink.hpp"
#include "dmSphericalLink.hpp"

//============================================================================
// class dmSphericalLink: public dmRigidBody, public dmLink
//============================================================================

//----------------------------------------------------------------------------
//    Summary: class constructor
// Parameters: cfg_ptr - pointer to file containing required parameters
//    Returns: none
//----------------------------------------------------------------------------
dmSphericalLink::dmSphericalLink() : dmRigidBody()
{
#ifdef DEBUG
   cout << "dmSphericalLink constructor: enter \n";
#endif

   for (int i=0; i<NUM_DOFS; i++)
   {
      m_qd[i] = 0.0;
      m_qdd[i] = 0.0;
      m_joint_limit[i] = 0.0;
      m_tau_limit[i] = 0.0;
      m_joint_input[i] = 0.0;
   }

   EulerAngles tem = {0., 0., 0.};
   setJointPos(tem);

#ifdef DEBUG
   cout << "dmSphericalLink constructor: exit \n" << flush;
#endif
}

//----------------------------------------------------------------------------
void dmSphericalLink::setJointLimits(Float axis_limits[NUM_DOFS],
                                     Float spring, Float damper)
{
   for (int i=0; i<NUM_DOFS; i++)
   {
      m_joint_limit[i] = axis_limits[i];
   }

   m_joint_limit_spring = spring;
   m_joint_limit_damper = damper;

   // hack to get the joint limits to take immediate effect
   setJointPos(m_q);
}

//----------------------------------------------------------------------------
void dmSphericalLink::initABVars()
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
inline void dmSphericalLink::matrixInverse3PD(CartesianTensor A,
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
//    Summary: Set the euler angles, compute a limit torque if necessary.
// Parameters: q - phi,theta,psi euler angles in radians
//    Returns: none
//----------------------------------------------------------------------------
inline void dmSphericalLink::setJointPos(EulerAngles q)
{
   register int i;
   register Float tem, angle;
   CartesianVector axis;

   m_joint_limit_flag = false;
   for (i = 0; i < 3; i++) {
      if (q[i] > (2.0*M_PI))
         q[i] -= (Float)(2.0*M_PI);
      else if (q[i] < -(2.0*M_PI))
         q[i] += (Float)(2.0*M_PI);

      m_tau_limit[i] = 0.0;
      m_q[i] = q[i];
   }

   m_sphi = sin(q[0]);
   m_cphi = cos(q[0]);
   m_stheta = sin(q[1]);
   m_ctheta = cos(q[1]);
   m_spsi = sin(q[2]);
   m_cpsi = cos(q[2]);

   if (fabs(m_ctheta) < 0.000001) {
      cerr << "Warning: Near euler angle singularity, cos(theta) = "
           << m_ctheta << endl;

      m_ctheta = (Float)0.000001;
   }

// compute limit torques if necessary:
   if (m_joint_limit[0] > 0.0)    // x-axis limit
   {
      angle = acos(m_ctheta*m_cpsi);
      if (angle > m_joint_limit[0])
      {
         m_joint_limit_flag = true;

         // joint spring moment axis:
         axis[1] = m_sphi*m_spsi + m_cphi*m_stheta*m_cpsi;
         axis[2] = m_cphi*m_spsi - m_sphi*m_stheta*m_cpsi;
         tem = sqrt(axis[1]*axis[1] + axis[2]*axis[2]);
         m_tau_limit[1] -= m_joint_limit_spring*(angle - m_joint_limit[0]) *
                              axis[1]/tem;
         m_tau_limit[2] -= m_joint_limit_spring*(angle - m_joint_limit[0]) *
                              axis[2]/tem;

         // joint damping moment axis:
         m_tau_limit[1] -= m_joint_limit_damper*m_qd[1];
         m_tau_limit[2] -= m_joint_limit_damper*m_qd[2];
      }
   }
   if (m_joint_limit[1] > 0.0)    // y-axis limit
   {
      angle = acos(m_cphi*m_cpsi + m_sphi*m_stheta*m_spsi);
      if (angle > m_joint_limit[1])
      {
         m_joint_limit_flag = true;

         // joint spring moment axis:
         axis[0] = m_sphi*m_cpsi - m_cphi*m_stheta*m_spsi;
         axis[2] = m_ctheta*m_spsi;
         tem = sqrt(axis[1]*axis[1] + axis[2]*axis[2]);
         m_tau_limit[0] -= m_joint_limit_spring*(angle - m_joint_limit[1])*
                         axis[0]/tem;
         m_tau_limit[2] -= m_joint_limit_spring*(angle - m_joint_limit[1])*
                         axis[2]/tem;

         // joint _damper moment axis:
         m_tau_limit[0] -= m_joint_limit_damper*m_qd[0];
         m_tau_limit[2] -= m_joint_limit_damper*m_qd[2];
      }
   }
   if (m_joint_limit[2] > 0.0)    // z-axis limit
   {
      angle = acos(m_cphi*m_ctheta);
      if (angle > m_joint_limit[2])
      {
         m_joint_limit_flag = true;

         // joint spring moment axis:
         axis[0] = m_sphi*m_ctheta;
         axis[1] = m_stheta;
         tem = sqrt(axis[1]*axis[1] + axis[2]*axis[2]);
         m_tau_limit[0] -= m_joint_limit_spring*(angle - m_joint_limit[2]) *
            axis[0]/tem;
         m_tau_limit[1] -= m_joint_limit_spring*(angle - m_joint_limit[2]) *
            axis[1]/tem;

         // joint damping moment axis:
         m_tau_limit[0] -= m_joint_limit_damper*m_qd[0];
         m_tau_limit[1] -= m_joint_limit_damper*m_qd[1];
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: set the state (position and velocity) variables and reset any
//             contacts in case there is any built up spring forces
// Parameters:  q - array of getNumDOFS (3) elements defining joint position
//             qd - array of      "      "  elements defining joint velocity
//    Returns: none
//----------------------------------------------------------------------------
void dmSphericalLink::setState(Float q[], Float qd[])
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
//    Summary: retreive the current state of the link (joint position and
//             velocity variables)
// Parameters:
//    Returns: q  - array of getNumDOFs (3) joint position variables
//             qd - array of     "       "  joint velocity variables
//----------------------------------------------------------------------------
void dmSphericalLink::getState(Float q[], Float qd[]) const
{
   for (int i=0; i<NUM_DOFS; i++)
   {
      qd[i] = m_qd[i];
      q[i] =  m_q[i];
   }
}

//----------------------------------------------------------------------------
void dmSphericalLink::getPose(RotationMatrix R, CartesianVector p) const
{
   // columns for rtxFromInboard
   R[0][0] = m_ctheta*m_cpsi;
   R[1][0] = m_sphi*(m_stheta*m_cpsi) - m_cphi*m_spsi;
   R[2][0] = m_cphi*(m_stheta*m_cpsi) + m_sphi*m_spsi;

   R[0][1] = m_ctheta*m_spsi;
   R[1][1] = m_sphi*(m_stheta*m_spsi) + m_cphi*m_cpsi;
   R[2][1] = m_cphi*(m_stheta*m_spsi) - m_sphi*m_cpsi;

   R[0][2] =-m_stheta;
   R[1][2] = m_sphi*m_ctheta;
   R[2][2] = m_cphi*m_ctheta;

   p[0] = m_p[0];
   p[1] = m_p[1];
   p[2] = m_p[2];
}

//----------------------------------------------------------------------------
//    Summary: rotate vector wrt inboard CS to this link's CS
// Parameters: prev - 3-vector quantity expressed wrt inboard CS
//    Returns: curr - 3-vector result express wrt this CS
//----------------------------------------------------------------------------
inline void dmSphericalLink::rtxFromInboard(const CartesianVector prev,
                                            CartesianVector curr) const
{
// From page 138 lab book #1.
// rotate psi about z axis.
   Float tem0 = m_cpsi*prev[0] + m_spsi*prev[1];
   Float tem1 = m_cpsi*prev[1] - m_spsi*prev[0];

// rotate theta about y axis.
   curr[0] = m_ctheta*tem0 - m_stheta*prev[2];
   Float tem2 = m_ctheta*prev[2] + m_stheta*tem0;

// rotate phi about x-axis.
   curr[1] = m_cphi*tem1 + m_sphi*tem2;
   curr[2] = m_cphi*tem2 - m_sphi*tem1;
}

//----------------------------------------------------------------------------
//    Summary: Rotation of 3d vector from this CS to inboard link
// Parameters: curr - 3d vector wrt this link's CS
//    Returns: prev - 3d vector wrt inboard link's CS
//----------------------------------------------------------------------------
inline void dmSphericalLink::rtxToInboard(const CartesianVector curr,
                                          CartesianVector prev) const
{
// From page 139 of lab book #1.
   // rotate -phi = -pose[0] about the x-axis.
   Float tem1 = m_cphi*curr[1] - m_sphi*curr[2];
   Float tem2 = m_cphi*curr[2] + m_sphi*curr[1];

   // rotate -theta = -pose[1] about the y-axis.
   Float tem0 = m_ctheta*curr[0] + m_stheta*tem2;
   prev[2] = m_ctheta*tem2 - m_stheta*curr[0];

   // rotate -psi = -pose[2] about the z-axis.
   prev[0] = m_cpsi*tem0 - m_spsi*tem1;
   prev[1] = m_cpsi*tem1 + m_spsi*tem0;
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence tranform from this link CS to inboard
//             link's CS of a 3x3 symmetric matrix
// Parameters: Curr - 3x3 symmetric matrix in this CS.
//    Returns: Prev - 3x3 symmetric matrix in inboard CS.
//----------------------------------------------------------------------------
inline void dmSphericalLink::rcongtxToInboardSym(const CartesianTensor Curr,
                                                 CartesianTensor Prev) const
{
// pp. 142ff in lab book 1.
   CartesianTensor Ta, Tb;

// x-axis congruence (phi):
   Float ss = m_sphi*m_sphi;
   Float sc = m_sphi*m_cphi;

   Float tem1 = Curr[2][2] - Curr[1][1];
   Float tem2 = tem1*ss - Curr[1][2]*(sc + sc);

   Ta[2][2] = Curr[2][2] - tem2;
   Ta[1][1] = Curr[1][1] + tem2;
   Ta[1][2] = Curr[1][2]*(1.0 - ss - ss) - tem1*sc;
   Ta[0][1] = Curr[0][1]*m_cphi - Curr[0][2]*m_sphi;
   Ta[0][2] = Curr[0][1]*m_sphi + Curr[0][2]*m_cphi;

// y-axis congruence (theta):
   ss = m_stheta*m_stheta;
   sc = m_stheta*m_ctheta;

   tem1 = Curr[0][0] - Ta[2][2];
   tem2 = tem1*ss - Ta[0][2]*(sc + sc);

   Tb[0][0] = Curr[0][0] - tem2;
   Prev[2][2] = Ta[2][2] + tem2;
   Tb[0][2] = Ta[0][2]*(1.0 - ss - ss) - tem1*sc;
   Tb[0][1] = Ta[0][1]*m_ctheta + Ta[1][2]*m_stheta;
   Tb[1][2] = Ta[1][2]*m_ctheta - Ta[0][1]*m_stheta;

// z-axis congruence (psi):
   ss = m_spsi*m_spsi;
   sc = m_spsi*m_cpsi;

   tem1 = Ta[1][1] - Tb[0][0];
   tem2 = tem1*ss - Tb[0][1]*(sc + sc);

   Prev[1][1] = Ta[1][1] - tem2;
   Prev[0][0] = Tb[0][0] + tem2;
   Prev[1][0] = Prev[0][1] = Tb[0][1]*(1.0 - ss - ss) - tem1*sc;
   Prev[2][0] = Prev[0][2] = Tb[0][2]*m_cpsi - Tb[1][2]*m_spsi;
   Prev[2][1] = Prev[1][2] = Tb[0][2]*m_spsi + Tb[1][2]*m_cpsi;
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence transform of general 3x3 to inboard
//             link's CS.
// Parameters: Curr - general 3x3 matrix in this link's CS.
//    Returns: Prev - general 3x3 matrix tx'ed to inboard link's CS.
//----------------------------------------------------------------------------
inline void dmSphericalLink::rcongtxToInboardGen(const CartesianTensor,
                                                 CartesianTensor) const
{
// not yet implemented.
   exit(5);
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation from inboard link's CS to this CS
// Parameters: prev - 6d vector expressed in inboard link's CS
//    Returns: curr - 6d vector in this link's CS
//----------------------------------------------------------------------------
inline void dmSphericalLink::stxFromInboard(const SpatialVector prev,
                                            SpatialVector curr) const
{
// From page 140 lab book 1.
   CartesianVector tem;

   crossproduct(&prev[0], m_p, tem);

   tem[0] += prev[3];
   tem[1] += prev[4];
   tem[2] += prev[5];

   rtxFromInboard(&prev[0], &curr[0]);
   rtxFromInboard(tem, &curr[3]);
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation to inboard links CS to this CS.
// Parameters: curr - 6d vector expressed in this CS
//    Returns: prev - 6d vector expressed in previous link's CS.
//----------------------------------------------------------------------------
inline void dmSphericalLink::stxToInboard(const SpatialVector curr,
                                          SpatialVector prev) const
{
// From page 140 lab book 1.
   CartesianVector tem;

   rtxToInboard(&curr[3], &prev[3]);
   rtxToInboard(&curr[0], &prev[0]);
   crossproduct(m_p, &prev[3], tem);

   prev[0] += tem[0];
   prev[1] += tem[1];
   prev[2] += tem[2];
}

//----------------------------------------------------------------------------
//    Summary: Spatial congruence transform of the 6x6 reflected AB inertia to
//             the inboard link's CS
// Parameters: N - 6x6 AB inertia reflected across this link's joint (input)
//    Returns: I - 6x6 reflected AB inertia transformed to inboard link's CS
//----------------------------------------------------------------------------
inline void dmSphericalLink::scongtxToInboardIrefl(const SpatialTensor N,
                                                   SpatialTensor I) const
{
// p.141 lab book #1.
   register int j, k;
   register Float tmp1, tmp2, tmp3;
   CartesianTensor Ta, Tb;

   for (k = 0; k < 3; k++) {
      for (j = k; j < 3; j++) {
         Ta[k][j] = N[k + 3][j + 3];
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
//    Summary: first forward kinematics recursion of the AB simulation alg.
// Parameters: q - current joint position computed by num. integration alg.
//             qd - current joint velocity computed by the num. int. alg.
//             link_val_inboard - struct containing kinematic (state dependent)
//                    quantities for the inboard (parent) link/body
//    Returns: link_val_curr - struct filled with kinematic quantities for this
//                    link.
//----------------------------------------------------------------------------
void dmSphericalLink::ABForwardKinematics(
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

   //crossproduct(&link_val_inboard.v[0], p, &m_zeta[3]);
   //crossproduct(&link_val_inboard.v[0], &m_zeta[3], &m_zeta[0]);
   //rtxFromInboard(&m_zeta[0], &m_zeta[3]);

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
void dmSphericalLink::ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
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
      m_tau_star[j] = m_beta_star[j] + m_joint_input[j] -
                    m_joint_friction*m_qd[j] +
                    (m_joint_limit_flag ? m_tau_limit[j] : 0.0);
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
void dmSphericalLink::ABBackwardDynamicsN(
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
      m_tau_star[j] = m_beta[j] + m_joint_input[j] -
                    m_joint_friction*m_qd[j] +
                    (m_joint_limit_flag ? m_tau_limit[j] : 0.0);
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
//             qd - array whose three elements are the time derivative
//                  of joint position - euler angle rates
//             qdd - array whose three elements are the time derivative
//                  of joint velocity - angular acceleration wrt BCS
//----------------------------------------------------------------------------
void dmSphericalLink::ABForwardAccelerations(SpatialVector a_inboard,
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

// 5.3
   a_curr[0] += m_qdd[0];
   a_curr[1] += m_qdd[1];
   a_curr[2] += m_qdd[2];

// Compute euler angle rates and transfer to the first half of the derivative
// of state vector:

   // convert qd = omega_r to Euler angle rates.
   Float tan_theta = m_stheta/m_ctheta;
   // phid =
   qd[0] = m_qd[0] + m_sphi*tan_theta*m_qd[1] + m_cphi*tan_theta*m_qd[2];
   // thetad =
   qd[1] = m_cphi*m_qd[1] - m_sphi*m_qd[2];
   // psid =
   qd[2] = m_sphi*m_qd[1]/m_ctheta + m_cphi*m_qd[2]/m_ctheta;
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
//             qd - array whose three elements are the time derivative
//                  of joint position - euler angle rates
//             qdd - array whose three elements are the time derivative
//                  of joint velocity - angular acceleration wrt BCS
//----------------------------------------------------------------------------
void dmSphericalLink::ABForwardAccelerations(SpatialVector a_inboard,
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

   Float forces_of_constraint[NUM_DOFS] = {0,0,0};
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

// 5.3
   a_curr[0] += m_qdd[0];
   a_curr[1] += m_qdd[1];
   a_curr[2] += m_qdd[2];

// Compute euler angle rates and transfer to the first half of the derivative
// of state vector:

   // convert qd = omega_r to Euler angle rates.
   Float tan_theta = m_stheta/m_ctheta;
   // phid =
   qd[0] = m_qd[0] +
      m_sphi*tan_theta*m_qd[1] +
      m_cphi*tan_theta*m_qd[2];
   // thetad =
   qd[1] = m_cphi*m_qd[1] -
      m_sphi*m_qd[2];
   // psid =
   qd[2] = m_sphi*m_qd[1]/m_ctheta +
      m_cphi*m_qd[2]/m_ctheta;
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
void dmSphericalLink::XikToInboard(Float **Xik_curr,
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

   Float pi_R_i[3][3];

   pi_R_i[0][0] = m_cpsi*m_ctheta;
   pi_R_i[0][1] = m_cpsi*m_stheta*m_sphi - m_spsi*m_cphi;
   pi_R_i[0][2] = m_cpsi*m_stheta*m_cphi + m_spsi*m_sphi;

   pi_R_i[1][0] = m_spsi*m_ctheta;
   pi_R_i[1][1] = m_spsi*m_stheta*m_sphi + m_cpsi*m_cphi;
   pi_R_i[1][2] = m_spsi*m_stheta*m_cphi - m_cpsi*m_sphi;

   pi_R_i[2][0] = -m_stheta;
   pi_R_i[2][1] = m_ctheta*m_sphi;
   pi_R_i[2][2] = m_ctheta*m_cphi;


   for (i = 0; i < 3; i++)
      for (j = 0; j < columns_Xik; j++)
         Xik_prev[i+3][j] = ( pi_R_i[i][0]*tmp[0][j] +
                              pi_R_i[i][1]*tmp[1][j] +
                              pi_R_i[i][2]*tmp[2][j] );

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
void dmSphericalLink::BToInboard(Float **Bkn,
                                 Float **Xik, int cols_Xik,
                                 Float **Xin, int cols_Xin) const
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
void dmSphericalLink::xformZetak(Float *zetak,
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


//-------------------------------------------------------------------
/* The following codes are mostly written in Pure Eigen style. 
 Consider to gradually optimize the following code in the future 
 by switching to those more efficient, DynaMechs-native functions, such as stxToInboard, stxFromInboard etc. */
//! DM v5.0 function, 
void dmSphericalLink::RNEAOutwardFKID(  dmRNEAStruct &link_val2_curr, 
								dmRNEAStruct &link_val2_inboard,
								bool ExtForceFlag)
{
	// compute the position and orientation of the link in the inertial coordinate system (ICS)
	// This part reuses the code snippet in dmLink::forwardKinematics() 
	// R_ICS and p_ICS are only used to compute contact forces
	// dmContactModel::computeForce() requires CartesianVector and RotationMatrix as inputs. 
	for (int i = 0; i < 3; i++)
	{
		link_val2_curr.p_ICS[i] = link_val2_inboard.p_ICS[i];
		for (int j = 0; j < 3; j++)
		{
	 		link_val2_curr.p_ICS[i] += link_val2_inboard.R_ICS[i][j] * m_p[j]; // position
		}
		rtxFromInboard(&(link_val2_inboard.R_ICS[i][0]),
					   &(link_val2_curr.R_ICS[i][0])); //orientation
	}
	
	Vector3F q, qd;
	getState(q.data(),qd.data());
	Matrix6XF Phi;
	jcalc(Phi);
	Vector6F vJ = Phi * qd;
	
	// v_i = v_pi + Phi qdot
	stxFromInboard(link_val2_inboard.v.data(), link_val2_curr.v.data());
	link_val2_curr.v += vJ;			   
	
	// a_i = a_pi + phi qddot + v x phi qdot
	stxFromInboard(link_val2_inboard.a.data(), link_val2_curr.a.data());
	link_val2_curr.a += Phi * link_val2_curr.qdd + crm( link_val2_curr.v ) * vJ;
	
	
	Matrix6F I = getSpatialInertiaMatrix();
	link_val2_curr.f = I *  link_val2_curr.a  + crf(link_val2_curr.v) * I * link_val2_curr.v;
	
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


//-------------------------------------------------------------------
//! DM v5.0 function, 
void dmSphericalLink::RNEAOutwardFKIDFirst(  dmRNEAStruct &link_val2_curr,
									 CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
									 RotationMatrix  R_ref_ICS,  
									 const Vector6F& a_ini, 
									 const Vector6F& v_ini,
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
	
	Vector3F q, qd;
	getState(q.data(),qd.data());
	
	Matrix6XF Phi;
	jcalc(Phi);
	
	Vector6F vJ = Phi* qd;
	
	//Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = vJ;
	
	//a_i = a_pi + phi qdd (v cross phi qdot is zero)
	stxFromInboard(a_ini.data(), link_val2_curr.a.data());
	link_val2_curr.a += Phi * link_val2_curr.qdd;
	
	
	Matrix6F I = getSpatialInertiaMatrix();
	link_val2_curr.f = I *  link_val2_curr.a  + crf(link_val2_curr.v) * I * link_val2_curr.v;	
	
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

//---------------------------------------------------
void dmSphericalLink::jcalc(Matrix6XF& S) {
	S.resize(6,3);
    S << 1,0,0, 0,1,0, 0,0,1, 0,0,0, 0,0,0, 0,0,0;
}

