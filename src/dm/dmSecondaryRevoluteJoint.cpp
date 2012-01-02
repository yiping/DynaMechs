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
 *     File: dmSecondaryRevoluteJoint.cpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Class implementation for revolute secondary joints.
 *****************************************************************************/

#include "dmSecondaryRevoluteJoint.hpp"

//============================================================================
// class dmSecondaryRevoluteJoint : public dmSecondaryJoint
//============================================================================


//----------------------------------------------------------------------------
//    Summary: constructor for dmSecondaryRevoluteJoint class
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmSecondaryRevoluteJoint::dmSecondaryRevoluteJoint()
      : dmSecondaryJoint(),
        m_pos_spring(0.0),
        m_pos_damper(0.0),
        m_euler_spring(0.0),
        m_euler_damper(0.0),
        m_joint_input(0.0)
{
}


//----------------------------------------------------------------------------
//    Summary: Set parameters for implementing the constraints.  The meaning
//             and use of the parameters depends on the type of secondary
//             joint.  For compliant (soft) joints, the terms are the linear
//             and angular springs and dampers which implement the joint
//             constraints.  For hard joints with spring/damper stabiliation,
//             the terms are the springs and dampers which implement the
//             constraint stabilization.  For hard joints with Baumgarte
//             stabilization, the terms are interpreted as the Baumgarte
//             coefficients. 
// Parameters: linear_spring - coefficient of linear proportional terms
//             linear_damper - coefficient of linear derivative terms
//             angular_spring - coefficient of euler proportional terms
//             angular_damper - coefficient of euler derivative terms
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::setConstraintParams(Float linear_spring,
                                                   Float linear_damper,
                                                   Float angular_spring,
                                                   Float angular_damper)
{
   m_pos_spring = linear_spring;
   m_pos_damper = linear_damper;
   m_euler_spring = angular_spring;
   m_euler_damper = angular_damper;
}


//----------------------------------------------------------------------------
//    Summary: retrieve the free joint position and velocity
// Parameters: none
//    Returns: q - array of one element set to the current joint position
//             qd - array of one element set to the current joint velocity
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::getFreeState(Float q[], Float qd[]) const
{
   q[0] = m_euler[2];
   qd[0] = m_euler_dot[2];
}

//----------------------------------------------------------------------------
//    Summary: retrieve the state of the joint constraints
// Parameters: none
//    Returns: q - array of constraint states
//             qd - array of constraint state derivatives
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::getConstrainedState(Float q[], Float qd[]) const
{
   q[0] = m_euler[0];
   q[1] = m_euler[1];
   q[2] = m_d[0];
   q[3] = m_d[1];
   q[4] = m_d[2];

   qd[0] = m_euler_dot[0];
   qd[1] = m_euler_dot[1];
   qd[2] = m_d_dot[0];
   qd[3] = m_d_dot[1];
   qd[4] = m_d_dot[2];
}

//----------------------------------------------------------------------------
//    Summary: Compute the state (free and constrained) of the revolute
//             secondary joint.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::computeState()
{
#ifdef DEBUG
   cout << "dmSecondaryRevoluteJoint::computeState() enter\n" << flush;
#endif

   // Compute state information common to all secondary joints.
   dmSecondaryJoint::computeState();

   // Compute euler angle state information.

   m_euler[0] = atan2(-m_oa_R_k[1][2], m_oa_R_k[2][2]);
   m_euler[1] = atan2(m_oa_R_k[0][2], sqrt(m_oa_R_k[1][2]*m_oa_R_k[1][2] +
                                           m_oa_R_k[2][2]*m_oa_R_k[2][2]));
   m_euler[2] = atan2(-m_oa_R_k[0][1], m_oa_R_k[0][0]);

   m_c2 = cos(m_euler[2]);
   m_s2 = sin(m_euler[2]);
   m_c1 = cos(m_euler[1]);
   m_s1 = sin(m_euler[1]);
   m_t1 = tan(m_euler[1]);

   m_euler_dot[0] =  m_c2/m_c1*m_k_w_rel[0] - m_s2/m_c1*m_k_w_rel[1];
   m_euler_dot[1] =       m_s2*m_k_w_rel[0] +      m_c2*m_k_w_rel[1];
   m_euler_dot[2] = -m_c2*m_t1*m_k_w_rel[0] + m_s2*m_t1*m_k_w_rel[1] +
      m_k_w_rel[2];
}


//----------------------------------------------------------------------------
//    Summary: Computes and sets the eta_k1 and eta_k2 acceleration bias
//             terms which appear in the loop-closure constraint equation.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::computeEtas()
{
#ifdef DEBUG
   cout << "dmSecondaryRevoluteJoint::computeEtas() enter\n" << flush;
#endif

   register int i;

   // Compute Eta_k1[0:2]

   CartesianVector tmp1;
   tmp1[0] =  m_c1*m_c2*m_euler_dot[0];
   tmp1[1] = -m_c1*m_s2*m_euler_dot[0];
   tmp1[2] =       m_s1*m_euler_dot[0];

   crossproduct(m_k_w_oa, tmp1, &(m_Eta_k1[0]));

   CartesianVector tmp2;
   tmp2[0] = m_k_w_oa[0] + tmp1[0];
   tmp2[1] = m_k_w_oa[1] + tmp1[1];
   tmp2[2] = m_k_w_oa[2] + tmp1[2];

   CartesianVector tmp3;
   tmp3[0] = m_s2*m_euler_dot[1];
   tmp3[1] = m_c2*m_euler_dot[1];
   tmp3[2] = 0;

   CartesianVector tmp4;
   crossproduct(tmp2, tmp3, tmp4);
   m_Eta_k1[0] += tmp4[0];
   m_Eta_k1[1] += tmp4[1];
   m_Eta_k1[2] += tmp4[2];

   tmp2[0] += tmp3[0];
   tmp2[1] += tmp3[1];
   tmp2[2] += tmp3[2];

   // Eta_k1[0:2] += m_euler_dot[2] * (tmp2 x [0 0 1]')
   m_Eta_k1[0] +=  tmp2[1]*m_euler_dot[2];
   m_Eta_k1[1] += -tmp2[0]*m_euler_dot[2];
   m_Eta_k1[2] += 0;


   // Compute Eta_k1[3:5]

   crossproduct(m_oa_w_oa, m_d_dot, tmp1);
   crossproduct(m_oa_w_oa, m_d, tmp2);
   crossproduct(m_oa_w_oa, tmp2, tmp3);

   tmp3[0] += 2*tmp1[0];
   tmp3[1] += 2*tmp1[1];
   tmp3[2] += 2*tmp1[2];

   for (i = 0; i < 3; i++)
      m_Eta_k1[i+3] = (m_oa_R_k[0][i]*tmp3[0] +
                       m_oa_R_k[1][i]*tmp3[1] +
                       m_oa_R_k[2][i]*tmp3[2]);

   const dmABForKinStruct *kinStructA;
   kinStructA = m_articulation->getForKinStruct(m_link_A_index);
   crossproduct(kinStructA->v, m_a_p_oa, tmp1);
   crossproduct(kinStructA->v, tmp1, tmp2);

   for (i = 0; i < 3; i++)
      m_Eta_k1[i+3] += (m_a_R_k[0][i]*tmp2[0] +
                        m_a_R_k[1][i]*tmp2[1] +
                        m_a_R_k[2][i]*tmp2[2]);


   // Compute Eta_k2

   m_Eta_k2[0] = m_Eta_k2[1] = m_Eta_k2[2] = 0.0;

   const dmABForKinStruct *kinStructB;
   kinStructB = m_articulation->getForKinStruct(m_link_B_index);
   crossproduct(kinStructB->v, m_b_p_k, tmp1);
   crossproduct(kinStructB->v, tmp1, tmp2);

   for (i = 0; i < 3; i++)
      m_Eta_k2[i+3] = (m_b_R_k[0][i]*tmp2[0] +
                       m_b_R_k[1][i]*tmp2[1] +
                       m_b_R_k[2][i]*tmp2[2]);
}


//----------------------------------------------------------------------------
//    Summary: Computes the resultant force and moment applied
//             from link A to link B at the K coordinate system due
//             to the joint input and friction.
// Parameters: none  
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::computeAppliedForce()
{
   m_k_f_k[0] = -m_t1*m_c2*(m_joint_input - m_joint_friction*m_euler_dot[2]);
   m_k_f_k[1] =  m_t1*m_s2*(m_joint_input - m_joint_friction*m_euler_dot[2]);
   m_k_f_k[2] =  (m_joint_input - m_joint_friction*m_euler_dot[2]);

   m_k_f_k[3] = m_k_f_k[4] = m_k_f_k[5] = 0.0;
}


//----------------------------------------------------------------------------
//    Summary: Adds the penalty force for a compliant (soft) secondary joint
//             to the applied forces at the joint.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::applyPenaltyForce()
{
#ifdef DEBUG
   cerr << "dmSecondaryRevoluteJoint::applyPenaltyForce()" << flush << endl;
#endif

   register int i;

   CartesianVector gimbal_trq;
   CartesianVector slider_frc;

   // Compute the torques about the gimbal axes.
   gimbal_trq[0] = -m_euler[0]*m_euler_spring - m_euler_dot[0]*m_euler_damper;
   gimbal_trq[1] = -m_euler[1]*m_euler_spring - m_euler_dot[1]*m_euler_damper;

   // Compute the forces (in link A outboard coordinates).
   slider_frc[0] = -m_d[0]*m_pos_spring - m_d_dot[0]*m_pos_damper;
   slider_frc[1] = -m_d[1]*m_pos_spring - m_d_dot[1]*m_pos_damper;
   slider_frc[2] = -m_d[2]*m_pos_spring - m_d_dot[2]*m_pos_damper;

   // Convert the gimbal torques into moments in K coordinates.
   m_k_f_k[0] +=  m_c2/m_c1*gimbal_trq[0] + m_s2*gimbal_trq[1];
   m_k_f_k[1] += -m_s2/m_c1*gimbal_trq[0] + m_c2*gimbal_trq[1];
   m_k_f_k[2] +=  0.0;

   // Convert the forces into K coordinates.
   for (i = 0; i < 3; i++)
      m_k_f_k[i+3] += (m_oa_R_k[0][i]*slider_frc[0] +
                       m_oa_R_k[1][i]*slider_frc[1] +
                       m_oa_R_k[2][i]*slider_frc[2]);
}


//----------------------------------------------------------------------------
//    Summary: Returns the forces/torques of the constraint stabilization
//             springs and dampers (for hard constraints with spring and
//             damper constraint stabilization).
// Parameters: none
//    Returns: force - constraint stabilization forces and torques
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::computeStabilizationForce(Float force[])
{
   // Gimbal torques.
   force[0] = -m_euler[0]*m_euler_spring - m_euler_dot[0]*m_euler_damper;
   force[1] = -m_euler[1]*m_euler_spring - m_euler_dot[1]*m_euler_damper;

   // Position forces.
   force[2] = -m_d[0]*m_pos_spring - m_d_dot[0]*m_pos_damper;
   force[3] = -m_d[1]*m_pos_spring - m_d_dot[1]*m_pos_damper;
   force[4] = -m_d[2]*m_pos_spring - m_d_dot[2]*m_pos_damper;
}


//----------------------------------------------------------------------------
//    Summary: Called prior to the AB backward dynamics routine to 
//             initialize the X_ik's for the secondary joint's
//             A link, B link, and the root link of the loop. 
// Parameters: link_index - index, i, of the link to compute Xik for
//             root_index - link index of the root of the loop closed by
//                          this secondary joint
//    Returns: Xik - coefficient of loop k's constraint forces in link
//                   i's force balance equation.  The transpose also
//                   appears loop k's constraint equations.
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::initXik(Float **Xik,
                                       int link_index,
                                       int root_index) const
{
   int i,j,r,c;

   if (link_index == m_link_A_index)  // Xik = X_p1(k)k
   {
      // Compute Xik(1:3,1:2)
      Float c2 = cos(m_euler[2]);
      Float s2 = sin(m_euler[2]);
      Float c1 = cos(m_euler[1]);
      for (i = 0; i < 3; i++)
      {
         Xik[i][0] = -(m_a_R_k[i][0]*c2/c1 - m_a_R_k[i][1]*s2/c1);
         Xik[i][1] = -(m_a_R_k[i][0]*s2    + m_a_R_k[i][1]*c2);
      }

      // Compute Xik(4:6,1:2)
      Xik[3][0] = Xik[3][1] = 0.0;
      Xik[4][0] = Xik[4][1] = 0.0;
      Xik[5][0] = Xik[5][1] = 0.0;

      // Compute Xik(4:6,3:5)
      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            Xik[3+i][2+j] = -m_a_R_oa[i][j];

      // Compute Xik(1:3,3:5)
      Xik[0][2] = -m_a_p_k[2]*Xik[4][2] + m_a_p_k[1]*Xik[5][2];
      Xik[0][3] = -m_a_p_k[2]*Xik[4][3] + m_a_p_k[1]*Xik[5][3];
      Xik[0][4] = -m_a_p_k[2]*Xik[4][4] + m_a_p_k[1]*Xik[5][4];

      Xik[1][2] =  m_a_p_k[2]*Xik[3][2] - m_a_p_k[0]*Xik[5][2];
      Xik[1][3] =  m_a_p_k[2]*Xik[3][3] - m_a_p_k[0]*Xik[5][3];
      Xik[1][4] =  m_a_p_k[2]*Xik[3][4] - m_a_p_k[0]*Xik[5][4];

      Xik[2][2] = -m_a_p_k[1]*Xik[3][2] + m_a_p_k[0]*Xik[4][2];
      Xik[2][3] = -m_a_p_k[1]*Xik[3][3] + m_a_p_k[0]*Xik[4][3];
      Xik[2][4] = -m_a_p_k[1]*Xik[3][4] + m_a_p_k[0]*Xik[4][4];
   }
   else if (link_index == m_link_B_index)  // Xik = X_p2(k)k
   {
      // Compute Xik(1:3,1:2)
      Float c2 = cos(m_euler[2]);
      Float s2 = sin(m_euler[2]);
      Float c1 = cos(m_euler[1]);
      for (i = 0; i < 3; i++)
      {
         Xik[i][0] = m_b_R_k[i][0]*c2/c1 - m_b_R_k[i][1]*s2/c1;
         Xik[i][1] = m_b_R_k[i][0]*s2    + m_b_R_k[i][1]*c2;
      }

      // Compute Xik(4:6,1:2)
      Xik[3][0] = Xik[3][1] = 0.0;
      Xik[4][0] = Xik[4][1] = 0.0;
      Xik[5][0] = Xik[5][1] = 0.0;

      // Compute Xik(4:6,3:5)
      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            Xik[3+i][2+j] =  (m_b_R_k[i][0]*m_oa_R_k[j][0] +
                              m_b_R_k[i][1]*m_oa_R_k[j][1] +
                              m_b_R_k[i][2]*m_oa_R_k[j][2]);

      // Compute Xik(1:3,3:5)
      Xik[0][2] = -m_b_p_k[2]*Xik[4][2] + m_b_p_k[1]*Xik[5][2];
      Xik[0][3] = -m_b_p_k[2]*Xik[4][3] + m_b_p_k[1]*Xik[5][3];
      Xik[0][4] = -m_b_p_k[2]*Xik[4][4] + m_b_p_k[1]*Xik[5][4];

      Xik[1][2] =  m_b_p_k[2]*Xik[3][2] - m_b_p_k[0]*Xik[5][2];
      Xik[1][3] =  m_b_p_k[2]*Xik[3][3] - m_b_p_k[0]*Xik[5][3];
      Xik[1][4] =  m_b_p_k[2]*Xik[3][4] - m_b_p_k[0]*Xik[5][4];

      Xik[2][2] = -m_b_p_k[1]*Xik[3][2] + m_b_p_k[0]*Xik[4][2];
      Xik[2][3] = -m_b_p_k[1]*Xik[3][3] + m_b_p_k[0]*Xik[4][3];
      Xik[2][4] = -m_b_p_k[1]*Xik[3][4] + m_b_p_k[0]*Xik[4][4];
   }
   else if  (link_index == root_index)  // Clear Xik for accumulation.
   {
      for (r = 0; r < 6; r++)
         for (c = 0; c < 5; c++)
            Xik[r][c] = 0.0;
   }
}


//----------------------------------------------------------------------------
//    Summary: Compute the acceleration bias term in the loop-closure
//             constraint equation.
// Parameters: none
//    Returns: zeta - the acceleration bias term  
//----------------------------------------------------------------------------
void dmSecondaryRevoluteJoint::getZeta(Float *zeta) const
{
   SpatialVector eta;
   eta[0] = m_Eta_k1[0] - m_Eta_k2[0];
   eta[1] = m_Eta_k1[1] - m_Eta_k2[1];
   eta[2] = m_Eta_k1[2] - m_Eta_k2[2];
   eta[3] = m_Eta_k1[3] - m_Eta_k2[3];
   eta[4] = m_Eta_k1[4] - m_Eta_k2[4];
   eta[5] = m_Eta_k1[5] - m_Eta_k2[5];


   zeta[0] = m_c2/m_c1*eta[0] - m_s2/m_c1*eta[1];
   zeta[1] =      m_s2*eta[0] +      m_c2*eta[1];

   for (int i = 0; i < 3; i++)
      zeta[2+i] = (m_oa_R_k[i][0]*eta[3] +
                   m_oa_R_k[i][1]*eta[4] +
                   m_oa_R_k[i][2]*eta[5]);

   if (m_stabilization == BAUMGARTE)
   {
      // Add Baumgarte stabilization terms to zeta.
      zeta[0] += -m_euler[0]*m_euler_spring - m_euler_dot[0]*m_euler_damper;
      zeta[1] += -m_euler[1]*m_euler_spring - m_euler_dot[1]*m_euler_damper;
      zeta[2] += -m_d[0]*m_pos_spring - m_d_dot[0]*m_pos_damper;
      zeta[3] += -m_d[1]*m_pos_spring - m_d_dot[1]*m_pos_damper;
      zeta[4] += -m_d[2]*m_pos_spring - m_d_dot[2]*m_pos_damper;
   }
}
