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
 *     File: dmMDHLink.cpp
 *   Author: Scott McMillan
 *  Summary: Class hierarchy implementation for 1dof robotic links.
 *****************************************************************************/

#include "dm.h"
#include "dmLink.hpp"
#include "dmRigidBody.hpp"
#include "dmMDHLink.hpp"
#include "dmActuator.hpp"
#include "dmRevDCMotor.hpp"

//=============================================================================
// class dmMDHLink: public dmRigidBody, public dmLink
//=============================================================================

//----------------------------------------------------------------------------
//    Summary: constructor for dmMDHLink class
// Parameters: cfg_ptr - ifstream reference pointing to an opened configuration
//                       file pointing to the require parameters.
//    Returns:
//----------------------------------------------------------------------------
dmMDHLink::dmMDHLink()
      : dmRigidBody(),
        m_actuator(NULL),
        m_qd(0.0),
        m_qdd(0.0),
        m_min_joint_pos(0.0),
        m_max_joint_pos(0.0),
        m_joint_input(0.0),
        m_tau_limit(0.0)
{
   setMDHParameters(0., 0., 0., 0.);

   // m_joint_axis_index, and m_joint_limit_flag are initialized in
   //   dmRevoluteLink or dmPrismaticLink.
   // AB vars are initialized each time inertial parameters are set in
   //   dmRigidBody.
}

//----------------------------------------------------------------------------
dmMDHLink::~dmMDHLink()
{
}

//----------------------------------------------------------------------------
void dmMDHLink::getMDHParameters(Float *a, Float *alpha,
                                 Float *d, Float *theta) const
{
   *a = m_aMDH;
   *alpha = m_alphaMDH;
   *d = m_dMDH;
   *theta = m_thetaMDH;
}

//----------------------------------------------------------------------------
void dmMDHLink::setMDHParameters(Float a, Float alpha, Float d, Float theta)
{
   m_aMDH = a;
   m_alphaMDH = alpha;
   m_dMDH = d;
   m_thetaMDH = theta;

   // ------------- x-screw ------------------

   m_salpha = sin(m_alphaMDH);
   m_calpha = cos(m_alphaMDH);

   // take care of rounding errors
   if (fabs(m_calpha) < 1.0e-5)
   {
      m_calpha = 0.0;
      if (m_salpha > 0.0)
         m_salpha = 1.0;
      else
         m_salpha = -1.0;
   }
   else if (fabs(m_salpha) < 1.0e-5)
   {
      m_salpha = 0.0;
      if (m_calpha > 0.0)
         m_calpha = 1.0;
      else
         m_calpha = -1.0;
   }

   m_sasa = m_salpha*m_salpha;
   m_saca = m_salpha*m_calpha;
   m_cacamsasa = 1 - m_sasa - m_sasa;
   m_saca2 = m_saca + m_saca;

   // ------------- z-screw ------------------

   m_stheta = sin(m_thetaMDH);
   m_ctheta = cos(m_thetaMDH);

   // take care of rounding errors
   if (fabs(m_ctheta) < 1.0e-5)
   {
      m_ctheta = 0.0;
      if (m_stheta > 0.0)
         m_stheta = 1.0;
      else
         m_stheta = -1.0;
   }
   else if (fabs(m_stheta) < 1.0e-5)
   {
      m_stheta = 0.0;
      if (m_ctheta > 0.0)
         m_ctheta = 1.0;
      else
         m_ctheta = -1.0;
   }

   m_stst = m_stheta*m_stheta;
   m_stct = m_stheta*m_ctheta;
   m_ctctmstst = 1 - m_stst - m_stst;
   m_stct2 = m_stct + m_stct;


   m_p[0] =  m_aMDH;
   m_p[1] = -m_dMDH*m_salpha;
   m_p[2] =  m_dMDH*m_calpha;
}

//----------------------------------------------------------------------------
void dmMDHLink::getJointLimits(Float *min, Float *max,
                               Float *spring, Float *damper) const
{
   *min = m_min_joint_pos;
   *max = m_max_joint_pos;
   *spring = m_joint_limit_spring;
   *damper = m_joint_limit_damper;
}

//----------------------------------------------------------------------------
void dmMDHLink::setJointLimits(Float min, Float max,
                               Float spring, Float damper)
{
   if (min < max)
   {
      m_min_joint_pos = min;
      m_max_joint_pos = max;
   }
   else
   {
      m_min_joint_pos = max;
      m_max_joint_pos = min;
   }

   m_joint_limit_spring = spring;
   m_joint_limit_damper = damper;

   // hack to make sure joint limits flags and forces are set properly
   Float q = getJointPos();
   setJointPos(q);
}


// ---------------------------------------------------------------------
// Function : rtxToInboard
// Purpose  : rotate a 3d vector from this link's CS to inboard CS
// Inputs   : curr - 3d vector in this link's CS
// Outputs  : prev - 3d vector in inboard link's CS
// ---------------------------------------------------------------------
void dmMDHLink::rtxToInboard(const CartesianVector curr,
                             CartesianVector prev) const
{
   register Float temp1;

// z planar rotation:
   prev[0] = curr[0]*m_ctheta - curr[1]*m_stheta;
   temp1 = curr[1]*m_ctheta + curr[0]*m_stheta;

// x planar rotation
   prev[1] = temp1*m_calpha - curr[2]*m_salpha;
   prev[2] = curr[2]*m_calpha + temp1*m_salpha;
}


// ---------------------------------------------------------------------
// Function : rtxFromInboard
// Purpose  : rotate a 3d vector from inboard link's CS to this CS
// Inputs   : prev - 3d vector in inboard CS
// Outputs  : curr - 3d vector in this CS
// ---------------------------------------------------------------------
void dmMDHLink::rtxFromInboard(const CartesianVector prev,
                               CartesianVector curr) const
{
   register Float temp1;

// x planar rotation.
   temp1 = prev[1]*m_calpha + prev[2]*m_salpha;
   curr[2] = prev[2]*m_calpha - prev[1]*m_salpha;

// z planar rotation.
   curr[0] = prev[0]*m_ctheta + temp1*m_stheta;
   curr[1] = temp1*m_ctheta - prev[0]*m_stheta;
}

// ---------------------------------------------------------------------
// Function : stxFromInboard
// Purpose  : Spatial transform of 6d vector from inboard CS
// Inputs   : prev - 6d vector wrt inboard link's CS
// Outputs  : curr - 6d vector wrt this link's CS
// ---------------------------------------------------------------------
void dmMDHLink::stxFromInboard(const SpatialVector prev,
                               SpatialVector curr) const
{
   register Float tmpa, tmpb, temp1, temp4;

// x-axis screw transform.
   tmpa = prev[5] - m_aMDH*prev[1];
   tmpb = prev[4] + m_aMDH*prev[2];

   temp1 = prev[1]*m_calpha + prev[2]*m_salpha;
   curr[2] = prev[2]*m_calpha - prev[1]*m_salpha;

   temp4 = tmpb*m_calpha + tmpa*m_salpha;
   curr[5] = tmpa*m_calpha - tmpb*m_salpha;

// z-axis screw transform.
   tmpa = temp4 - m_dMDH*prev[0];
   tmpb = prev[3] + m_dMDH*temp1;

   curr[0] = prev[0]*m_ctheta + temp1*m_stheta;
   curr[1] = temp1*m_ctheta - prev[0]*m_stheta;

   curr[3] = tmpb*m_ctheta + tmpa*m_stheta;
   curr[4] = tmpa*m_ctheta - tmpb*m_stheta;
}




// ---------------------------------------------------------------------
// Function : stxToInboard
// Purpose  : Spatial transform of 6d vector to inboard CS
// Inputs   : curr - 6d vector in this link's CS
// Outputs  : prev - 6d vector in inboard link's CS
// ---------------------------------------------------------------------
void dmMDHLink::stxToInboard(const SpatialVector curr,
                             SpatialVector prev) const
{
   register Float tmpa, tmpb, temp1, temp4;

// z-axis transpose screw transform.
   tmpa = curr[0] - m_dMDH*curr[4];
   tmpb = curr[1] + m_dMDH*curr[3];

   prev[0] = tmpa*m_ctheta - tmpb*m_stheta;
   temp1 = tmpb*m_ctheta + tmpa*m_stheta;

   prev[3] = curr[3]*m_ctheta - curr[4]*m_stheta;
   temp4 = curr[4]*m_ctheta + curr[3]*m_stheta;

// x-axis transpose screw transform.
   tmpa = temp1 - m_aMDH*curr[5];
   tmpb = curr[2] + m_aMDH*temp4;

   prev[1] = tmpa*m_calpha - tmpb*m_salpha;
   prev[2] = tmpb*m_calpha + tmpa*m_salpha;

   prev[4] = temp4*m_calpha - curr[5]*m_salpha;
   prev[5] = curr[5]*m_calpha + temp4*m_salpha;
}


// ---------------------------------------------------------------------
// Function : rcongtxToInboardSym
// Purpose  : Congruence transform of 3x3 sym matric to inboard link CS
// Inputs   : Curr - 3x3 symmetric matrix wrt this CS
// Outputs  : Prev - 3x3 symmetric matrix wrt inboard CS.
// ---------------------------------------------------------------------
void dmMDHLink::rcongtxToInboardSym(const CartesianTensor Curr,
                                    CartesianTensor Prev) const
// See page 122 of Labbook #1
{
   register Float tmp1, tmp2;
   CartesianTensor temp;

// z-axis congruence:
   tmp1 = Curr[1][1] - Curr[0][0];
   tmp2 = tmp1*m_stst - Curr[0][1]*m_stct2;

   Prev[0][0] = Curr[0][0] + tmp2;
   temp[0][1] = Curr[0][1]*m_ctctmstst - tmp1*m_stct;
   temp[0][2] = Curr[0][2]*m_ctheta - Curr[1][2]*m_stheta;

   temp[1][1] = Curr[1][1] - tmp2;
   temp[1][2] = Curr[0][2]*m_stheta + Curr[1][2]*m_ctheta;

// x-axis congruence:
   tmp1 = Curr[2][2] - temp[1][1];
   tmp2 = tmp1*m_sasa - temp[1][2]*m_saca2;

   Prev[1][0] = Prev[0][1] = temp[0][1]*m_calpha - temp[0][2]*m_salpha;
   Prev[2][0] = Prev[0][2] = temp[0][1]*m_salpha + temp[0][2]*m_calpha;

   Prev[1][1] = temp[1][1] + tmp2;
   Prev[2][1] = Prev[1][2] = temp[1][2]*m_cacamsasa - tmp1*m_saca;

   Prev[2][2] = Curr[2][2] - tmp2;
}

// ---------------------------------------------------------------------
// Function : rcongtxToInboardGen
// Purpose  : Congruence transform of general 3x3 matrix to inboard CS.
// Inputs   : Curr - 3x3 general matrix in this coordinate system
// Outputs  : Prev - 3x3 general matrix in inboard CS
// ---------------------------------------------------------------------
void dmMDHLink::rcongtxToInboardGen(const CartesianTensor Curr,
                                    CartesianTensor Prev) const
// See page 123 of labbook #1
{
   register Float tmpa, tmpb, tmp1, tmp2;
   CartesianTensor temp;

// z-axis congruence.
   tmpa = Curr[1][1] - Curr[0][0];    // n5 - n1
   tmpb = Curr[0][1] + Curr[1][0];    // n2 + n4

   tmp1 = tmpa*m_stst - tmpb*m_stct;
   tmp2 = tmpa*m_stct + tmpb*m_stst;

   Prev[0][0] = Curr[0][0] + tmp1;
   temp[0][1] = Curr[0][1] - tmp2;
   temp[1][0] = Curr[1][0] - tmp2;
   temp[1][1] = Curr[1][1] - tmp1;

   temp[0][2] = Curr[0][2]*m_ctheta - Curr[1][2]*m_stheta;
   temp[1][2] = Curr[0][2]*m_stheta + Curr[1][2]*m_ctheta;

   temp[2][0] = Curr[2][0]*m_ctheta - Curr[2][1]*m_stheta;
   temp[2][1] = Curr[2][0]*m_stheta + Curr[2][1]*m_ctheta;

// x-axis congruence.
   tmpa = Curr[2][2] - temp[1][1];
   tmpb = temp[1][2] + temp[2][1];
   tmp1 = tmpa*m_sasa - tmpb*m_saca;
   tmp2 = tmpa*m_saca + tmpb*m_sasa;

   Prev[1][1] = temp[1][1] + tmp1;
   Prev[1][2] = temp[1][2] - tmp2;
   Prev[2][1] = temp[2][1] - tmp2;
   Prev[2][2] = Curr[2][2] - tmp1;

   Prev[2][0] = temp[1][0]*m_salpha + temp[2][0]*m_calpha;
   Prev[1][0] = temp[1][0]*m_calpha - temp[2][0]*m_salpha;

   Prev[0][2] = temp[0][1]*m_salpha + temp[0][2]*m_calpha;
   Prev[0][1] = temp[0][1]*m_calpha - temp[0][2]*m_salpha;
}

//----------------------------------------------------------------------------
void dmMDHLink::setActuator(dmActuator *actuator)
{
   // detach an previous actuator cleanly
   if (m_actuator)
   {
      m_SpInertia[m_joint_axis_index][m_joint_axis_index] -=
         ((dmRevDCMotor *)m_actuator)->getRotorInertia();

      delete m_actuator;
   }

   m_actuator = actuator;
   m_actuator->initStiction(m_qd);

   m_SpInertia[m_joint_axis_index][m_joint_axis_index] +=
      ((dmRevDCMotor *)m_actuator)->getRotorInertia();
}

//----------------------------------------------------------------------------
//    Summary: set the state of the link (joint position and velocity) and
//             reset/override any built up contact state
// Parameters: q - an array of one element containing the joint position -
//                 either d or theta depending on the derived joint state
//             qd - an array of one element containing the joint velocity
//    Returns: none
//----------------------------------------------------------------------------
void dmMDHLink::setState(Float q[], Float qd[])
{
   m_qd = qd[0];
   setJointPos(q[0]);

   /* FIXME - I don't think the following is quite right but it must be done
              with the current contact model when overriding the state. */
   // So it doesn't try to stick to a previous point, This may be wrong:
   // if (m_contact_model) m_contact_model->reset();

   // Now we must reset the force objects 'cause that is what the contact model
   // has become...
   for (unsigned int i=0; i<m_force.size(); i++)
   {
      m_force[i]->reset();
   }
}

//----------------------------------------------------------------------------
//    Summary: retrieve the joint position and velocity
// Parameters: none
//    Returns: q - array of one element set to the current joint position
//             qd - array of one element set to the current joint velocity
//----------------------------------------------------------------------------
void dmMDHLink::getState(Float q[], Float qd[]) const
{
   qd[0] = m_qd;
   q[0] = getJointPos();
}

//----------------------------------------------------------------------------
void dmMDHLink::getPose(RotationMatrix R, CartesianVector p) const
{
   R[0][0] = m_ctheta;
   R[1][0] = -m_stheta;
   R[2][0] = 0;

   R[0][1] = m_stheta*m_calpha;
   R[1][1] = m_ctheta*m_calpha;
   R[2][1] = -m_salpha;

   R[0][2] = m_stheta*m_salpha;
   R[1][2] = m_ctheta*m_salpha;
   R[2][2] = m_calpha;

   p[0] = m_p[0];
   p[1] = m_p[1];
   p[2] = m_p[2];
}

//----------------------------------------------------------------------------
void dmMDHLink::initABVars()
{
   // usually called by RigidBody::setInertiaParameters

   register int j, k;

// Initialize the AB variables.
   // 3.0.5
   for (j = 0; j < 6; j++)
   {
      for (k = 0; k < 6; k++)
      {
         m_I_star[j][k] = m_SpInertia[j][k];
      }
   }

// 3.1, 3.2, 3.3, 3.4 (_eta[j] = m_I_star[j][joint_axis_index])
   m_minv = 1.0/m_I_star[m_joint_axis_index][m_joint_axis_index];

   for (j = 0; j < 6; j++)
   {
      m_n_minv[j] = m_minv*m_I_star[j][m_joint_axis_index];
   }

   for (j = 0; j < 6; j++)
   {
      for (k = 0; k < 6; k++)
      {
         m_N_refl[j][k] = m_I_star[j][k] -
            m_n_minv[j]*m_I_star[k][m_joint_axis_index];
      }
   }
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
void dmMDHLink::ABForwardKinematics(Float q[],
                                    Float qd[],
                                    const dmABForKinStruct &link_val_inboard,
                                    dmABForKinStruct &link_val_curr)
{
   register int i, j;

   if (m_actuator)
   {
      m_actuator->updateStiction(&qd[0]);
   }

// set the new state
   m_qd = qd[0];
   setJointPos(q[0]);

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
   link_val_curr.v[m_joint_axis_index] += m_qd;

// 2.7
   computeZeta(link_val_inboard.v, link_val_curr.v, m_zeta);

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
void dmMDHLink::ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
                                   SpatialVector f_star_curr,
                                   SpatialTensor N_refl_curr,
                                   SpatialVector f_star_inboard,
                                   SpatialTensor N_refl_inboard)
{
   register int j, k;

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

// 3.1, 3.2, 3.3, 3.4 (n[j] = m_I_star[j][joint_axis_index])
   m_minv = 1.0/m_I_star[m_joint_axis_index][m_joint_axis_index];

   for (j = 0; j < 6; j++)
   {
      m_n_minv[j] = m_minv*m_I_star[j][m_joint_axis_index];
   }
   m_n_minv[m_joint_axis_index] = 1.0;

   for (j = 0; j < 6; j++)
   {
      for (k = j; k < 6; k++)
      {
         m_N_refl[j][k] = m_N_refl[k][j] = m_I_star[j][k] -
                       m_n_minv[j]*m_I_star[k][m_joint_axis_index];
      }
   }

// 3.6 with joint limits and coulomb friction.  VERIFY
   if (m_actuator)
   {
      m_tau_star = m_actuator->computeTau(m_joint_input,
                                          (m_beta_star[m_joint_axis_index] +
                             (m_joint_limit_flag ? m_tau_limit : 0.0)),
                                          m_qd)
                 + (m_joint_limit_flag ? m_tau_limit : 0.0);
   }
   else
   {
      m_tau_star = m_beta_star[m_joint_axis_index] + m_joint_input +
         (m_joint_limit_flag ? m_tau_limit : (-m_joint_friction*m_qd));
   }

// 3.7
   for (j = 0; j < 6; j++)
   {
      m_gamma[j] = m_beta_star[j] - m_n_minv[j]*m_tau_star;
      if (j != m_joint_axis_index)
      {
         for (k = 0; k < 6; k++)
         {
            if ((k != m_joint_axis_index) && (m_zeta[k] != 0.0))
            {
               m_gamma[j] -= m_N_refl[j][k]*m_zeta[k];
            }
         }
      }
   }

// 3.5 and 3.8
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
void dmMDHLink::ABBackwardDynamicsN(const dmABForKinStruct &link_val_curr,
                                    SpatialVector f_star_inboard,
                                    SpatialTensor N_refl_inboard)
{
   register int j, k;

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

// 3.6  VERIFY
   if (m_actuator)
   {
      m_tau_star =
         m_actuator->computeTau(m_joint_input,
                                (m_beta[m_joint_axis_index] +
                                 (m_joint_limit_flag ? m_tau_limit : 0.0)),
                                m_qd)
         + (m_joint_limit_flag ? m_tau_limit : 0.0);
   }
   else
   {
      m_tau_star = m_beta[m_joint_axis_index] + m_joint_input +
         (m_joint_limit_flag ? m_tau_limit : (-m_joint_friction*m_qd));
   }

// 3.7
   for (j = 0; j < 6; j++)
   {
      m_gamma[j] = m_beta[j] - m_n_minv[j]*m_tau_star;
      if (j != m_joint_axis_index)
      {
         for (k = 0; k < 6; k++)
         {
            if ((k != m_joint_axis_index) && (m_zeta[k] != 0.0))
            {
               m_gamma[j] -= m_N_refl[j][k]*m_zeta[k];
            }
         }
      }
   }

// 3.5 and 3.8
   stxToInboard(m_gamma, f_star_inboard);
   scongtxToInboardIrefl(m_N_refl, N_refl_inboard);
}

//----------------------------------------------------------------------------
//    Summary: third (final) forward recursion of the AB algorithm compute the
//             link's state derivative (velocity and acceleration)
// Parameters: a_inboard - spatial acceleration of inboard link
//    Returns: a_curr    - spatial accel of this link
//             qd - array whose first (and only) element is the time derivative
//                  of joint position - joint velocity
//             qdd - array whose first (and only) element is the time
//                  derivative of joint velocity - joint acceleration
//----------------------------------------------------------------------------
void dmMDHLink::ABForwardAccelerations(SpatialVector a_inboard,
                                       SpatialVector a_curr,
                                       Float qd[],
                                       Float qdd[])
{
   register int j;              // VERIFY CORRECTNESS w/o aprime.

// 5.1 - \bar{a}
   stxFromInboard(a_inboard, a_curr);
   for (j = 0; j < 6; j++)
   {
      a_curr[j] += m_zeta[j];
   }

   // Model motor stiction by appropriately setting joint accel. to
   // zero.  Joint velocity should be set to zero in ABForwardKinematics
   if ((m_actuator) && (m_actuator->getStictionFlag()))
   {
      m_qdd = 0.0;
   }
   else
   {
// 5.2
      m_qdd = m_minv*m_tau_star;

      for (j = 0; j < 6; j++)
      {
         m_qdd -= m_n_minv[j]*a_curr[j];
      }

// 5.3 - \prime{a}
      a_curr[m_joint_axis_index] += m_qdd;
   }

// Transfer new derivative of state values to output vectors
   qd[0] = m_qd;
   qdd[0] = m_qdd;
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
void dmMDHLink::ABForwardAccelerations(SpatialVector a_inboard,
                                       unsigned int *LB,
                                       unsigned int num_elements_LB,
                                       Float ***Xik,
                                       Float **constraint_forces,
                                       unsigned int *num_constraints,
                                       SpatialVector a_curr,
                                       Float qd[],
                                       Float qdd[])
{
   register unsigned int i, j;              // VERIFY CORRECTNESS w/o aprime.

// 5.1 - \bar{a}
   stxFromInboard(a_inboard, a_curr);
   for (j = 0; j < 6; j++)
   {
      a_curr[j] += m_zeta[j];
   }

   // Model motor stiction by appropriately setting joint accel. to
   // zero.  Joint velocity should be set to zero in ABForwardKinematics
   if ((m_actuator) && (m_actuator->getStictionFlag()))
   {
      m_qdd = 0.0;
   }
   else
   {
// 5.2
      m_qdd = m_tau_star;

      for (i = 0; i < num_elements_LB; i++)
      {
         int k = LB[i];

         // Add in effects of constraint forces.
         for (j = 0; j < num_constraints[k]; j++)
            m_qdd += Xik[k][m_joint_axis_index][j]*
               constraint_forces[k][j];
      }

      m_qdd *= m_minv;


      for (j = 0; j < 6; j++)
      {
         m_qdd -= m_n_minv[j]*a_curr[j];
      }

// 5.3 - \prime{a}
      a_curr[m_joint_axis_index] += m_qdd;
   }

// Transfer new derivative of state values to output vectors
   qd[0] = m_qd;
   qdd[0] = m_qdd;
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
void dmMDHLink::XikToInboard(Float **Xik_curr,
                             Float **Xik_prev,
                             int columns_Xik) const
{
   register int r, c;

   Float tmp[6][6]; // Maximum dimensions of tmp (really tmp[6][columns_Xik]).

   for (r = 0; r < 6; r++)
      if ( r == m_joint_axis_index)
      {
         for (c = 0; c < columns_Xik; c++)
            tmp[r][c] = 0.0;
      }
      else
      {
         for (c = 0; c < columns_Xik; c++)
            tmp[r][c] = ( Xik_curr[r][c] -
                          m_n_minv[r]*Xik_curr[m_joint_axis_index][c] );
      }

   // Spatial transform columns of tmp (borrowed code from stxToInboard).

   register Float tmpa, tmpb, temp1, temp4;

   for (c = 0; c < columns_Xik; c++)
   {
      // z-axis transpose screw transform.
      tmpa = tmp[0][c] - m_dMDH*tmp[4][c];
      tmpb = tmp[1][c] + m_dMDH*tmp[3][c];

      Xik_prev[0][c] = tmpa*m_ctheta - tmpb*m_stheta;
      temp1 = tmpb*m_ctheta + tmpa*m_stheta;

      Xik_prev[3][c] = tmp[3][c]*m_ctheta - tmp[4][c]*m_stheta;
      temp4 = tmp[4][c]*m_ctheta + tmp[3][c]*m_stheta;

      // x-axis transpose screw transform.
      tmpa = temp1 - m_aMDH*tmp[5][c];
      tmpb = tmp[2][c] + m_aMDH*temp4;

      Xik_prev[1][c] = tmpa*m_calpha - tmpb*m_salpha;
      Xik_prev[2][c] = tmpb*m_calpha + tmpa*m_salpha;

      Xik_prev[4][c] = temp4*m_calpha - tmp[5][c]*m_salpha;
      Xik_prev[5][c] = tmp[5][c]*m_calpha + temp4*m_salpha;
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
void dmMDHLink::BToInboard(Float **Bkn,
                           Float **Xik, int cols_Xik,
                           Float **Xin, int cols_Xin) const
{
   for (int i = 0; i < cols_Xik; i++)
      for (int j = 0; j < cols_Xin; j++)
         Bkn[i][j] += m_minv*(Xik[m_joint_axis_index][i]*
                              Xin[m_joint_axis_index][j]);
}

//----------------------------------------------------------------------------
//    Summary: Compute the new bias term (zetak) in loop k's loop-closure
//             constraint equation by eliminating link i in favor of its
//             predecessor.
// Parameters: Xik - coefficient of loop k's constraint forces in current
//                   link i's force balance equation
//             cols_Xik - number of columns in Xik
//    Returns: zetak - bias term in loop k's loop-closure constraint equation
//----------------------------------------------------------------------------
void dmMDHLink::xformZetak(Float *zetak,
                           Float **Xik, int cols_Xik) const
{
   register int i, j;

   Float tmp = 0.0;

   for (i = 0; i < 6; i++)
      tmp += m_I_star[m_joint_axis_index][i]*m_zeta[i];

   tmp = m_minv*(m_tau_star - tmp);

   SpatialVector tmpVec;
   tmpVec[0] = m_zeta[0];
   tmpVec[1] = m_zeta[1];
   tmpVec[2] = m_zeta[2];
   tmpVec[3] = m_zeta[3];
   tmpVec[4] = m_zeta[4];
   tmpVec[5] = m_zeta[5];

   tmpVec[m_joint_axis_index] += tmp;

   for (i = 0; i < cols_Xik; i++)
      for (j = 0; j < 6; j++)
         zetak[i] -= Xik[j][i]*tmpVec[j];
}









//---------------------------------------------------------------------
//! \f$ {}^i X_{i-1} \f$, belongs to link i.
//!  DM v5.0
Matrix6F dmMDHLink::get_X_FromParent_Motion()
{
    RotationMatrix R;
    CartesianVector p;
    getPose(R,p);
    Matrix3F R1;
    Vector3F p1; 
    R1 << R[0][0], R[0][1], R[0][2],
          R[1][0], R[1][1], R[1][2],
	  R[2][0], R[2][1], R[2][2];
    p1 << p[0], p[1], p[2];
    Matrix6F X;
    X.block<3,3>(0,0) = R1;
    X.block<3,3>(0,3) = Matrix3F::Zero();
    X.block<3,3>(3,0) = -R1*cr3(p1);
    X.block<3,3>(3,3) = R1;
  
    return X;
}




//-------------------------------------------------------------------
/* The following codes are mostly written in Pure Eigen style. 
   Consider to gradually optimize the following code in the future 
   by switching to those more efficient, DynaMechs-native functions, such as stxToInboard, stxFromInboard etc. */
 //! DM v5.0 function, 
void dmMDHLink::RNEAOutwardFKID(  dmRNEAStruct &link_val2_curr, 
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

	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = X * link_val2_inboard.v + vJ;
	link_val2_curr.a = X * link_val2_inboard.a 
			 + jcalc() * link_val2_curr.qdd
			 + crm( link_val2_curr.v ) * vJ;	
	Matrix6F I = getSpatialInertiaMatrix();
	link_val2_curr.f = I *  link_val2_curr.a  + crf(link_val2_curr.v) * I * link_val2_curr.v;


	if (ExtForceFlag == false)
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
void dmMDHLink::RNEAOutwardFKIDFirst(  dmRNEAStruct &link_val2_curr,
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

	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = vJ;
	link_val2_curr.a = X * a_ini 
			 + jcalc() * link_val2_curr.qdd;	
	Matrix6F I = getSpatialInertiaMatrix();
	link_val2_curr.f = I *  link_val2_curr.a  + crf(link_val2_curr.v) * I * link_val2_curr.v;	

	if (ExtForceFlag == false)
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


//--------------------------------------------------------------------
 //! DM v5.0 function, 
void dmMDHLink::RNEAInwardID(dmRNEAStruct &link_val2_curr,
                                  dmRNEAStruct &link_val2_inboard)
{
	link_val2_curr.tau = jcalc().transpose() * link_val2_curr.f;
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_inboard.f += X.transpose() *  link_val2_curr.f;
}


//--------------------------------------------------------------------
void dmMDHLink::compute_AccBias_First(dmRNEAStruct &link_val2_curr)
{
	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X =  get_X_FromParent_Motion();

	link_val2_curr.v =  vJ;
	link_val2_curr.a =  crm( link_val2_curr.v ) * vJ;

}

//--------------------------------------------------------------------
void dmMDHLink::compute_AccBias(dmRNEAStruct &link_val2_curr,
                                         dmRNEAStruct &link_val2_inboard)
{
	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X =  get_X_FromParent_Motion();

	link_val2_curr.v = X * link_val2_inboard.v + vJ;
	link_val2_curr.a = X * link_val2_inboard.a + crm( link_val2_curr.v ) * vJ;

}


//---------------------------------------------------------------------
//! DM v5.0 function,
void dmMDHLink::computeSpatialVelAndICSPoseFirst(  dmRNEAStruct &link_val2_curr,
                                       CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
                                       RotationMatrix  R_ref_ICS,
                                          Vector6F a_ini)
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

	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = vJ;

}


//----------------------------------------------------------------------
void dmMDHLink::computeSpatialVelAndICSPose(  dmRNEAStruct &link_val2_curr,
                                         dmRNEAStruct &link_val2_inboard)
{

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

	Float q[1], qd[1];
	getState(q,qd);
	Vector6F vJ = jcalc() * qd[0];
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = X * link_val2_inboard.v + vJ;

}
