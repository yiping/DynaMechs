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
 *     File: dmRevoluteLink.cpp
 *   Author: Scott McMillan
 *  Summary: Class implementation for links with revolute joints.
 *****************************************************************************/

#include "dm.h"
#include "dmLink.hpp"
#include "dmRigidBody.hpp"
#include "dmMDHLink.hpp"
#include "dmRevoluteLink.hpp"
#include "dmActuator.hpp"

//============================================================================
// class dmRevoluteLink: public dmMDHLink
//============================================================================

//----------------------------------------------------------------------------
//    Summary: constructor
// Parameters: cfg_ptr - ifstream reference to file containing parameters
//                       needed to initialize this object
//    Returns: none
//----------------------------------------------------------------------------
dmRevoluteLink::dmRevoluteLink() : dmMDHLink()
{
   m_joint_axis_index = 2;
}

//----------------------------------------------------------------------------
dmRevoluteLink::~dmRevoluteLink()
{
}

//----------------------------------------------------------------------------
//    Summary: set the joint position (m_thetaMDH) and if the joint limits are
//             exceeded compute the appropriate constraint forces
// Parameters: q - new joint position
//    Returns: none
//----------------------------------------------------------------------------
void dmRevoluteLink::setJointPos(Float q)
{
// Compute limit torques if necessary.
   m_joint_limit_flag = false;
   m_tau_limit = 0.0;
   if (q < m_min_joint_pos)
   {
      m_joint_limit_flag = true;
      m_tau_limit = m_joint_limit_spring*(m_min_joint_pos - q)
                  - m_joint_limit_damper*m_qd;
   }
   else if (q > m_max_joint_pos)
   {
      m_joint_limit_flag = true;
      m_tau_limit = m_joint_limit_spring*(m_max_joint_pos - q)
                  - m_joint_limit_damper*m_qd;
   }

   m_thetaMDH = q;
   m_stheta = sin(q);
   m_ctheta = cos(q);
   m_stst = m_stheta*m_stheta;
   m_stct = m_stheta*m_ctheta;
   m_stct2 = m_stct + m_stct;
   m_ctctmstst = 1.0 - m_stst - m_stst;
}

//----------------------------------------------------------------------------
//    Summary: compute the acceleration bias (state dependent) term, zeta
// Parameters: omega_inboard - angular velocity of the inboard link
//             omega_curr - angular velocity of the current link
//    Returns: zeta - spatial acceleration bias term
//----------------------------------------------------------------------------
void dmRevoluteLink::computeZeta(const CartesianVector omega_inboard,
                                 const CartesianVector omega_curr,
                                 SpatialVector zeta)
{
   //crossproduct(&omega_inboard[0], p, &zeta[3]);
   //crossproduct(&omega_inboard[0], &zeta[3], &zeta[0]);
   //rtxFromInboard(&zeta[0], &zeta[3]);

   zeta[0] = omega_curr[1]*m_qd;
   zeta[1] = -omega_curr[0]*m_qd;
   zeta[2] = 0.0;

   CartesianVector tmp, tmp1;
   crossproduct(&omega_inboard[0], m_p, tmp);
   crossproduct(&omega_inboard[0], tmp, tmp1);
   rtxFromInboard(tmp1, &zeta[3]);
}

//----------------------------------------------------------------------------
//    Summary: compute the acceleration bias (state dependent) term, zeta
// Parameters: omega_inboard - angular velocity of the inboard link
//             omega_curr - angular velocity of the current link
//    Returns: zeta - spatial acceleration bias term
//----------------------------------------------------------------------------
void dmRevoluteLink::scongtxToInboardIrefl(const SpatialTensor N,
                                           SpatialTensor I) const
{
   register int j;
   register Float k1, k2, tmpa, tmpb;
   CartesianTensor tE, tF, tG;
   SpatialTensor Iz;

// *** z-axis congruence transformation ***

// (Bz) Iz11 := Rz'*N11*Rz
   tmpa = N[1][1] - N[0][0];
   k1 = tmpa*m_stst - N[0][1]*m_stct2;
   Iz[0][0] = N[0][0] + k1;
   Iz[1][1] = N[1][1] - k1;
   Iz[0][1] = N[0][1]*m_ctctmstst - tmpa*m_stct;  // k2

// (Cz) Iz22  = Rz'*N22*Rz
   tmpa = N[4][4] - N[3][3];
   k1 = tmpa*m_stst - N[3][4]*m_stct2;

   Iz[3][3] = N[3][3] + k1;
   Iz[3][4] = N[3][4]*m_ctctmstst - tmpa*m_stct;
   Iz[3][5] = N[3][5]*m_ctheta - N[4][5]*m_stheta;

   Iz[4][4] = N[4][4] - k1;
   Iz[4][5] = N[3][5]*m_stheta + N[4][5]*m_ctheta;

   Iz[5][5] = N[5][5];

// (Dz) Iz12 := Rz'*N12*Rz
   tmpa = N[1][4] - N[0][3];
   tmpb = N[0][4] + N[1][3];
   k1 = tmpa*m_stst - tmpb*m_stct;
   k2 = tmpa*m_stct + tmpb*m_stst;

   Iz[0][3] = N[0][3] + k1;
   Iz[0][4] = N[0][4] - k2;
   Iz[0][5] = N[0][5]*m_ctheta - N[1][5]*m_stheta;

   Iz[1][3] = N[1][3] - k2;
   Iz[1][4] = N[1][4] - k1;
   Iz[1][5] = N[0][5]*m_stheta + N[1][5]*m_ctheta;

// (Ez) (pz x)*(Rz'*N22*Rz)
   tE[1][0] = m_dMDH*Iz[3][3];
   tE[1][1] = m_dMDH*Iz[3][4];
   tE[1][2] = m_dMDH*Iz[3][5];

   tE[0][0] = -tE[1][1];
   tE[0][1] = -m_dMDH*Iz[4][4];
   tE[0][2] = -m_dMDH*Iz[4][5];

// (Fz) (pz x)*(Rz'*N22*Rz)*(pz x)'
   tF[0][0] = -m_dMDH*tE[0][1];
   tF[0][1] = m_dMDH*tE[0][0];

   tF[1][1] = m_dMDH*tE[1][0];

// (Gz) (Rz'*N12*Rz)*(pz x)'
   tG[0][0] = -m_dMDH*Iz[0][4];
   tG[0][1] = m_dMDH*Iz[0][3];

   tG[1][0] = -m_dMDH*Iz[1][4];
   tG[1][1] = m_dMDH*Iz[1][3];

// (Hz) Iz11 += (Fz)+(Gz)+(Gz)'
   Iz[0][0] += tF[0][0] + tG[0][0] + tG[0][0];
   Iz[0][1] += tF[0][1] + tG[0][1] + tG[1][0];
   Iz[1][1] += tF[1][1] + tG[1][1] + tG[1][1];

// (Iz) Iz12 += (Ez)
   for (j = 0; j < 3; j++) {
      Iz[0][j + 3] += tE[0][j];
      Iz[1][j + 3] += tE[1][j];
   }


// *** x-axis congruence transformation ***

// (Bx) I11  := Rx'*N11*Rx
   I[2][2] = Iz[1][1]*m_sasa;

   I[1][1] = Iz[1][1] - I[2][2];
   I[1][2] = Iz[1][1]*m_saca;

   I[0][0] = Iz[0][0];
   I[0][1] = Iz[0][1]*m_calpha;
   I[0][2] = Iz[0][1]*m_salpha;


// (Cx) I22  := Rx'*N22*Rx
   tmpa = Iz[5][5] - Iz[4][4];
   k1 = tmpa*m_sasa - Iz[4][5]*m_saca2;

   I[3][3] = Iz[3][3];
   I[3][4] = I[4][3] = Iz[3][4]*m_calpha - Iz[3][5]*m_salpha;
   I[3][5] = I[5][3] = Iz[3][4]*m_salpha + Iz[3][5]*m_calpha;

   I[4][4] = Iz[4][4] + k1;
   I[4][5] = I[5][4] = Iz[4][5]*m_cacamsasa - tmpa*m_saca;  // k2

   I[5][5] = Iz[5][5] - k1;

// (Dx) I12  := Rx'*N12*Rx
   I[2][5] = Iz[1][4]*m_sasa + Iz[1][5]*m_saca;  // k1

   I[2][4] = Iz[1][4]*m_saca - Iz[1][5]*m_sasa;  // k2

   I[2][3] = Iz[1][3]*m_salpha;

   I[1][3] = Iz[1][3]*m_calpha;
   I[1][4] = Iz[1][4] - I[2][5];
   I[1][5] = Iz[1][5] + I[2][4];

   I[0][3] = Iz[0][3];
   I[0][4] = Iz[0][4]*m_calpha - Iz[0][5]*m_salpha;
   I[0][5] = Iz[0][4]*m_salpha + Iz[0][5]*m_calpha;

// (Ex) (px x)*(Rx'*N22*Rx)
   tE[2][2] = m_aMDH*I[4][5];
   tE[2][1] = m_aMDH*I[4][4];
   tE[2][0] = m_aMDH*I[3][4];

   tE[1][2] = -m_aMDH*I[5][5];
   tE[1][1] = -tE[2][2];
   tE[1][0] = -m_aMDH*I[3][5];

// (Fx) (px x)*(Rx'*N22*Rx)*(px x)'
   tF[1][1] = -m_aMDH*tE[1][2];
   tF[1][2] = m_aMDH*tE[1][1];
   tF[2][2] = m_aMDH*tE[2][1];

// (Gx) (Rx'*N12*Rx)*(px x)'
   tG[0][1] = -m_aMDH*I[0][5];
   tG[0][2] = m_aMDH*I[0][4];
   tG[1][1] = -m_aMDH*I[1][5];
   tG[1][2] = m_aMDH*I[1][4];
   tG[2][1] = -m_aMDH*I[2][5];
   tG[2][2] = m_aMDH*I[2][4];

// (Hx) I11 += (Fx)+(Gx)+(Gx)'
   I[1][0] = (I[0][1] += tG[0][1]);
   I[2][0] = (I[0][2] += tG[0][2]);

   I[1][1] += tF[1][1] + tG[1][1] + tG[1][1];
   I[2][1] = (I[1][2] += tF[1][2] + tG[1][2] + tG[2][1]);

   I[2][2] += tF[2][2] + tG[2][2] + tG[2][2];

// (Ix) I12 += (Ex)
   for (j = 0; j < 3; j++) {
      I[j + 3][0] = I[0][j + 3];
      I[j + 3][1] = (I[1][j + 3] += tE[1][j]);
      I[j + 3][2] = (I[2][j + 3] += tE[2][j]);
   }
}


//---------------------------------------------------
Matrix6XF dmRevoluteLink::jcalc()
{
    
    Matrix6XF  S(6,1);
    S << 0, 0, 1, 0, 0, 0;
    return S;
}
