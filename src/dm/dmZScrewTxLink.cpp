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
 *     File: dmZScrewTxLink.cpp
 *   Author: Scott McMillan
 *  Created: 22 May 1998
 *  Summary: Class declaration for Z axial screw transformations (mainly for
 *           efficient branching).
 *****************************************************************************/

#include "dm.h"
#include "dmLink.hpp"
#include "dmZScrewTxLink.hpp"

//============================================================================
// class dmZScrewTxLink : public dmLink
//============================================================================

//----------------------------------------------------------------------------
//    Summary: class constructor
// Parameters: cfg_ptr - pointer to file containing required parameters
//    Returns: none
//----------------------------------------------------------------------------
dmZScrewTxLink::dmZScrewTxLink(Float d, Float theta) :
      dmLink(),
      m_dMDH(d),
      m_thetaMDH(theta)
{
// Get Inboard to Chain base transformation info.
   m_stheta = sin(m_thetaMDH);
   m_ctheta = cos(m_thetaMDH);
   m_stst = m_stheta*m_stheta;
   m_stct = m_stheta*m_ctheta;
   m_ctctmstst = 1.0 - m_stst - m_stst;
   m_stct2 = m_stct + m_stct;

// for hip equations
   m_zeta[0] = m_zeta[1] = m_zeta[2] = 0.0;  // not used
   m_zeta[3] = m_zeta[4] = m_zeta[5] = 0.0;  // used
}

//----------------------------------------------------------------------------
dmZScrewTxLink::~dmZScrewTxLink()
{
}

//----------------------------------------------------------------------------
void dmZScrewTxLink::getPose(RotationMatrix R, CartesianVector p) const
{
   R[0][0] = m_ctheta;
   R[1][0] = -m_stheta;
   R[2][0] = 0;

   R[0][1] = m_stheta;
   R[1][1] = m_ctheta;
   R[2][1] = 0;

   R[0][2] = 0;
   R[1][2] = 0;
   R[2][2] = 1;

   p[0] = 0;
   p[1] = 0;
   p[2] = m_dMDH;
}

//----------------------------------------------------------------------------
//    Summary: Rotation of 3d vector from this CS to inboard link
// Parameters: p_0 - 3d vector wrt this link's CS
//    Returns: p_inboard - 3d vector wrt inboard link's CS
//----------------------------------------------------------------------------
void dmZScrewTxLink::rtxToInboard(const CartesianVector p_0,
                                  CartesianVector p_inboard) const
{
// z planar rotation:
   p_inboard[0] = p_0[0]*m_ctheta - p_0[1]*m_stheta;
   p_inboard[1] = p_0[1]*m_ctheta + p_0[0]*m_stheta;
   p_inboard[2] = p_0[2];
}

//----------------------------------------------------------------------------
//    Summary: rotate vector wrt inboard CS to this link's CS
// Parameters: p_inboard - 3-vector quantity expressed wrt inboard CS
//    Returns: p_0 - 3-vector result express wrt this CS
//----------------------------------------------------------------------------
void dmZScrewTxLink::rtxFromInboard(const CartesianVector p_inboard,
                                    CartesianVector p_0) const
{
// z planar rotation.
   p_0[0] = p_inboard[0]*m_ctheta + p_inboard[1]*m_stheta;
   p_0[1] = p_inboard[1]*m_ctheta - p_inboard[0]*m_stheta;
   p_0[2] = p_inboard[2];
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation to inboard links CS to this CS.
// Parameters: p_0 - 6d vector expressed in this CS
//    Returns: p_inboard - 6d vector tx'ed to previous link's CS.
//----------------------------------------------------------------------------
void dmZScrewTxLink::stxToInboard(const SpatialVector p_0,
                                  SpatialVector p_inboard) const
{
   register Float tmpa, tmpb;

// z-axis transpose screw transform.
   tmpa = p_0[0] - m_dMDH*p_0[4];
   tmpb = p_0[1] + m_dMDH*p_0[3];

   p_inboard[0] = tmpa*m_ctheta - tmpb*m_stheta;
   p_inboard[1] = tmpb*m_ctheta + tmpa*m_stheta;
   p_inboard[2] = p_0[2];

   p_inboard[3] = p_0[3]*m_ctheta - p_0[4]*m_stheta;
   p_inboard[4] = p_0[4]*m_ctheta + p_0[3]*m_stheta;
   p_inboard[5] = p_0[5];
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation from inboard link's CS to this CS
// Parameters: p_inboard - 6d vector expressed in inboard link's CS
//    Returns: p_0 - 6d vector in this link's CS
//----------------------------------------------------------------------------
void dmZScrewTxLink::stxFromInboard(const SpatialVector p_inboard,
                                    SpatialVector p_0) const
{
   register Float tmpa, tmpb;

// z-axis screw transform.
   tmpa = p_inboard[4] - m_dMDH*p_inboard[0];
   tmpb = p_inboard[3] + m_dMDH*p_inboard[1];

   p_0[0] = p_inboard[0]*m_ctheta + p_inboard[1]*m_stheta;
   p_0[1] = p_inboard[1]*m_ctheta - p_inboard[0]*m_stheta;
   p_0[2] = p_inboard[2];

   p_0[3] = tmpb*m_ctheta + tmpa*m_stheta;
   p_0[4] = tmpa*m_ctheta - tmpb*m_stheta;
   p_0[5] = p_inboard[5];
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence tranform from this link CS to inboard
//             link's CS of a 3x3 symmetric matrix
// Parameters: M_0 - 3x3 symmetric matrix in this CS.
//    Returns: M_inboard - 3x3 symmetric matrix tx'ed to inboard CS.
//----------------------------------------------------------------------------
void dmZScrewTxLink::rcongtxToInboardSym(const CartesianTensor M_0,
                                         CartesianTensor M_inboard) const
{
   register Float tmpa, k1;

// z-axis congruence:
   tmpa = M_0[1][1] - M_0[0][0];
   k1 = tmpa*m_stst - M_0[0][1]*m_stct2;

   M_inboard[0][0] = M_0[0][0] + k1;
   M_inboard[0][1] = M_inboard[1][0] = M_0[0][1]*m_ctctmstst - tmpa*m_stct;
   M_inboard[0][2] = M_inboard[2][0] = M_0[0][2]*m_ctheta - M_0[1][2]*m_stheta;

   M_inboard[1][1] = M_0[1][1] - k1;
   M_inboard[1][2] = M_inboard[2][1] = M_0[0][2]*m_stheta + M_0[1][2]*m_ctheta;

   M_inboard[2][2] = M_0[2][2];
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence transform of general 3x3 to inboard
//             link's CS.
// Parameters: M_0 - general 3x3 matrix in this link's CS. (input)
//    Returns: M_inboard - general 3x3 matrix tx'ed to inboard link's CS.
//----------------------------------------------------------------------------
void dmZScrewTxLink::rcongtxToInboardGen(const CartesianTensor M_0,
                                         CartesianTensor M_inboard) const
{
   register Float tmpa, tmpb, k1, k2;

// z-axis congruence.
   tmpa = M_0[1][1] - M_0[0][0];  // n5 - n1

   tmpb = M_0[0][1] + M_0[1][0];  // n2 + n4

   k1 = tmpa*m_stst - tmpb*m_stct;
   k2 = tmpa*m_stct + tmpb*m_stst;

   M_inboard[0][0] = M_0[0][0] + k1;
   M_inboard[0][1] = M_0[0][1] - k2;
   M_inboard[0][2] = M_0[0][2]*m_ctheta - M_0[1][2]*m_stheta;

   M_inboard[1][0] = M_0[1][0] - k2;
   M_inboard[1][1] = M_0[1][1] - k1;
   M_inboard[1][2] = M_0[0][2]*m_stheta + M_0[1][2]*m_ctheta;

   M_inboard[2][0] = M_0[2][0]*m_ctheta - M_0[2][1]*m_stheta;
   M_inboard[2][1] = M_0[2][0]*m_stheta + M_0[2][1]*m_ctheta;
   M_inboard[2][2] = M_0[2][2];
}

//----------------------------------------------------------------------------
//    Summary: Spatial congruence transform of the 6x6 reflected AB inertia to
//             the inboard link's CS
// Parameters: N - 6x6 AB inertia for this link
//    Returns: I - 6x6 reflected AB inertia transformed to inboard link's CS
//----------------------------------------------------------------------------
void dmZScrewTxLink::scongtxToInboardIrefl(const SpatialTensor M_0,
                                           SpatialTensor M_inboard) const
{
// see p. 187 Lab book 1 for details.
   register int i, j;
   register Float k1, k2, tmpa, tmpb;
   CartesianTensor tE, tF, tG;

// (Bz) Rz'*N11*Rz
   tmpa = M_0[1][1] - M_0[0][0];
   k1 = tmpa*m_stst - M_0[0][1]*m_stct2;

   M_inboard[0][0] = M_0[0][0] + k1;
   M_inboard[0][1] = M_0[0][1]*m_ctctmstst - tmpa*m_stct;
   M_inboard[0][2] = M_0[0][2]*m_ctheta - M_0[1][2]*m_stheta;

   M_inboard[1][1] = M_0[1][1] - k1;
   M_inboard[1][2] = M_0[0][2]*m_stheta + M_0[1][2]*m_ctheta;

   M_inboard[2][2] = M_0[2][2];

// (Cz) Rz'*N22*Rz
   tmpa = M_0[4][4] - M_0[3][3];
   k1 = tmpa*m_stst - M_0[3][4]*m_stct2;

   M_inboard[3][3] = M_0[3][3] + k1;
   M_inboard[3][4] = M_0[3][4]*m_ctctmstst - tmpa*m_stct;
   M_inboard[3][5] = M_0[3][5]*m_ctheta - M_0[4][5]*m_stheta;

   M_inboard[4][4] = M_0[4][4] - k1;
   M_inboard[4][5] = M_0[3][5]*m_stheta + M_0[4][5]*m_ctheta;

   M_inboard[5][5] = M_0[5][5];

// (Dz)  Rz'*N12*Rz
   tmpa = M_0[1][4] - M_0[0][3];  // n5 - n1

   tmpb = M_0[0][4] + M_0[1][3];  // n2 + n4

   k1 = tmpa*m_stst - tmpb*m_stct;
   k2 = tmpa*m_stct + tmpb*m_stst;

   M_inboard[0][3] = M_0[0][3] + k1;
   M_inboard[0][4] = M_0[0][4] - k2;
   M_inboard[0][5] = M_0[0][5]*m_ctheta - M_0[1][5]*m_stheta;

   M_inboard[1][3] = M_0[1][3] - k2;
   M_inboard[1][4] = M_0[1][4] - k1;
   M_inboard[1][5] = M_0[0][5]*m_stheta + M_0[1][5]*m_ctheta;

   M_inboard[2][3] = M_0[2][3]*m_ctheta - M_0[2][4]*m_stheta;
   M_inboard[2][4] = M_0[2][3]*m_stheta + M_0[2][4]*m_ctheta;
   M_inboard[2][5] = M_0[2][5];

// (Ez)  (pz x)*(Rz'*N22*Rz)
   tE[1][0] = m_dMDH*M_inboard[3][3];
   tE[1][1] = m_dMDH*M_inboard[3][4];
   tE[1][2] = m_dMDH*M_inboard[3][5];

   tE[0][0] = -tE[1][1];
   tE[0][1] = -m_dMDH*M_inboard[4][4];
   tE[0][2] = -m_dMDH*M_inboard[4][5];

// (Fz)  [(pz x)*(Rz'*N22*Rz)]*(pz x)'
   tF[0][0] = -m_dMDH*tE[0][1];
   tF[0][1] = m_dMDH*tE[0][0];

   tF[1][1] = m_dMDH*tE[1][0];

// (Gz)  (Rz'*N12*Rz)*(pz x)'
   for (j = 0; j < 3; j++) {
      tG[j][0] = -m_dMDH*M_inboard[j][4];
      tG[j][1] = m_dMDH*M_inboard[j][3];
   }

// (Hz) I11 = (B) + (F) + (G) + (G)'
   M_inboard[0][0] += tF[0][0] + tG[0][0] + tG[0][0];
   M_inboard[0][1] += tF[0][1] + tG[0][1] + tG[1][0];
   M_inboard[0][2] += tG[2][0];

   M_inboard[1][1] += tF[1][1] + tG[1][1] + tG[1][1];
   M_inboard[1][2] += tG[2][1];

// (Iz) I12 = (D) + (E);
   for (j = 0; j < 3; j++) {
      M_inboard[0][j + 3] += tE[0][j];
      M_inboard[1][j + 3] += tE[1][j];
   }

   for (i = 0; i < 6; i++)
      for (j = i + 1; j < 6; j++)
         M_inboard[j][i] = M_inboard[i][j];
}

//----------------------------------------------------------------------------
//    Summary: first forward kinematics recursion of the AB simulation alg.
// Parameters: q - current joint position (unused)
//             qd - current joint velocity (unused)
//             link_val_inboard - struct containing kinematic (state dependent)
//                    quantities for the inboard (parent) link/body
//    Returns: link_val_curr - struct filled with kinematic quantities
//                    transformed to this link.
//----------------------------------------------------------------------------
void dmZScrewTxLink::ABForwardKinematics(
   Float*,
   Float*,
   const dmABForKinStruct &link_val_inboard,
   dmABForKinStruct &link_val_curr)
{
   register int i;
   register Float tmp;
   CartesianVector tem;

// initialize the computation of the R_ICS and p_ICS
   for (i = 0; i < 3; i++)
   {
      link_val_curr.p_ICS[i] = link_val_inboard.p_ICS[i] +
                               link_val_inboard.R_ICS[i][2]*m_dMDH;
      rtxFromInboard(&link_val_inboard.R_ICS[i][0],
                     &link_val_curr.R_ICS[i][0]);
   }

// must perform transformations to get motion vectors to 0 coord sys.
   stxFromInboard(link_val_inboard.v, link_val_curr.v);

   tmp = m_dMDH*link_val_inboard.v[2];      // tem = w_r x (w_r x ^r{p}_0)

   tem[0] = tmp*link_val_inboard.v[0];
   tem[1] = tmp*link_val_inboard.v[1];
   tem[2] = -m_dMDH*(link_val_inboard.v[0]*link_val_inboard.v[0] +
                     link_val_inboard.v[1]*link_val_inboard.v[1]);

   rtxFromInboard(tem, &m_zeta[3]);

#ifdef DM_HYDRODYNAMICS
   rtxFromInboard(link_val_inboard.v_f,  link_val_curr.v_f);
   rtxFromInboard(link_val_inboard.a_fg, link_val_curr.a_fg);
#endif
}

//----------------------------------------------------------------------------
//    Summary: the second backward dynamics recursion of the AB algorithm -
//               compute needed quantities for this link
// Parameters: f_star_curr - AB bias force reflected across and transformed
//                           from outboard link to this one
//             I_star_curr - AB spatial inertia reflected across and
//                           transformed  from outboard link to this one
//    Returns: f_star_inboard - AB bias force transformed to the inboard link
//             I_star_inboard - AB spatial inertia transformed to the inboard
//                              link
//----------------------------------------------------------------------------
void dmZScrewTxLink::ABBackwardDynamics(const dmABForKinStruct &,
                                        SpatialVector f_star_curr,
                                        SpatialTensor I_refl_curr,
                                        SpatialVector f_star_inboard,
                                        SpatialTensor I_refl_inboard)
{
   register int i;
   SpatialVector tem;

   // Hip equations.
   scongtxToInboardIrefl(I_refl_curr, I_refl_inboard);

   for (i = 0; i < 6; i++)
   {
      tem[i] = f_star_curr[i] - I_refl_curr[i][3]*m_zeta[3]
                              - I_refl_curr[i][4]*m_zeta[4]
                              - I_refl_curr[i][5]*m_zeta[5];
   }
   stxToInboard(tem, f_star_inboard);
}

//----------------------------------------------------------------------------
//    Summary: the second backward dynamics recursion of the AB algorithm -
//               compute needed quantities for this link when this link is a
//               leaf node in the tree structure (this requires much less
//               computation than ABBackwardDynamics function).  This should
//               never happen in this class b/c it would really be a waste
// Parameters: none
//    Returns: f_star_inboard - AB bias force = zero
//             I_star_inboard - AB spatial inertia = zero
//----------------------------------------------------------------------------
void dmZScrewTxLink::ABBackwardDynamicsN(const dmABForKinStruct &,
                                         SpatialVector f_star_inboard,
                                         SpatialTensor I_refl_inboard)
{
   // Hip equations.
   for (int j=0; j<6; j++)
   {
      I_refl_inboard[j][0] = I_refl_inboard[j][1] =
         I_refl_inboard[j][2] = I_refl_inboard[j][3] =
         I_refl_inboard[j][4] = I_refl_inboard[j][5] = 0.0;
      f_star_inboard[j] = 0.0;
   }
}

//----------------------------------------------------------------------------
//    Summary: third (final) forward recursion of the AB algorithm compute the
//             link's state derivative (velocity and acceleration)
// Parameters: a_inboard - spatial acceleration of inboard link
//    Returns: a_curr    - spatial accel wrt to this coordinate system
//             qd - unused
//             qdd - unused
//----------------------------------------------------------------------------
void dmZScrewTxLink::ABForwardAccelerations(SpatialVector a_inboard,
                                            SpatialVector a_curr,
                                            Float*,
                                            Float*)
{
// spatial transform of acceleration from body CS to shoulder CS (with bias).
   stxFromInboard(a_inboard, a_curr);
   a_curr[3] += m_zeta[3];
   a_curr[4] += m_zeta[4];
   a_curr[5] += m_zeta[5];
}

//----------------------------------------------------------------------------
//    Summary: third (final) forward recursion of the AB algorithm to compute
//             the link's state derivative (velocity and acceleration) for
//             links in systems containing loops
// Parameters: a_inboard - spatial acceleration of inboard link
//             LB - unused
//             num_elements_LB - unused
//             Xik - unused
//             constraint_forces - unused
//             num_constraints - unused
//    Returns: a_curr    - spatial accel of this link
//             qd - unused
//             qdd - unused
//----------------------------------------------------------------------------
void dmZScrewTxLink::ABForwardAccelerations(SpatialVector a_inboard,
                                            unsigned int*,
                                            unsigned int,
                                            Float***,
                                            Float**,
                                            unsigned int*,
                                            SpatialVector a_curr,
                                            Float*,
                                            Float*)
{
// spatial transform of acceleration from body CS to shoulder CS (with bias).
   stxFromInboard(a_inboard, a_curr);
   a_curr[3] += m_zeta[3];
   a_curr[4] += m_zeta[4];
   a_curr[5] += m_zeta[5];
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
void dmZScrewTxLink::XikToInboard(Float **Xik_curr,
                                  Float **Xik_prev,
                                  int columns_Xik) const
{
   register int c;

   // Take spatial transform of columns of Xik_curr.
   // (Borrowed code from stxToInboard.)

   register Float tmpa, tmpb;

   for (c = 0; c < columns_Xik; c++)
   {
      // z-axis transpose screw transform.
      tmpa = Xik_curr[0][c] - m_dMDH*Xik_curr[4][c];
      tmpb = Xik_curr[1][c] + m_dMDH*Xik_curr[3][c];

      Xik_prev[0][c] = tmpa*m_ctheta - tmpb*m_stheta;
      Xik_prev[1][c] = tmpb*m_ctheta + tmpa*m_stheta;
      Xik_prev[2][c] = Xik_curr[2][c];

      Xik_prev[3][c] = Xik_curr[3][c]*m_ctheta - Xik_curr[4][c]*m_stheta;
      Xik_prev[4][c] = Xik_curr[4][c]*m_ctheta + Xik_curr[3][c]*m_stheta;
      Xik_prev[5][c] = Xik_curr[5][c];
   }
}


//----------------------------------------------------------------------------
//    Summary: Compute the new coefficient of loop n's constraint forces
//             in the loop-closure constraint equation for loop k after
//             elimination of the current link (i) in favor of its predecessor
// Parameters: Xik - unused
//             cols_Xik - unused
//             Xin - unused
//             cols_Xin - unused
//    Returns: Bkn - unused
//----------------------------------------------------------------------------
void dmZScrewTxLink::BToInboard(Float **, Float **, int, Float **, int) const
{
   // ZScrew doesn't affect Bmn constraint parameters.
   return;
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
void dmZScrewTxLink::xformZetak(Float *zetak,
                                Float **Xik, int cols_Xik) const
{
   register int i, j;

   for (i = 0; i < cols_Xik; i++)
      for (j = 3; j < 6; j++)             // Know that m_zeta[0:2] = 0.0
         zetak[i] -= Xik[j][i]*m_zeta[j];
}



//---------------------------------------------------------------------
Matrix6F dmZScrewTxLink::get_X_FromParent_Motion()
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
















//--------------------------------------------------------------------
void dmZScrewTxLink::compute_AccBias_First(dmRNEAStruct &link_val2_curr)
{

	Vector6F vJ = Vector6F::Zero();
	Matrix6F X =  get_X_FromParent_Motion();

	link_val2_curr.v =  vJ;
	link_val2_curr.a =  crm( link_val2_curr.v ) * vJ;

}

//--------------------------------------------------------------------
void dmZScrewTxLink::compute_AccBias(dmRNEAStruct &link_val2_curr,
                                         dmRNEAStruct &link_val2_inboard)
{

	Vector6F vJ = Vector6F::Zero();
	Matrix6F X =  get_X_FromParent_Motion();

	link_val2_curr.v = X * link_val2_inboard.v;
	link_val2_curr.a = X * link_val2_inboard.a ;

}












//---------------------------------------------------------------------
//! DM v5.0 function,
void dmZScrewTxLink::computeSpatialVelAndICSPoseFirst(  dmRNEAStruct &link_val2_curr,
                                       CartesianVector  p_ref_ICS,  // articulation w.r.t ICS
                                       RotationMatrix  R_ref_ICS,
                                          Vector6F a_ini)
{
    CartesianVector mp;
    mp[0] = 0; mp[1] = 0; mp[2]= m_dMDH;
	// compute R_ICS and p_ICS)
	for (int i = 0; i < 3; i++)
	{
		link_val2_curr.p_ICS[i] = p_ref_ICS[i];
		for (int j = 0; j < 3; j++)
		{
			link_val2_curr.p_ICS[i] += R_ref_ICS[i][j] * mp[j]; // position Note here!!! not m_p
			rtxFromInboard(&(R_ref_ICS[i][0]),
		     		         &(link_val2_curr.R_ICS[i][0])); //orientation
		}
	}


	Vector6F vJ = Vector6F::Zero();
	link_val2_curr.v = vJ;

}


//----------------------------------------------------------------------
void dmZScrewTxLink::computeSpatialVelAndICSPose(  dmRNEAStruct &link_val2_curr,
                                                   dmRNEAStruct &link_val2_inboard)
{
    CartesianVector mp;
    mp[0] = 0; mp[1] = 0; mp[2]= m_dMDH;
	for (int i = 0; i < 3; i++)
	{
		link_val2_curr.p_ICS[i] = link_val2_inboard.p_ICS[i];
		for (int j = 0; j < 3; j++)
		{
	 		link_val2_curr.p_ICS[i] += link_val2_inboard.R_ICS[i][j] * mp[j]; // position   Note here!!! not m_p
		}
		rtxFromInboard(&(link_val2_inboard.R_ICS[i][0]),
		     		  &(link_val2_curr.R_ICS[i][0])); //orientation
	}


	Vector6F vJ = Vector6F::Zero();
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = X * link_val2_inboard.v;

}


//------------------------------------------------------------------------

Matrix6XF dmZScrewTxLink::jcalc()
{
    Matrix6XF  S(6,1);
    S = Matrix6XF::Zero(6,1);
    return S;
}


//-------------------------------------------------------------------

void dmZScrewTxLink::RNEAOutwardFKID(  dmRNEAStruct &link_val2_curr,
                                     dmRNEAStruct &link_val2_inboard,
                                        bool ExtForceFlag)
{
	// compute the position and orientation of the link in the inertial coordinate system (ICS)

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


	Vector6F vJ = Vector6F::Zero();
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = X * link_val2_inboard.v + vJ;
	link_val2_curr.a = X * link_val2_inboard.a ;
	link_val2_curr.f = Vector6F::Zero();


	if (ExtForceFlag != false)
	{
		// doing nothing, since dmZScrewTxLink does not have a m_force member
	}

}





//------------------------------------------------------------------------
void dmZScrewTxLink::RNEAOutwardFKIDFirst(  dmRNEAStruct &link_val2_curr,
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


	Vector6F vJ = Vector6F::Zero();
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_curr.v = vJ;
	link_val2_curr.a = X * a_ini ;

	link_val2_curr.f = Vector6F::Zero();

	if (ExtForceFlag != false)
	{
		// doing nothing, since dmZScrewTxLink does not have a m_force member
	}
}

//-----------------------------------------------------------------------------

void dmZScrewTxLink::RNEAInwardID(dmRNEAStruct &link_val2_curr,
                                  dmRNEAStruct &link_val2_inboard)
{
	link_val2_curr.tau = Vector6F::Zero();
	Matrix6F X = get_X_FromParent_Motion();
	link_val2_inboard.f += X.transpose() *  link_val2_curr.f;
}

