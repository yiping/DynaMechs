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
 *     File: dmStaticRootLink.cpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary:
 *****************************************************************************/

#include "dm.h"
#include "dmLink.hpp"
#include "dmStaticRootLink.hpp"

//============================================================================
// class dmStaticRootLink : public dmLink
//============================================================================


//----------------------------------------------------------------------------
//    Summary: retrieve the position and orientation matrix of the 6 dof link
// Parameters: none
//    Returns: R - orientation matrix relative to inboard link
//             p - position of link's c.s. wrt inboard link expressed in
//                 inboard link coordinates
//----------------------------------------------------------------------------
void dmStaticRootLink::getPose(RotationMatrix R, CartesianVector p) const
{
   R[0][0] = R[1][1] = R[2][2] = 1.0;
   R[0][1] = R[0][2] = 0.0;
   R[1][0] = R[1][2] = 0.0;
   R[2][0] = R[2][1] = 0.0;

   p[0] = p[1] = p[2] = 0.0;
}

//----------------------------------------------------------------------------
//    Summary: Rotation of 3d vector from this CS to inboard link
// Parameters: p_0 - 3d vector wrt this link's CS
//    Returns: p_inboard - 3d vector wrt inboard link's CS
//----------------------------------------------------------------------------
void dmStaticRootLink::rtxToInboard(const CartesianVector p_0,
                                    CartesianVector p_inboard) const
{
   p_inboard[0] = p_0[0];
   p_inboard[1] = p_0[1];
   p_inboard[2] = p_0[2];
}

//----------------------------------------------------------------------------
//    Summary: rotate vector wrt inboard CS to this link's CS
// Parameters: p_inboard - 3-vector quantity expressed wrt inboard CS
//    Returns: p_0 - 3-vector result express wrt this CS
//----------------------------------------------------------------------------
void dmStaticRootLink::rtxFromInboard(const CartesianVector p_inboard,
                                      CartesianVector p_0) const
{
   p_0[0] = p_inboard[0];
   p_0[1] = p_inboard[1];
   p_0[2] = p_inboard[2];
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation to inboard links CS to this CS.
// Parameters: p_0 - 6d vector expressed in this CS
//    Returns: p_inboard - 6d vector tx'ed to previous link's CS.
//----------------------------------------------------------------------------
void dmStaticRootLink::stxToInboard(const SpatialVector p_0,
                                    SpatialVector p_inboard) const
{
   p_inboard[0] = p_0[0];
   p_inboard[1] = p_0[1];
   p_inboard[2] = p_0[2];

   p_inboard[3] = p_0[3];
   p_inboard[4] = p_0[4];
   p_inboard[5] = p_0[5];
}

//----------------------------------------------------------------------------
//    Summary: Spatial transformation from inboard link's CS to this CS
// Parameters: p_inboard - 6d vector expressed in inboard link's CS
//    Returns: p_0 - 6d vector in this link's CS
//----------------------------------------------------------------------------
void dmStaticRootLink::stxFromInboard(const SpatialVector p_inboard,
                                      SpatialVector p_0) const
{
   p_0[0] = p_inboard[0];
   p_0[1] = p_inboard[1];
   p_0[2] = p_inboard[2];

   p_0[3] = p_inboard[3];
   p_0[4] = p_inboard[4];
   p_0[5] = p_inboard[5];
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence transform from this link CS to inboard
//             link's CS of a 3x3 symmetric matrix
// Parameters: M_0 - 3x3 symmetric matrix in this CS.
//    Returns: M_inboard - 3x3 symmetric matrix tx'ed to inboard CS.
//----------------------------------------------------------------------------
void dmStaticRootLink::rcongtxToInboardSym(const CartesianTensor M_0,
                                           CartesianTensor M_inboard) const
{
   M_inboard[0][0] = M_0[0][0];
   M_inboard[0][1] = M_inboard[1][0] = M_0[0][1];
   M_inboard[0][2] = M_inboard[2][0] = M_0[0][2];

   M_inboard[1][1] = M_0[1][1];
   M_inboard[1][2] = M_inboard[2][1] = M_0[1][2];

   M_inboard[2][2] = M_0[2][2];
}

//----------------------------------------------------------------------------
//    Summary: rotational congruence transform of general 3x3 to inboard
//             link's CS.
// Parameters: M_0 - general 3x3 matrix in this link's CS. (input)
//    Returns: M_inboard - general 3x3 matrix tx'ed to inboard link's CS.
//----------------------------------------------------------------------------
void dmStaticRootLink::rcongtxToInboardGen(const CartesianTensor M_0,
                                           CartesianTensor M_inboard) const
{
   M_inboard[0][0] = M_0[0][0];
   M_inboard[0][1] = M_0[0][1];
   M_inboard[0][2] = M_0[0][2];

   M_inboard[1][0] = M_0[1][0];
   M_inboard[1][1] = M_0[1][1];
   M_inboard[1][2] = M_0[1][2];

   M_inboard[2][0] = M_0[2][0];
   M_inboard[2][1] = M_0[2][1];
   M_inboard[2][2] = M_0[2][2];
}

//----------------------------------------------------------------------------
//    Summary: Spatial congruence transform of the 6x6 reflected AB inertia to
//             the inboard link's CS
// Parameters: N - 6x6 AB inertia for this link
//    Returns: I - 6x6 reflected AB inertia transformed to inboard link's CS
//----------------------------------------------------------------------------
void dmStaticRootLink::scongtxToInboardIrefl(const SpatialTensor M_0,
                                             SpatialTensor M_inboard) const
{
   register int i, j;

   for (i = 0; i < 6; i++)
      for (j = 0; j < 6; j++)
         M_inboard[i][j] = M_0[i][j];
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
void dmStaticRootLink::ABForwardKinematics(
   Float*,
   Float*,
   const dmABForKinStruct &link_val_inboard,
   dmABForKinStruct &link_val_curr)
{
   register int i, j;

// initialize the computation of the R_ICS and p_ICS
   for (i = 0; i < 3; i++)
   {
      link_val_curr.p_ICS[i] = link_val_inboard.p_ICS[i];

      for (j = 0; j < 3; j++)
         link_val_curr.R_ICS[i][j] = link_val_inboard.R_ICS[i][j];
   }

// must perform transformations to get motion vectors to 0 coord sys.
   stxFromInboard(link_val_inboard.v, link_val_curr.v);

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
void dmStaticRootLink::ABBackwardDynamics(const dmABForKinStruct &,
                                          SpatialVector f_star_curr,
                                          SpatialTensor I_refl_curr,
                                          SpatialVector f_star_inboard,
                                          SpatialTensor I_refl_inboard)
{
   scongtxToInboardIrefl(I_refl_curr, I_refl_inboard);
   stxToInboard(f_star_curr, f_star_inboard);
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
void dmStaticRootLink::ABBackwardDynamicsN(const dmABForKinStruct &,
                                           SpatialVector f_star_inboard,
                                           SpatialTensor I_refl_inboard)
{
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
void dmStaticRootLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              SpatialVector a_curr,
                                              Float*,
                                              Float*)
{
   stxFromInboard(a_inboard, a_curr);
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
void dmStaticRootLink::ABForwardAccelerations(SpatialVector a_inboard,
                                              unsigned int*,
                                              unsigned int,
                                              Float***,
                                              Float**,
                                              unsigned int*,
                                              SpatialVector a_curr,
                                              Float*,
                                              Float*)
{
   stxFromInboard(a_inboard, a_curr);
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
void dmStaticRootLink::XikToInboard(Float **Xik_curr,
                                    Float **Xik_prev,
                                    int columns_Xik) const
{
   register int c;

   for (c = 0; c < columns_Xik; c++)
   {
      Xik_prev[0][c] = Xik_curr[0][c];
      Xik_prev[1][c] = Xik_curr[1][c];
      Xik_prev[2][c] = Xik_curr[2][c];

      Xik_prev[3][c] = Xik_curr[3][c];
      Xik_prev[4][c] = Xik_curr[4][c];
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
void dmStaticRootLink::BToInboard(Float **, Float **, int, Float **, int) const
{
   // Massless link doesn't affect Bmn constraint parameters.
   return;
}

//----------------------------------------------------------------------------
//    Summary: Compute the new bias term (zetak) in loop k's loop-closure
//             constraint equation by eliminating link i in favor of its
//             predecessor.
// Parameters: Xik - coefficient of loop k's constraint forces in current
//                   link i's force balance equation (unused)
//             cols_Xik - number of columns in Xik (unused)
//    Returns: zetak - bias term in loop k's loop-closure constraint
//                     equation after elimination of link i (unused)
//----------------------------------------------------------------------------
void dmStaticRootLink::xformZetak(Float *, Float **, int) const
{
   // Doesn't affect bias term.
   return;
}
