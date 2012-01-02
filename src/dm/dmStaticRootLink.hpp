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
 *     File: dmStaticRootLink.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary:
 *****************************************************************************/

#ifndef _DM_STATIC_ROOT_LINK_HPP
#define _DM_STATIC_ROOT_LINK_HPP

#include "dm.h"
#include "dmLink.hpp"

/**
The {\tt dmStaticRootLink} class is derived from {\tt dmLink}.  It is a
massless link, whose only purpose is for use in systems containing kinematic
loops closed through a fixed base.  In such a case, a {\tt dmStaticRootLink}
object must be used as the root link of the entire system.  This satisfies a
criteria of the simulation algorithm that every loop has a root link.

The static root link has no parameters and no degrees of freedom.  Its
coordinate system is coincident with that of its predecessor, which is generally
the system itself, as this link serves no useful purpose other than as mentioned
above. */

class DM_DLL_API dmStaticRootLink : public dmLink
{
   enum {NUM_DOFS = 0};

public:
   ///
   dmStaticRootLink() {};
   ///
   virtual ~dmStaticRootLink() {};

   ///
   inline int getNumDOFs() const {return NUM_DOFS;}
   ///
   inline void setState(Float [], Float []) {};
   ///
   inline void getState(Float [], Float []) const {};
   ///
   void getPose(RotationMatrix R, CartesianVector p) const;

   ///
   inline void setJointInput(Float []) {};

   // link-to-link transformation functions:
   ///
   void   rtxToInboard(const CartesianVector p_0,
                       CartesianVector p_inboard) const;
   ///
   void rtxFromInboard(const CartesianVector p_inboard,
                       CartesianVector p_0) const;

   ///
   void   stxToInboard(const SpatialVector p_0,
                       SpatialVector p_inboard) const;
   ///
   void stxFromInboard(const SpatialVector p_inboard,
                       SpatialVector p_0) const;

   ///
   void rcongtxToInboardSym(const CartesianTensor M_0,
                            CartesianTensor M_inboard) const;
   ///
   void rcongtxToInboardGen(const CartesianTensor M_0,
                            CartesianTensor M_inboard) const;
   ///
   void scongtxToInboardIrefl(const SpatialTensor M_0,
                              SpatialTensor M_inboard) const;
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

// AB algorithm functions:
   ///
   void ABForwardKinematics(Float [],
                            Float [],
                            const dmABForKinStruct &link_val_inboard,
                            dmABForKinStruct &link_val_curr);

   ///
   void ABBackwardDynamics(const dmABForKinStruct &link_val_curr,
                           SpatialVector f_star_curr,
                           SpatialTensor I_refl_curr,
                           SpatialVector f_star_inboard,
                           SpatialTensor I_refl_inboard);
   ///
   void ABBackwardDynamicsN(const dmABForKinStruct &link_val_curr,
                            SpatialVector f_star_inboard,
                            SpatialTensor I_refl_inboard);

   ///
   void ABForwardAccelerations(SpatialVector a_inboard,
                               SpatialVector a_curr,
                               Float [],
                               Float []);
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
   ///
   virtual Float getPotentialEnergy(const dmABForKinStruct &,
                                    CartesianVector) const { return 0; }
   ///
   virtual Float getKineticEnergy(const dmABForKinStruct &) const { return 0; }

// rendering function:
   ///
   virtual void draw() const;

private:
   dmStaticRootLink(const dmStaticRootLink &);
   dmStaticRootLink &operator=(const dmStaticRootLink &);
};

#endif
