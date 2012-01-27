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
 *     File: dmPrismaticLink.hpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for links with 1dof translational joints
 *****************************************************************************/

#ifndef _DM_PRISMATIC_LINK_HPP
#define _DM_PRISMATIC_LINK_HPP

#include "dm.h"
#include "dmMDHLink.hpp"

//============================================================================
/**

This class is one of the two concrete link classes derived from the 
dmMDHLink class.  It implements the dynamics specific to a prismatic
(translational) one degree of freedom joint.

<b>Not reference manual material:</b> The primary purpose of this class is to
implement the \b scongToInboardIrefl member function, a very efficient
congruence transformation of the Articulated-Body inertia matrix that is
transformed across the prismatic joint.  In this case, the sixth row and column
of the matrix are zero and taking this into account results in significant
computation savings (for all the gorey details, please read my dissertation or
Robotics and Automation Journal article).  The 
dmMDHLink::ABBackwardDynamics[N] functions call this function in the course of
computing the dynamics.  It should probably be a private/protected member
function, but since it does not modify any member variables there is no harm in
leaving it public.

See also:  dmMDHLink ,  dmLoadFile_dm .
 */

class DM_DLL_API dmPrismaticLink : public dmMDHLink
{
public:
   ///
   dmPrismaticLink();
   ///
   virtual ~dmPrismaticLink();

   ///
   void scongtxToInboardIrefl(const SpatialTensor N, SpatialTensor I) const;

// Rendering functions:
   ///
   void draw() const;

private:
   // not implemented
   dmPrismaticLink(const dmPrismaticLink &);
   dmPrismaticLink &operator=(const dmPrismaticLink &);

   void computeZeta(const CartesianVector omega_inboard,
                    const CartesianVector omega_curr,
                    SpatialVector zeta);

   void setJointPos(Float joint_pos);
   inline Float getJointPos() const { return m_dMDH; }
};

#endif
