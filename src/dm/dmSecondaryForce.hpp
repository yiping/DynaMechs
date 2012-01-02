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
 *     File: dmSecondaryForce.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: dmForce subclass for applying known forces at secondary
 *           joints to the connected links.  Known forces are joint inputs,
 *           friction forces, and also constraint forces for compliant
 *           (soft) secondary joints.
 *****************************************************************************/

#ifndef _DM_SECONDARY_FORCE_HPP
#define _DM_SECONDARY_FORCE_HPP

#include "dm.h"
#include "dmForce.hpp"
#include "dmSecondaryJoint.hpp"

//============================================================================
/**

The {\tt dmSecondaryForce} class is a concrete implementation of the
{\tt dmForce} class for applying known forces through secondary
joints.  Known forces are secondary joint inputs and joint friction
forces.  Additionally, if compliant (soft) secondary joints are used,
the joint constraint forces are also known.  An object of this class
is automatically created and added to the specified link by a call
to {\tt dmSecondaryJoint::setLinkA/B}.

The constructor for the class requires as parameters a {\tt JointSideEnum},
which specifies the joint side of the attached link, and a pointer to the
associated {\tt dmSecondaryJoint} object.  The {\tt computeForce} function,
required of all {\tt dmForce} objects, simply returns the appropriate
forces as previously computed by the {\tt dmSecondaryJoint} object.

 */

class dmSecondaryForce : public dmForce
{
public:
   enum JointSideEnum { LINK_A = 0, LINK_B = 1 };
public:
   ///
   dmSecondaryForce();
   ///
   dmSecondaryForce(JointSideEnum side, dmSecondaryJoint *joint);
   ///
   virtual ~dmSecondaryForce();

   ///
   virtual void reset();
   ///
   virtual void computeForce(const dmABForKinStruct &val,
                             SpatialVector force);

   ///
   void drawInit() const {};
   ///
   void draw() const {};

private:
   JointSideEnum     m_joint_side;    // 0 = linkA, 1 = linkB
   dmSecondaryJoint *m_sec_joint;
};

#endif
