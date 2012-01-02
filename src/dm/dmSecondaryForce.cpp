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
 *     File: dmSecondaryForce.cpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: class implementation for dmSecondaryForce
 *****************************************************************************/

#include "dm.h"
#include "dmSecondaryForce.hpp"

//----------------------------------------------------------------------------
//    Summary: constructor for dmSecondaryForce class
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmSecondaryForce::dmSecondaryForce()
      : m_joint_side(LINK_A),
        m_sec_joint(NULL)
{
}

//----------------------------------------------------------------------------
//    Summary: constructor for dmSecondaryForce class.
//             A dmSecondaryForce is attached to each of the two links
//             connected by a secondary joint in order to apply known
//             forces at the joint (joint inputs, friction, and also
//             constraint forces for compliant (soft) secondary joints)
//             to these links.
// Parameters: side - set whether force object will be attached to the
//                    A link or B link of the joint (LINK_A or LINK_B)
//             joint - pointer to the secondary joint whose forces this
//                     force object is used to transmit
//    Returns: none
//----------------------------------------------------------------------------
dmSecondaryForce::dmSecondaryForce(JointSideEnum side, dmSecondaryJoint *joint)
      : m_joint_side(side),
        m_sec_joint(joint)
{
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmSecondaryForce::~dmSecondaryForce()
{
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmSecondaryForce::reset()
{
}

//----------------------------------------------------------------------------
//    Summary: Returns the known forces applied to the attached link through
//             the associated secondary joint.  Known forces are joint inputs,
//             friction forces, and also constraint forces for compliant
//             (soft) secondary joints.
// Parameters: none
//    Returns: force - known forces expressed w.r.t. the attached link c.s.
//----------------------------------------------------------------------------
void dmSecondaryForce::computeForce(const dmABForKinStruct &,
                                    SpatialVector force)
{
   if (m_joint_side == LINK_A)
      m_sec_joint->getAppliedAForce(force);
   else
      m_sec_joint->getAppliedBForce(force);
}
