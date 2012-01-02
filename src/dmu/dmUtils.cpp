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
 *     File: dmUtils.cpp
 *   Author: Scott McMillan
 *  Created:
 *  Summary: A collection of miscellaneous useful functions
 *****************************************************************************/

#include "../dm/dm.h"
#include "dmu.h"
#include "../dm/dmObject.hpp"
#include "../dm/dmSystem.hpp"
#include "../dm/dmArticulation.hpp"
#include "../dm/dmClosedArticulation.hpp"
#include "../dm/dmLink.hpp"
#include "../dm/dmSecondaryJoint.hpp"

//----------------------------------------------------------------------------
dmObject *dmuFindObject(const char *label, dmArticulation *system)
{
   const char *name;

   if ((system == NULL) || (label == NULL) || (label[0] == '\0'))
   {
      return NULL;
   }

   // check the System object first
   name = system->getName();
   if (name && strcmp(name, label) == 0)
   {
      return system;
   }

   // check the links
   for (unsigned int j=0; j<system->getNumLinks(); j++)
   {
      dmLink *link = system->getLink(j);
      name = link->getName();
      if (name && strcmp(name, label) == 0)
      {
         return link;
      }
   }

   // if system is a closed articulation, check the secondary joints
   dmClosedArticulation *CArt =
      dynamic_cast<dmClosedArticulation*>(system);
   if (CArt)
   {
      // Check 'Hard' secondary joints.
      unsigned int j;
      for (j=0; j<CArt->getNumHardSecondaryJoints(); j++)
      {
         dmSecondaryJoint *secJoint = CArt->getHardSecondaryJoint(j);
         name = secJoint->getName();
         if (name && strcmp(name, label) == 0)
         {
            return secJoint;
         }
      }

      // Check 'Soft' secondary joints.
      for (j=0; j<CArt->getNumSoftSecondaryJoints(); j++)
      {
         dmSecondaryJoint *secJoint = CArt->getSoftSecondaryJoint(j);
         name = secJoint->getName();
         if (name && strcmp(name, label) == 0)
         {
            return secJoint;
         }
      }
   }

   return NULL;
}
