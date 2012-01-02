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
 *     File: dmIntegrator.cpp
 *   Author: Scott McMillan
 *  Summary: Base (abtract) numerical integrator class
 *****************************************************************************/

#include "dm.h"
#include "dmIntegrator.hpp"
#include "dmSystem.hpp"
#include <algorithm>

//----------------------------------------------------------------------------
dmIntegrator::dmIntegrator()
      : dmObject(),
        m_ready_to_sim(false),
        m_num_state_vars(0),
        m_qy(NULL),
        m_qdy(NULL)
{
}

//----------------------------------------------------------------------------
dmIntegrator::~dmIntegrator()
{
   // deallocate any state variables.
   if (m_qy)
   {
      delete [] m_qy;
      delete [] m_qdy;
   }
}

//----------------------------------------------------------------------------
dmSystem *dmIntegrator::getSystem(unsigned int index) const
{
   dmSystem *system = NULL;
   if (index < getNumSystems())
   {
      system = m_systems[index];
   }

   return system;
}

//----------------------------------------------------------------------------
bool dmIntegrator::addSystem(dmSystem *system)
{
   bool success = false;

   if (system &&
       find(m_systems.begin(), m_systems.end(), system) == m_systems.end())
   {
      m_systems.push_back(system);
      m_ready_to_sim = allocateStateVariables();
      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
bool dmIntegrator::removeSystem(unsigned int index)
{
   bool success = false;
   if (index < getNumSystems())
   {
      m_systems.erase(m_systems.begin() + index);
      m_ready_to_sim = allocateStateVariables();
      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
bool dmIntegrator::removeSystem(dmSystem *system)
{
   bool success = false;
   vector<dmSystem*>::iterator elt = m_systems.end();

   if (system &&
       (elt = find(m_systems.begin(),
                   m_systems.end(),
                   system)) != m_systems.end())
   {
      m_systems.erase(elt);
      m_ready_to_sim = allocateStateVariables();
      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
void dmIntegrator::synchronizeState()
{
   unsigned int index = 0;
   for (vector<dmSystem*>::iterator elt = m_systems.begin();
        elt != m_systems.end();
        ++elt)
   {
      (*elt)->initSimVars(&m_qy[index], &m_qdy[index]);
      index += (2*(*elt)->getNumDOFs());
   }
}
