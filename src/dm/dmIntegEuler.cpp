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
 *     File: dmIntegEuler.cpp
 *   Author: Scott McMillan
 *  Summary: 1st order fixed stepsize numerical integrator
 *****************************************************************************/

#include "dm.h"
#include "dmIntegEuler.hpp"

//----------------------------------------------------------------------------
dmIntegEuler::dmIntegEuler()
      : dmIntegrator()
{
}

//----------------------------------------------------------------------------
dmIntegEuler::~dmIntegEuler()
{
}

//----------------------------------------------------------------------------
bool dmIntegEuler::allocateStateVariables()
{
   bool success = false;
   m_num_state_vars = 0;

   // delete any preexisting simulation variables
   if (m_qy)   delete [] m_qy;
   if (m_qdy)  delete [] m_qdy;
   m_qy = m_qdy = NULL;

   for (vector<dmSystem*>::iterator elt = m_systems.begin();
        elt != m_systems.end();
        ++elt)
   {
      m_num_state_vars += 2*((*elt)->getNumDOFs());
   }

   if (m_num_state_vars)
   {
      // allocate new ones
      m_qy  = new Float[m_num_state_vars];
      m_qdy = new Float[m_num_state_vars];

      if (m_qy && m_qdy)
      {
         synchronizeState();
         success = true;
      }
   }
   else
   {
      success = true;
   }

   return success;
}

// ---------------------------------------------------------------------------
// Function : simulateEuler
// Purpose  : Perform dynamic sim and numerical integration to obtain new
//              system state
// Inputs   : fixed integration stepsize.
// Outputs  : none
// ---------------------------------------------------------------------------
void dmIntegEuler::simulate(Float &delta_t)
{
   // assume m_ry, m_qy state vectors are consistent with internally stored
   // information.

// Step 1. update the state based previously computed information
   for (unsigned int j = 0; j < m_num_state_vars; j++)
   {
      m_qy[j] += delta_t*m_qdy[j];
   }

// Step 1a. update the derivative based on the new state (and push the new
   // state to the internal variables while I am at it.
   unsigned int index = 0;
   for (vector<dmSystem*>::iterator elt = m_systems.begin();
        elt != m_systems.end();
        ++elt)
   {
      (*elt)->dynamics(&m_qy[index], &m_qdy[index]);
      index += (2*(*elt)->getNumDOFs());
   }
}
