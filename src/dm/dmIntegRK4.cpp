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
 *     File: dmIntegRK4.cpp
 *   Author: Scott McMillan
 *  Summary: 4th order fixed-step Runge Kutta integrator
 *****************************************************************************/

#include "dm.h"
#include "dmIntegRK4.hpp"

//----------------------------------------------------------------------------
dmIntegRK4::dmIntegRK4()
      : dmIntegrator(),
        m_qyt(NULL),
        m_qdyt(NULL),
        m_qdym(NULL),
        m_qdyb(NULL)
{
}

//----------------------------------------------------------------------------
dmIntegRK4::~dmIntegRK4()
{
   // deallocate any state variables.
   if (m_qyt)
   {
      delete [] m_qyt;
      delete [] m_qdyt;
      delete [] m_qdym;
      delete [] m_qdyb;
   }
}

//----------------------------------------------------------------------------
bool dmIntegRK4::allocateStateVariables()
{
   bool success = false;
   m_num_state_vars = 0;

   // delete any preexisting simulation variables
   if (m_qy)   delete [] m_qy;
   if (m_qdy)  delete [] m_qdy;
   if (m_qyt)  delete [] m_qyt;
   if (m_qdyt) delete [] m_qdyt;
   if (m_qdym) delete [] m_qdym;
   if (m_qdyb) delete [] m_qdyb;
   m_qy = m_qdy = m_qyt = m_qdyt = m_qdym = m_qdyb = NULL;

   for (vector<dmSystem*>::iterator elt = m_systems.begin();
        elt != m_systems.end();
        ++elt)
   {
      m_num_state_vars += 2*((*elt)->getNumDOFs());
   }

   if (m_num_state_vars)
   {
      // allocate new ones
      m_qy   = new Float[m_num_state_vars];
      m_qdy  = new Float[m_num_state_vars];
      m_qyt  = new Float[m_num_state_vars];
      m_qdyt = new Float[m_num_state_vars];
      m_qdym = new Float[m_num_state_vars];
      m_qdyb = new Float[m_num_state_vars];

      if (m_qy && m_qdy && m_qyt && m_qdyt && m_qdym && m_qdyb)
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

//----------------------------------------------------------------------------
//   Function: simulateRK4
//    Summary: simulate system from current time, t, to next time t+h with
//             one RK4 iteration.
// Parameters: fixed integration stepsize
//    Returns: nothing
//----------------------------------------------------------------------------
void dmIntegRK4::simulate(Float &delta_t)
{
   unsigned int j;

   Float h2 = delta_t/2.0;
   Float h6 = delta_t/6.0;

// Step 1. Forward Euler half step across interval [t, t+h).
   // Again assume m_qy, m_ry match the internal state and m_qdy and m_rdy are
   // the corresponding derivative of state vectors
   for (j = 0; j < m_num_state_vars; j++)
   {
      m_qyt[j] = m_qy[j] + h2*m_qdy[j];
   }

   // derfun(t+h/2, &torque[torque_index][0][0], qyt, qdyt)
   unsigned int index = 0;
   vector<dmSystem*>::iterator elt;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qyt[index], &m_qdyt[index]);
      index += (2*(*elt)->getNumDOFs());
   }

// Step 2.
   for (j = 0; j < m_num_state_vars; j++)
   {
      m_qyt[j] = m_qy[j] + h2*m_qdyt[j];
   }

   // derfun(t+h/2, &torque[torque_index][0][0], qyt, qdym);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qyt[index], &m_qdym[index]);
      index += (2*(*elt)->getNumDOFs());
   }

// Step 3.
   for (j = 0; j < m_num_state_vars; j++)
   {
      m_qyt[j] = m_qy[j] + delta_t*m_qdym[j];
      m_qdym[j] += m_qdyt[j];
   }

   // derfun(t+h, &torque[torque_index+torque_skip][0][0], qyt, qdyb);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qyt[index], &m_qdyb[index]);
      index += (2*(*elt)->getNumDOFs());
   }

// Step 4. accumulate results.
   for (j = 0; j < m_num_state_vars; j++)
   {
      m_qy[j] += h6*(m_qdy[j] + m_qdyb[j] + 2.0*m_qdym[j]);
   }

   // derfun(t+h, &torque[torque_index + torque_skip][0][0], qy, qdy);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy[index], &m_qdy[index]);
      index += (2*(*elt)->getNumDOFs());
   }
}
