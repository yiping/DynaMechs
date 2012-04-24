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
 *     File: dmIntegRK4.hpp
 *   Author: Scott McMillan
 *  Summary: 4th order fixed step size Runge Kutta
 *****************************************************************************/

#ifndef _DM_INTEG_RK4_HPP
#define _DM_INTEG_RK4_HPP

#include "dm.h"
#include "dmIntegrator.hpp"

//============================================================================

/**

This class is under major construction.

This is a concrete integrator class derived from the dmIntegrator class,
which implements the fourth order, fixed stepsize Runge-Kutta integration
algorithm.  Note that in the process, four calls to the associated
dmSystem::ABDynamics function will be made. The \b idt parameter
corresponds to the size of the step that is taken with a single iteration of
the algorithm.  No judgement of success or failure of the step is made and the
parameter will be unchanged on return.

 */

//======================================================================

class DM_DLL_API dmIntegRK4 : public dmIntegrator
{
public:
   ///
   dmIntegRK4();
   ///
   virtual ~dmIntegRK4();

   ///
   virtual void simulate(Float &delta_t);

private:
   // not implemented
   dmIntegRK4(const dmIntegRK4 &);
   dmIntegRK4 &operator=(const dmIntegRK4 &);

   virtual bool allocateStateVariables();

private:
   // additional RK4 state and derivative variables (uses m_qy and m_qdy from
   // base class)
   Float *m_qyt, *m_qdyt;
   Float *m_qdym;
   Float *m_qdyb;
};

#endif
