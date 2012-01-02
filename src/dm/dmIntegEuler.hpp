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
 *     File: dmIntegEuler.hpp
 *   Author: Scott McMillan
 *  Summary: 1st order fixed step size Euler
 *****************************************************************************/

#ifndef _DM_INTEG_EULER_HPP
#define _DM_INTEG_EULER_HPP

#include "dm.h"
#include "dmIntegrator.hpp"

//============================================================================

/**

This is a concrete integrator class derived from the {\tt dmIntegrator} class,
which implements the first order, fixed stepsize Euler integration algorithm.
The {\tt idt} parameter corresponds to the size of the step that is taken with
a single iteration of the algorithm.  No judgement of success or failure of the
step is made and the parameter will be unchanged on return.

 */

//======================================================================

class DM_DLL_API dmIntegEuler : public dmIntegrator
{
public:
   ///
   dmIntegEuler();
   ///
   virtual ~dmIntegEuler();

   ///
   virtual void simulate(Float &idt);

private:
   // not implemented
   dmIntegEuler(const dmIntegEuler &);
   dmIntegEuler &operator=(const dmIntegEuler &);

   virtual bool allocateStateVariables();
};

#endif
