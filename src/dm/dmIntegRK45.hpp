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
 *     File: dmIntegRK45.hpp
 *   Author: Scott McMillan
 *  Summary: Embedded 4th/5th order adaptive step size Runge Kutta
 *****************************************************************************/

#ifndef _DM_INTEG_RK45_HPP
#define _DM_INTEG_RK45_HPP

#include "dm.h"
#include "dmIntegrator.hpp"

//============================================================================

/**
This is a concrete integrator class derived from the
dmIntegrator class, which implements an embedded fourth/fifth order,
adaptive stepsize Runge-Kutta integration algorithm.  Note that when calling
\b simulate, six calls to the associated dmSystem::dynamics
functions will be made.  The \b delta_t parameter corresponds to the
amount of time that should be simulated, but not necessarily the size
of the step that is taken within a single iteration of the algorithm.
Upon returning from this function, \b delta_t will be set to the amount of
time was simulated.

Four parameters are available to configure the numerical behaviour of this
integrator.  The \b setMaxSteps function specifies the
maximum number of steps are permitted in attempting to simulate the \b
delta_t interval when \b simulate(..) is called before returning with what
it has.  The \b setErrorBound function is the main method of controlling the
accuracy of the integrator with two parameters.  The fourth- and fifth-order
Runge-Kutta steps must produce new state vectors \f$q_{n+1}^{RK4}\f$ and
\f$q_{n+1}^{RK5}\f$ such that \f$|q_{n+1}^{RK4} - q_{n+1}^{RK5}|\f$ is less
than the \b epsilon \f$*q_n\f$ (a function of the first parameter) for each
element.  However, the elements of \f$|q_{n+1}^{RK4} - q_{n+1}^{RK5}|\f$ are never
required to be smaller than the second parameter, \b max_accuracy.
Finally, when the integrator crosses collision boundaries by first approaching
the boundary using successively smaller Runge-Kutte steps, and then finally
taking an Euler step over the boundary.  The stepsize of this Euler step is can
be set by calling \b setEulerStep.
*/

//======================================================================

class DM_DLL_API dmIntegRK45 : public dmIntegrator
{
public:
   ///
   dmIntegRK45();
   ///
   virtual ~dmIntegRK45();

   ///
   void setMaxSteps(unsigned int max_steps);
   ///
   unsigned int getMaxSteps() const { return m_max_steps; }

   /// both must be positive
   void setErrorBound(Float epsilon, Float max_accuracy);
   ///
   void getErrorBound(Float &epsilon, Float &max_accuracy) const
      { epsilon = m_eps;  max_accuracy = m_min_scale*epsilon; }

   ///
   void setEulerStep(Float euler_step);
   ///
   Float getEulerStep() const { return m_euler_step; }

   ///
   virtual void simulate(Float &delta_t);

private:
   // not implemented
   dmIntegRK45(const dmIntegRK45 &);
   dmIntegRK45 &operator=(const dmIntegRK45 &);

   void rkck(Float h);
   void rkqs(Float htry, Float *hdid, Float *hnext);

   virtual bool allocateStateVariables();

private:
   // additional RK45 state and derivative variables (uses m_qy and m_qdy from
   // base class)
   Float *m_qdy2;
   Float *m_qdy3;
   Float *m_qdy4;
   Float *m_qdy5;
   Float *m_qdy6;

   Float *m_qy_temp;
   Float *m_qy_error;
   Float *m_qy_scale;

   Float m_lasth;

   // control parameters
   unsigned int m_max_steps;
   Float        m_eps;
   Float        m_min_scale;
   Float        m_euler_step;
};

#endif
