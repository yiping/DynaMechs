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
 *     File: dmIntegRK45.cpp
 *   Author: Scott McMillan
 *  Summary: Embedded 4th/5th order adaptive-step Runge Kutta integrator
 *****************************************************************************/

#include "dm.h"
#include "dmForce.hpp"
#include "dmIntegRK45.hpp"

//----------------------------------------------------------------------------
//    Summary: default constructor for dmIntegRK45 objects
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmIntegRK45::dmIntegRK45()
      : dmIntegrator(),
        m_qdy2(NULL),
        m_qdy3(NULL),
        m_qdy4(NULL),
        m_qdy5(NULL),
        m_qdy6(NULL),
        m_qy_temp(NULL),
        m_qy_error(NULL),
        m_qy_scale(NULL),
        m_lasth((Float)0.0),
        m_max_steps(250),
        m_eps((Float)1.0e-5),
        m_min_scale((Float)1.0),
        m_euler_step((Float)0.0001)
{
}

//----------------------------------------------------------------------------
//    Summary: destructor for dmIntegRK45 objects which frees allocated
//             variables
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmIntegRK45::~dmIntegRK45()
{
   // deallocate any state variables.
   if (m_qdy2)
   {
      delete [] m_qdy2;
      delete [] m_qdy3;
      delete [] m_qdy4;
      delete [] m_qdy5;
      delete [] m_qdy6;
      delete [] m_qy_temp;
      delete [] m_qy_error;
      delete [] m_qy_scale;
   }
}


//----------------------------------------------------------------------------
//    Summary: set integrator parameters
// Parameters: steps_max - maximum number of integration timesteps in which to
//                         simulate the time period passed to
//                         dmIntegrateRK45::simulate() before giving up
//    Returns: none.
//----------------------------------------------------------------------------
void dmIntegRK45::setMaxSteps(unsigned int steps_max)
{
   if (steps_max > 0)
      m_max_steps = steps_max;
}

//----------------------------------------------------------------------------
//    Summary: set integrator parameters
// Parameters: epsilon - the integrator will ensure that the magnitude of the
//                       difference in the next state as computed using 4th
//                       and 5th order Runge-Kutta is less than 'epsilon' times
//                       the magnitude of the current state for each state
//                       variable (by adjusting the stepsize)
//             max_accuracy - an exception to epsilon is that the next state
//                            values will never be forced to have a difference
//                            less than 'max_accuracy'
//    Returns: none.
//----------------------------------------------------------------------------
void dmIntegRK45::setErrorBound(Float epsilon, Float max_accuracy)
{
   if (epsilon > 0  && max_accuracy > 0)
   {
      m_eps = epsilon;
      m_min_scale = max_accuracy/epsilon;
   }
}

//----------------------------------------------------------------------------
//    Summary: set integrator parameters
// Parameters: euler_step - size of the euler step across a collision
//             boundary.
//    Returns: none.
//----------------------------------------------------------------------------
void dmIntegRK45::setEulerStep(Float euler_step)
{
   if (euler_step > 0)
      m_euler_step = euler_step;
}

//----------------------------------------------------------------------------
//    Summary: Allocate all of the state vectors needed to hold simulation
//             results (intermediate or otherwise).  This function also
//             allocates the base class vectors as well.
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
bool dmIntegRK45::allocateStateVariables()
{
   bool success = false;
   m_num_state_vars = 0;

   // delete any preexisting simulation variables
   if (m_qy)   delete [] m_qy;
   if (m_qdy)  delete [] m_qdy;
   if (m_qdy2) delete [] m_qdy2;
   if (m_qdy3) delete [] m_qdy3;
   if (m_qdy4) delete [] m_qdy4;
   if (m_qdy5) delete [] m_qdy5;
   if (m_qdy6) delete [] m_qdy6;
   if (m_qy_temp)    delete [] m_qy_temp;
   if (m_qy_error)   delete [] m_qy_error;
   if (m_qy_scale)   delete [] m_qy_scale;

   m_qy = m_qdy = m_qdy2 = m_qdy3 = m_qdy4 = m_qdy5 = m_qdy6 = NULL;
   m_qy_temp = m_qy_error = m_qy_scale = NULL;

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
      m_qdy2 = new Float[m_num_state_vars];
      m_qdy3 = new Float[m_num_state_vars];
      m_qdy4 = new Float[m_num_state_vars];
      m_qdy5 = new Float[m_num_state_vars];
      m_qdy6 = new Float[m_num_state_vars];

      m_qy_temp  = new Float[m_num_state_vars];
      m_qy_error = new Float[m_num_state_vars];
      m_qy_scale = new Float[m_num_state_vars];

      if (m_qy && m_qdy && m_qdy2 && m_qdy3 && m_qdy4 && m_qdy5 && m_qdy6 &&
          m_qy_temp && m_qy_error && m_qy_scale)
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
//    Summary: Take trial 4th and 5th order Runge-Kutta integration steps of
//             given stepsize.  Compute potential next state and error
//             estimate.
// Parameters: h - stepsize
//    Returns: sets potential next state (m_qy_temp) and
//             error estimate (m_qy_error) member variables
//----------------------------------------------------------------------------
void dmIntegRK45::rkck(Float h)
{
   //const Float a2=0.2, a3=0.3, a4=0.6, a5=1.0, a6=0.875;

   const Float b21=(Float)0.2;
   const Float b31=(Float)(3.0/40.0), b32=(Float)(9.0/40.0);
   const Float b41=(Float)(0.3), b42=(Float)(-0.9), b43=(Float)(1.2);
   const Float b51=(Float)(-11.0/54.0), b52=(Float)(2.5),
               b53=(Float)(-70.0/27.0), b54=(Float)(35.0/27.0);
   const Float b61=(Float)(1631.0/55296.0), b62=(Float)(175.0/512.0),
               b63=(Float)(575.0/13824.0);
   const Float b64=(Float)(44275.0/110592.0), b65=(Float)(253.0/4096.0);

   const Float c1=(Float)(37.0/378.0), c3=(Float)(250.0/621.0),
               c4=(Float)(125.0/594.0), c6=(Float)(512.0/1771.0);

   const Float dc1=c1-(Float)(2825.0/27648.0), dc3=c3-(Float)(18575.0/48384.0);
   const Float dc4=c4-(Float)(13525.0/55296.0), dc5=(Float)(-277.0/14336.0);
   const Float dc6=c6-(Float)(0.25);

   unsigned int j, index;
   Float h1,h2,h3,h4,h5,h6;

   // ============= Step I. =============
   h1=b21*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + h1*m_qdy[j];
   }

   // m_system->dynamics(/*t+a2*h,*/  m_qy_temp, m_qdy2);
   index = 0;
   vector<dmSystem*>::iterator elt;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy_temp[index], &m_qdy2[index]);
      index += (2*(*elt)->getNumDOFs());
   }

   // ============= Step II. =============
   h1=b31*h;
   h2=b32*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + (h1*m_qdy[j] + h2*m_qdy2[j]);
   }
   // m_system->dynamics(/*t+a3*h,*/ m_qy_temp, m_qdy3);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy_temp[index], &m_qdy3[index]);
      index += (2*(*elt)->getNumDOFs());
   }

   // ============= Step III. =============
   h1=b41*h;
   h2=b42*h;
   h3=b43*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + (h1*m_qdy[j] + h2*m_qdy2[j] + h3*m_qdy3[j]);
   }

   // m_system->dynamics(/*t+a4*h,*/ m_qy_temp, m_qdy4);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy_temp[index], &m_qdy4[index]);
      index += (2*(*elt)->getNumDOFs());
   }

   // ============= Step IV. =============
   h1=b51*h;
   h2=b52*h;
   h3=b53*h;
   h4=b54*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + (h1*m_qdy[j]  + h2*m_qdy2[j] +
                                h3*m_qdy3[j] + h4*m_qdy4[j]);
   }

   // m_system->dynamics(/*t+a5*h,*/ m_qy_temp, m_qdy5);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy_temp[index], &m_qdy5[index]);
      index += (2*(*elt)->getNumDOFs());
   }

   // ============= Step V. =============
   h1=b61*h;
   h2=b62*h;
   h3=b63*h;
   h4=b64*h;
   h5=b65*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + (h1*m_qdy[j]  + h2*m_qdy2[j] + h3*m_qdy3[j] +
                                h4*m_qdy4[j] + h5*m_qdy5[j]);
   }

   // m_system->dynamics(/*t+a6*h,*/ m_qy_temp, m_qdy6);
   index = 0;
   for (elt = m_systems.begin(); elt != m_systems.end(); ++elt)
   {
      (*elt)->dynamics(&m_qy_temp[index], &m_qdy6[index]);
      index += (2*(*elt)->getNumDOFs());
   }

   // ============= Step VI (beginning). =============
   h1=c1*h;
   h3=c3*h;
   h4=c4*h;
   h6=c6*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_temp[j] = m_qy[j] + (h1*m_qdy[j]  + h3*m_qdy3[j] +
                                h4*m_qdy4[j] + h6*m_qdy6[j]);
   }

   // ============= Error Estimate. =============
   h1=dc1*h;
   h3=dc3*h;
   h4=dc4*h;
   h5=dc5*h;
   h6=dc6*h;
   for (j=0; j<m_num_state_vars; ++j)
   {
      m_qy_error[j] = h1*m_qdy[j] + h3*m_qdy3[j] + h4*m_qdy4[j] +
         h5*m_qdy5[j] + h6*m_qdy6[j];
   }
}

//----------------------------------------------------------------------------
//    Summary: Take an integration step which satisfies the quality-control
//             requirements.
// Parameters: htry - initial stepsize to attempt
//    Returns: hdid - actual stepsize done (meeting accuracy requirements)
//             hnext - recommendation for next integration stepsize
//----------------------------------------------------------------------------
void dmIntegRK45::rkqs(Float htry, Float *hdid, Float *hnext)
{
   const Float      SAFETY =  (Float) 0.9;
   const Float    PCORRECT =  (Float)-0.2;
   const Float      ERRCON =  (Float) 1.89e-4; // pow(5/SAFETY,1/PGROW)

   unsigned int j, index;
   Float h, err=0.0, errmax, hscale;

   // Store force widget states in case must back out of integration step.
   // m_system->pushForceStates();
   for (vector<dmSystem*>::iterator elt = m_systems.begin();
        elt != m_systems.end();
        ++elt)
   {
      (*elt)->pushForceStates();
   }

   h=htry;
   for (;;)
   {
      dmForce::resetBoundaryFlag();
      rkck(h);

      if (dmForce::getBoundaryFlag())
      {
         if ( h <= m_euler_step )
         {
            // Perform a small Euler step across discontinuity.
            for (j = 0; j < m_num_state_vars; j++)
            {
               m_qy[j] += m_euler_step*m_qdy[j];
            }

            *hdid = *hnext = m_euler_step;

            // m_system->dynamics(/*t,*/ m_qy, m_qdy);
            index = 0;
            for (vector<dmSystem*>::iterator elt = m_systems.begin();
                 elt != m_systems.end();
                 ++elt)
            {
               (*elt)->dynamics(&m_qy[index], &m_qdy[index]);
               index += (2*(*elt)->getNumDOFs());
            }
            return;
         }
         else
         {
            // m_system->popForceStates();
            for (vector<dmSystem*>::iterator elt = m_systems.begin();
                 elt != m_systems.end();
                 ++elt)
            {
               (*elt)->popForceStates();
            }

            h *= 0.25; // reduce stepsize by factor of 4.0
            continue;
         }
      }

      errmax=0.0;
      for (j=0; j < m_num_state_vars; j++)
      {
         err = m_qy_error[j]/m_qy_scale[j];
         if (err < 0.0) err=-err;
         if (err > errmax)
            errmax = err;
      }

      errmax /= m_eps;

      if (errmax > 1.0)  // Didn't satisfy accuracy requirement.
      {
         hscale = SAFETY*pow(errmax,(Float)PCORRECT);
         if (hscale < 0.1)
            hscale=(Float)0.1;
         h *= hscale;
      }
      else // Satisfied accuracy requirement.
      {
         // Compute final derivative.
         // m_system->dynamics(/*t,*/ m_qy_temp, m_qdy);
         index = 0;
         for (vector<dmSystem*>::iterator elt = m_systems.begin();
              elt != m_systems.end();
              ++elt)
         {
            (*elt)->dynamics(&m_qy_temp[index], &m_qdy[index]);
            index += (2*(*elt)->getNumDOFs());
         }

         // Check if overstepped a force boundary.
         if (dmForce::getBoundaryFlag())
         {
            // m_system->popForceStates();
            for (vector<dmSystem*>::iterator elt = m_systems.begin();
                 elt != m_systems.end();
                 ++elt)
            {
               (*elt)->popForceStates();
            }

            h *= 0.75;  // Reduce stepsize by 1/4.
            continue;
         }
         else  // Success...  Save state, grow stepsize, and return.
         {
            for (j = 0; j < m_num_state_vars; j++)
               m_qy[j] = m_qy_temp[j];

            if (errmax > ERRCON)
               *hnext = SAFETY*h*pow(errmax,(Float)PCORRECT);
            else
               *hnext = 5.0*h;
            *hdid = h;

            return;
         }
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: Simulate system from current time, t, to next time, t+delta_t,
//             using as many adaptively-sized integration steps as necessary.
// Parameters: delta_t - desired time to simulate
//    Returns: delta_t - actual time simulated
//----------------------------------------------------------------------------
void dmIntegRK45::simulate(Float &delta_t)
{
   Float t = 0;
   Float h, hdid;

   unsigned int nstp, j;

   if (m_lasth == 0)
      m_lasth = delta_t;

   for (nstp = 0; nstp < m_max_steps; nstp++)
   {
      for (j = 0; j < m_num_state_vars; j++)
      {
         if ( fabs(m_qy[j]) >= m_min_scale )
            m_qy_scale[j] = fabs(m_qy[j]);
         else
            m_qy_scale[j] = m_min_scale; // Limit the accuracy to max_accuracy.
      }

      h = m_lasth;
      if ((t + h) > delta_t)
         h = delta_t - t;

      rkqs(h, &hdid, &m_lasth);

      t += hdid;
      if (t >= delta_t)
      {
         delta_t = t;
         return;
      }
   }

   cerr << "dmIntegRK45::simulate() error: Too many steps " << endl;
   delta_t = t;
   return;
}
