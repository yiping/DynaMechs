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
 *     File: dmIntegrator.hpp
 *   Author: Scott McMillan
 *  Project: DynaMechs 3.0
 *  Summary:  Abstract base class for numerical integrators.
 *****************************************************************************/

#ifndef _DM_INTEGRATOR_HPP
#define _DM_INTEGRATOR_HPP

#include "dm.h"
#include "dmObject.hpp"
#include "dmSystem.hpp"

#include <vector>

//============================================================================

/**

This is the abstract base class for numerical integrators that will be used in
conjunction with a {\tt dmSystem} object to perform the dynamic simulation.
After construction of a derived class, one or more {\tt dmSystem} objects must
be added to this object with a call to the {\tt addSystem} function.  Pointers
to currently assigned systems can be obtained with a call to the {\tt
getSystem} function by passing an index corresponding to the order the systems
were added (zero index corresponds to the first system).  Systems can be
removed from an integrator with calls to one of the {\tt removeSystem}
functions (providing either a pointer or index to the system).  If the system
is present and has been removed then these functions will return true.  Removal
of a system causes all the systems added after it to decrease in index.
Removal of a system does not result in the destruction of the state.  It is up
to the user to manage the dmSystems (e.g., a pointer to a system could be
obtained from this class, removed from here, and then destructed).

Immediately after construction of this class, it is not setup to numerically
integrate anything and a call to {\tt isReadyToSim} returns false.  After all
the systems have been assigned and internal variables have been successfully
allocated (based on the number of degrees of freedom in the system) and
initialized, this function will return true.  A call to {\tt synchronizeState}
will initialize these internal state and derivative of state vectors with the
values stored in the system objects.  In some scenarios, it may be necessary
for an application program to override the state of the system by making calls
to {\tt dmLink::setState}.  In this event, these calls must be followed by a
call to {\tt synchronizeState} to update the integrator with the new state of
the system.

The system is simulated by calling the {\tt simulate} function which is a pure
virtual function implemented in the derived integrator classes (see {\tt
dmIntegEuler, dmIntegRK4, dmIntegRK45}, etc.).  The {\tt idt} parameter,
indicates how much time should be simulated in seconds, and on return will
contain the amount of time that was actually simulated.

 */

//======================================================================

class DM_DLL_API dmIntegrator : public dmObject
{
public:
   ///
   dmIntegrator();
   ///
   virtual ~dmIntegrator();

   ///
   unsigned int getNumSystems() const { return m_systems.size(); }
   ///
   dmSystem *getSystem(unsigned int index) const ;

   /// returns true if system is non-NULL and not already added.
   bool addSystem(dmSystem *system);

   /// removes but does not delete the system, returns true when index valid.
   bool removeSystem(unsigned int index);
   /// removes but does not delete the system, returns true when system valid.
   bool removeSystem(dmSystem *system);

   ///
   bool isReadyToSim() const { return m_ready_to_sim; }

   ///
   void synchronizeState();

   ///
   virtual void simulate(Float &idt) = 0;

protected:
   // not implemented
   dmIntegrator(const dmIntegrator &);
   dmIntegrator &operator=(const dmIntegrator &);

   virtual bool allocateStateVariables() = 0;

protected:
   vector<dmSystem*> m_systems;

   bool m_ready_to_sim;  // true if all sim vars allocated properly
   unsigned int m_num_state_vars;

   // minimum required state and derivative variables; subclasses must use
   // these to set the final state and state derivative at the end of the
   // simulate() method (also used at the beginning) as these are the vectors
   // that are initialized with synchronizeState().
   Float *m_qy, *m_qdy;
};

#endif
