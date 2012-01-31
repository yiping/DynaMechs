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
 *     File: dmForce.hpp
 *   Author: Scott McMillan
 *  Summary: Abstract base class definition for modelling compliant (functions
 *         : position and velocity) exerted on rigid bodies.
 *****************************************************************************/

#ifndef _DM_FORCE_HPP
#define _DM_FORCE_HPP

#include "dm.h"
#include "dmObject.hpp"

//============================================================================

/**

This is the abstract base class for all force objects that would be associated
with a  dmRigidBody  class.

The \b reset  function is one required by the  dmContactModel  because
certain retained state variables when the state of the rigid body is brutally
overridden (which should never be done in a normal simulation).  As such this
requirement on the interface is most unsatisfying.

The \b computeForce function is the heart of this class which computes the
compliant force based on the rigid body's state information provided by the
\b dmABForKinStruct  parameter and returns the spatial force in the \b
force  parameter.

The \b getBoundaryFlag  function is used by adaptive stepsize
integrators to determine if any force objects encountered collision
boundaries across which the integrator may not be able to converge.
The \b resetBoundaryFlag  function is called to reset the collision
boundary flag prior to an adaptive integration step.  Note that all
force objects share one flag.  Additionally, the \b pushState and
\b popState functions must be implemented for derived force objects
which retain state.  These functions are used by adaptive stepsize
integrators to back out of the integration at collision boundaries.


See also  dmContactModel,  dmRigidBody. */

//============================================================================

class DM_DLL_API dmForce : public dmObject
{
public:
   ///
   dmForce();
   ///
   virtual ~dmForce();

   ///
   virtual void reset() = 0;
   ///
   virtual void computeForce(const dmABForKinStruct &val,
                             SpatialVector force) = 0;
   virtual void computeForce(const dmRNEAStruct &val,
                             SpatialVector force);

   ///
   static bool getBoundaryFlag() {return m_boundary_flag;}
   ///
   static void resetBoundaryFlag() {m_boundary_flag = false;}
   ///
   virtual void pushState() {};
   ///
   virtual void popState() {};

// rendering functions (for future expansion/visualization):
   ///
   virtual void draw() const = 0;

private:
   // not implemented
   dmForce(const dmForce &);
   dmForce &operator=(const dmForce &);

protected:
   static bool m_boundary_flag;
};

#endif
