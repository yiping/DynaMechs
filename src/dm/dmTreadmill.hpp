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
 *     File: dmTreadmill.hpp
 *   Author: Steven Rodenbaugh
 *  Created: 2000
 *  Summary: class definition for a dynamic environment with a treadmill
 *****************************************************************************/

#ifndef _DM_TREADMILL_HPP
#define _DM_TREADMILL_HPP

#include "dm.h"
#include "dmObject.hpp"
#include "dmEnvironment.hpp"

#include <vector>

//======================================================================

/* Steven Rodenbaugh writes....

General Notes:
- Might make this be a child a general class like dmMovingEnvironment
- Need generalized way to specify environment type -probably change to env file
- Need permission from Scott to be using his code?
- Need class description here
- Should probably add OpenGL support equivalent to rest of DynaMechs

- The parsing method follows the same technique utilized in the rest of
       of DynaMechs (This does have the drawback that there is not a graceful
       recovery from parse errors.  In the future, that should probably be
       addressed.)
- The information that strictly concerns the graphical representation is
       included in another file because it does not impact the simulation
*/


//======================================================================

/**
   The treadmill is a dynamic environment requiring simulation to update its
   state.  It can be thought of as surface with a horizontal velocity
   component.  It has a length along the direction of motion, width
   perpendicular in the horizontal plane, and a normal (up) direction.

   It is derived from {\tt dmEnvironment}, and when hydrodynamics are enabled
   in the build, this class appends to the base class state vectors.
 */


class DM_DLL_API dmTreadmill : public dmEnvironment
{
public:
   ///
   dmTreadmill();
   ///
   virtual ~dmTreadmill();

   ///
   void setPosition(const CartesianVector position);
   ///
   void getPosition(CartesianVector pos) const;

   ///
   void setWidth (Float width);
   ///
   Float getWidth() const;

   ///
   void setLength (Float length);
   ///
   Float getLength() const;

   // dmSystem functions
   ///
   virtual unsigned int getNumDOFs() const
      {
         return dmEnvironment::getNumDOFs() + 1;
      }
   ///
   virtual void setState(Float treadmill_pos[], Float treadmill_vel[]);
   ///
   virtual void getState(Float treadmill_pos[], Float treadmill_vel[]) const;

   ///
   void setVelocityDirection(const CartesianVector v_dir);
   ///
   void getVelocityDirection(CartesianVector v_dir) const;

   ///
   void setNormalDirection(const CartesianVector normal);
   ///
   void getNormalDirection(CartesianVector normal) const;

   /// These functions should be moved to the loader
   //void loadConveyorData(const char *filename);
   ///
   //const char *getConveyorFilename() const {return m_conveyor_filename;}

   ///
   //virtual Float getGroundDepth(CartesianVector contact_pos,
   //                             CartesianVector ground_normal);

   ///
   //virual Float getGroundElevation(CartesianVector contact_pos,
   //                                CartesianVector ground_normal);

   // dynamics algorithm
   ///
   virtual void dynamics(Float *qy, Float *qdy);
   virtual void draw() const;

protected: // Functions
   void computeOrientation();

private: // Functions
   dmTreadmill(const dmTreadmill &);
   dmTreadmill &operator=(const dmTreadmill &);

private: // Variables
   Float m_half_width;
   Float m_half_length;
   CartesianVector m_position;
   CartesianVector m_normal;    // normalized
   CartesianVector m_forward;   // normalized
   CartesianVector m_left;      // normalized

   Float m_q, m_qd, m_qdd;

   //CartesianVector m_earthz;
   HomogeneousTransformationMatrix m_eTc, m_cTe;
};

#endif
