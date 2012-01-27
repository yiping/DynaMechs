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
 *     File: dmSystem.cpp
 *   Author: Scott McMillan
 *  Summary: Class definitions for dmSystem
 *****************************************************************************/

#include "dm.h"
#include "dmSystem.hpp"
#include "dmEnvironment.hpp"

//============================================================================
// class dmSystem : public dmObject
//============================================================================

//----------------------------------------------------------------------------
//    Summary: default dmSystem constructor initializes an empty (invalid)
//             system.
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmSystem::dmSystem() : dmObject()
{
   QuaternionDM q = {0., 0., 0., 1.};
   CartesianVector p = {0., 0., 0.};
   setRefSystem(q,p);
}


//----------------------------------------------------------------------------
//    Summary: Destructor for dmSystem class.  Deallocates all of the
//             dynamically allocated variables and objects allocated by this
//             class.
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmSystem::~dmSystem()
{
}

//----------------------------------------------------------------------------
//    Summary: define the reference member's coordinate system
// Parameters: quaternion and position vector relative to the origin
//    Returns: none
//----------------------------------------------------------------------------
void dmSystem::setRefSystem(QuaternionDM quat, CartesianVector pos)
{
   normalizeQuat(quat);
   m_quat_ICS[0] = quat[0];
   m_quat_ICS[1] = quat[1];
   m_quat_ICS[2] = quat[2];
   m_quat_ICS[3] = quat[3];

   m_p_ICS[0] = pos[0];
   m_p_ICS[1] = pos[1];
   m_p_ICS[2] = pos[2];

   buildRotMat(quat, m_R_ICS);
}

//----------------------------------------------------------------------------
void dmSystem::getRefSystem(QuaternionDM quat, CartesianVector pos) const
{
   quat[0] = m_quat_ICS[0];
   quat[1] = m_quat_ICS[1];
   quat[2] = m_quat_ICS[2];
   quat[3] = m_quat_ICS[3];

   pos[0] = m_p_ICS[0];
   pos[1] = m_p_ICS[1];
   pos[2] = m_p_ICS[2];
}

//----------------------------------------------------------------------------
void dmSystem::getPose(RotationMatrix R, CartesianVector pos) const
{
   for (unsigned int i=0; i<3; i++)
   {
      R[i][0] = m_R_ICS[i][0];
      R[i][1] = m_R_ICS[i][1];
      R[i][2] = m_R_ICS[i][2];

      pos[i]  = m_p_ICS[i];
   }
}

//----------------------------------------------------------------------------
//   Function: forwardKinematics
void dmSystem::forwardKinematics(dmABForKinStruct &fk) const
{
   register int i, j;

   for (i = 0; i < 3; i++)
   {
      fk.p_ICS[i] = m_p_ICS[i];
      for (j = 0; j < 3; j++)
      {
         fk.R_ICS[i][j] = m_R_ICS[j][i];
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: initialize all of the variables needed for the numerical
//             integration algorithms.  Note that it queries the articulations
//             for the number of DOFS in each; therefore, all links should be
//             assigned to the articulations before this function is called.
// Parameters: none
//    Returns: none (I should check for valid allocation and return true if
//             this function is successfully completed.
//----------------------------------------------------------------------------
void dmSystem::initSimVars(Float *qy, Float *qdy)
{
   unsigned int num_vars = getNumDOFs();

   getState(&qy[0], &qy[num_vars]);

   for (unsigned int j=0; j<num_vars; j++)
   {
      /* FIXME */
      // In the case of SphericalLinks
      // the following commented line incorrectly transfers angular velocity
      // components to Euler angle rates in derivative state vector.
      //
      // m_qdy[aindex + j] = m_qy[aindex + j+m_dofs_per_articulation[k]];
      //
      // for now I am setting the Euler angle rates to zero will be
      // incorrect for first iteration only if angular velocity is nonzero
      qdy[j] = 0.0;
      qdy[j + num_vars] = 0.0;
   }
}
