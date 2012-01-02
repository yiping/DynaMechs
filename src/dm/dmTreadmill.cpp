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
 *     File: dmTreadmill.cpp
 *   Author: Steven J. Rodenbaugh
 *  Created: 1999
 *  Summary: Class implementation of dmConveyor, a type of dmEnvironment
 *****************************************************************************/

#include "dm.h"
#include "dmEnvironment.hpp"
#include "dmTreadmill.hpp"

//============================================================================
// class dmEnvironment : public dmObject
//============================================================================

//----------------------------------------------------------------------------
//    Summary: Constructor for dmEnvironment class loads in all sorts of stuff
// Parameters: cfg_ptr - ifstream containing the necessary parameters to
//                initialize this class
//    Returns: none
//----------------------------------------------------------------------------
dmTreadmill::dmTreadmill()
      : dmEnvironment(),
        m_half_width(1),
        m_half_length(1),
        m_q(0),
        m_qd(0),
        m_qdd(0)
        //m_conveyor_filename(NULL)
{
   // Initialize member variables
   for (unsigned int i=0; i<3; i++)
   {
      m_position[i] = 0;
      m_normal[i]   = 0;
      m_forward[i]  = 0;
      m_left[i]     = 0;
   }

   m_forward[0] = 1;
   m_left[1]    = 1;
   m_normal[2]  = 1;
}

//----------------------------------------------------------------------------
dmTreadmill::~dmTreadmill()
{
   //if (m_conveyor_filename)
   //   free(m_conveyor_filename);
}

/*  move this to a loader in dmu
// ---------------------------------------------------------------------------
// Function : loadConveyorData
// Purpose  : Function called from constructor to load conveyor description
// Inputs   : filename containing data
// Outputs  :
// ---------------------------------------------------------------------------
void dmTreadmill::loadConveyorData(const char *filename)
{
   // unallocate if already something there
   if (m_conveyor_filename)
      free (m_conveyor_filename);

   m_conveyor_filename = (char *) malloc(strlen(filename)+1);
   memcpy(m_conveyor_filename, filename, strlen(filename)+1);

   ifstream data_ptr(filename);
   if (!data_ptr)
   {
      cerr << "Unable to open conveyor data file: " << filename << endl;
      exit(3);
   }

   // Coveyor Normal
   readConfigParameterLabel(data_ptr, "Normal_direction");
   data_ptr >> m_normal[0] >> m_normal[1] >> m_normal[2];

   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: invalid normal" << endl;
      data_ptr.close ();
      exit(3);
   }

   // Make sure the vector is normalized
   if (normalize(m_normal) == (Float)0)
   {
      cerr << "Parse error: normal all 0's" << endl;
      data_ptr.close ();
      exit (3);
   }

   // Conveyor Velocity
   readConfigParameterLabel(data_ptr, "Velocity_direction");
   data_ptr >> m_forward[0] >> m_forward[1] >> m_forward[2];
   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: invalid velocity direction" << endl;
      data_ptr.close ();
      exit(3);
   }
   // Make sure the vector is normalized
   if (normalize(m_forward) == (Float)0)
   {
      cerr << "Parse error: velocity direction all 0's" << endl;
      data_ptr.close ();
      exit (3);
   }

   // Conveyor Position - center of conveyor
   readConfigParameterLabel(data_ptr, "Position");
   data_ptr >> m_position[0] >> m_position[1] >> m_position[2];
   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: invalid position" << endl;
      data_ptr.close ();
      exit(3);
   }

   // Conveyor Width
   readConfigParameterLabel(data_ptr, "Width");
   data_ptr >> m_half_width;
   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: width" << endl;
      data_ptr.close ();
      exit(3);
   }
   // Verify greater than 0
   if (m_half_width <= 0)
   {
      cerr << "Parse error: width" << endl;
      data_ptr.close ();
      exit(3);
   }
   m_half_width *= 0.5;

   // Conveyor Length
   readConfigParameterLabel(data_ptr, "Length");
   data_ptr >> m_half_length;
   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: length" << endl;
      data_ptr.close ();
      exit(3);
   }
   // Verify greater than 0
   if (m_half_length <= 0)
   {
      cerr << "Parse error: length" << endl;
      data_ptr.close ();
      exit(3);
   }
   m_half_length *= 0.5;

   // Velocity Magnitude
   readConfigParameterLabel(data_ptr, "Velocity_Mag");
   data_ptr >> m_qd;
   // Verify successful read of data
   if (!data_ptr.good ())
   {
      cerr << "Parse error: Velocity_Mag" << endl;
      data_ptr.close ();
      exit(3);
   }

   data_ptr.close();

   // After the data has been loaded, calculate the Transform from Earth to
   // Conveyor
   computeOrientation();
}
*/

//----------------------------------------------------------------------------
void dmTreadmill::computeOrientation()
{
   // Find y direction
   crossproduct (m_normal, m_forward, m_left);  // (z_dir, x_dir, y_dir);

   // Build the transform from conveyor to earth in conveyor
   // Build the transform from earth to conveyor
   // Build the Rotation matrix from Earth to Conveyor Transposed (ie so
   //    conveyor to earth)
   RotationMatrix cRe;
   m_eTc[0][0] = m_cTe[0][0] = cRe[0][0] = m_forward[0];
   m_eTc[1][0] = m_cTe[0][1] = cRe[0][1] = m_forward[1];
   m_eTc[2][0] = m_cTe[0][2] = cRe[0][2] = m_forward[2];
   m_eTc[0][1] = m_cTe[1][0] = cRe[1][0] = m_left[0];
   m_eTc[1][1] = m_cTe[1][1] = cRe[1][1] = m_left[1];
   m_eTc[2][1] = m_cTe[1][2] = cRe[1][2] = m_left[2];
   m_eTc[0][2] = m_cTe[2][0] = cRe[2][0] = m_normal[0];
   m_eTc[1][2] = m_cTe[2][1] = cRe[2][1] = m_normal[1];
   m_eTc[2][2] = m_cTe[2][2] = cRe[2][2] = m_normal[2];
/*
   CartesianVector cPeorg; // vector from conveyor to earth in conveyor
   rotateCartesianVector (cRe,m_position,cPeorg);
   // Finish building the cTe
   m_cTe[0][3] = cPeorg[0];
   m_cTe[1][3] = cPeorg[1];
   m_cTe[2][3] = cPeorg[2];
   m_cTe[3][0] = m_cTe[3][1] = m_cTe[3][2] = 0;
   m_cTe[3][3] = 1;
   // Finish building the eTc
   m_eTc[0][3] = m_position[0];
   m_eTc[1][3] = m_position[1];
   m_eTc[2][3] = m_position[2];
   m_eTc[3][0] = m_eTc[3][1] = m_eTc[3][2] = 0;
   m_eTc[3][3] = 1;
   // Save the earth z direction in conveyor
   // todo: maybe more efficient to get from m_eTc
   const CartesianVector earthz = {0, 0, 1};
   rotateCartesianVector (cRe, earthz, m_earthz);
*/
}


//----------------------------------------------------------------------------
void dmTreadmill::getPosition(CartesianVector position) const
{
   position[0] = m_position[0];
   position[1] = m_position[1];
   position[2] = m_position[2];
}

//----------------------------------------------------------------------------
void dmTreadmill::setPosition(const CartesianVector position)
{
   m_position[0] = position[0];
   m_position[1] = position[1];
   m_position[2] = position[2];
}


//----------------------------------------------------------------------------
void dmTreadmill::getNormalDirection(CartesianVector normal) const
{
   normal[0] = m_normal[0];
   normal[1] = m_normal[1];
   normal[2] = m_normal[2];
}

//----------------------------------------------------------------------------
void dmTreadmill::setNormalDirection(const CartesianVector normal)
{
   m_normal[0] = normal[0];
   m_normal[1] = normal[1];
   m_normal[2] = normal[2];

   if (normalize(m_normal) == (Float)0)
   {
      m_normal[2] = 1;
   }
   computeOrientation();
}


//----------------------------------------------------------------------------
void dmTreadmill::getVelocityDirection(CartesianVector v_dir) const
{
   v_dir[0] = m_forward[0];
   v_dir[1] = m_forward[1];
   v_dir[2] = m_forward[2];
}

//----------------------------------------------------------------------------
void dmTreadmill::setVelocityDirection(const CartesianVector v_dir)
{
   m_forward[0] = v_dir[0];
   m_forward[1] = v_dir[1];
   m_forward[2] = v_dir[2];

   if (normalize(m_forward) == (Float)0)
   {
      m_forward[0] = 1;
   }
   computeOrientation();
}

//----------------------------------------------------------------------------
void dmTreadmill::setWidth(Float width)
{
   m_half_width = width*0.5;
}

//----------------------------------------------------------------------------
Float dmTreadmill::getWidth() const
{
   return m_half_width*2;
}

//----------------------------------------------------------------------------
void dmTreadmill::setLength(Float length)
{
   m_half_length = length*0.5;
}

//----------------------------------------------------------------------------
Float dmTreadmill::getLength() const
{
   return m_half_length*2;
}

//----------------------------------------------------------------------------
void dmTreadmill::setState(Float q[], Float qd[])
{
   unsigned int offset = dmEnvironment::getNumDOFs();
   m_q  =  q[offset];  // treadmill pos
   m_qd = qd[offset];  // treadmill vel

   if (offset)
      dmEnvironment::setState(q, qd);
}

//----------------------------------------------------------------------------
void dmTreadmill::getState(Float q[], Float qd[]) const
{
   unsigned int offset = dmEnvironment::getNumDOFs();
   q[offset]  = m_q;   // treadmill pos
   qd[offset] = m_qd;  // treadmill vel

   if (offset)
      dmEnvironment::getState(q, qd);
}

//----------------------------------------------------------------------------
void dmTreadmill::dynamics(Float *qy, Float *qdy)
{
   unsigned int offset = dmEnvironment::getNumDOFs();
   m_q = qy[offset];
   qdy[offset] = m_qd = qy[getNumDOFs() + offset];
   qdy[getNumDOFs() + offset] = m_qdd;

   if (offset)
      dmEnvironment::dynamics(qy, qdy);
}
