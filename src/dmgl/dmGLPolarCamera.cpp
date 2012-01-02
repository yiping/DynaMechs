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
 *     File: dmGLPolarCamera.cpp
 *   Author: Scott McMillan
 *  Created: 21 March 1997
 *  Summary:
 *****************************************************************************/

#include "dmGL.h"
#include "dmGLMouse.hpp"
#include "dmGLCamera.hpp"
#include "dmGLPolarCamera.hpp"

const float DEGTORAD = (float)(M_PI/180.0);

//============================================================================
// class dmGLPolarCamera : public dmGLCamera
//============================================================================

//----------------------------------------------------------------------------
//   Function: dmGLPolarCamera Constructor
//    Summary: This will initialize a camera that will view a scene
//             constructed in a Performer scene graph
// Parameters: Performer channel this camera will be responsible for
//    Returns: none
//----------------------------------------------------------------------------
dmGLPolarCamera::dmGLPolarCamera() :  dmGLCamera()
{
   reset();
}

//----------------------------------------------------------------------------
//   Function: dmGLPolarCamera Destructor
//    Summary:
// Parameters:
//    Returns: none
//----------------------------------------------------------------------------
dmGLPolarCamera::~dmGLPolarCamera()
{
#ifdef DEBUG
   cerr << "destructing dmGLPolarCamera" << endl;
#endif
}

//----------------------------------------------------------------------------
//   Function:
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLPolarCamera::reset()
{
   dmGLCamera::reset();

   m_pos_coi[0] = m_pos_coi[1] = m_pos_coi[2] = 0.0;
   setAzimuth(0.0);
   setElevation(0.0);
   setRadius(0.5);
   m_trans_scale = 1.0;
}


//----------------------------------------------------------------------------
//   Function:
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLPolarCamera::outputState()
{
   cerr << "dmGLPolarCamera state:" << endl;
   cerr << "Azimuth:   " << m_azimuth << endl;
   cerr << "Elevation: " << m_elevation << endl;
   cerr << "Radius:    " << m_radius << endl;
   cerr << "COI pos:   " << m_pos_coi[0] << ", "
                         << m_pos_coi[1] << ", "
                         << m_pos_coi[2] << endl;
}

//----------------------------------------------------------------------------
//   Function: updateAzimuth
//    Summary: Change the azimuth angle of the camera (works in all modes)
// Parameters: change in azimuth angle, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::updateAzimuth(float delta_azimuth)
{
   setAzimuth(m_azimuth + delta_azimuth);
}


//----------------------------------------------------------------------
//   Function: setAzimuth
//    Summary: Change the azimuth angle of the camera (works in all modes)
// Parameters: change in azimuth angle, in degrees
//    Returns: none
//----------------------------------------------------------------------
void dmGLPolarCamera::setAzimuth(float new_azimuth)
{
   m_azimuth = new_azimuth;

   // allow it to wrap around
   if (m_azimuth > 180.0)
   {
      m_azimuth -= 360.0;
   }
   else if (m_azimuth < -180.0)
   {
      m_azimuth += 360.0;
   }

   double tmp = m_azimuth*DEGTORAD;
   m_sin_az = sin(tmp);
   m_cos_az = cos(tmp);
}

//----------------------------------------------------------------------------
//   Function: updateElevation
//    Summary: Change the elevation angle of the camera (works in all modes)
// Parameters: change in elevation angle, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::updateElevation(float delta_elevation)
{
   setElevation(m_elevation + delta_elevation);
}

//----------------------------------------------------------------------
//   Function: setElevation
//    Summary: Change the elevation angle of the camera (works in all modes)
// Parameters: change in elevation angle, in degrees
//    Returns: none
//----------------------------------------------------------------------
void dmGLPolarCamera::setElevation(float new_elevation)
{
   m_elevation = new_elevation;
   if (m_elevation > 90.0)         // looking straight down from overhead
      m_elevation = 90.0;
   else if (m_elevation < -90.0)   // straight up from underneath
      m_elevation = -90.0;

   double tmp = m_elevation*DEGTORAD;
   m_sin_el = sin(tmp);
   m_cos_el = cos(tmp);
}

//----------------------------------------------------------------------------
//   Function: updateRadius
//    Summary: Change the distance of the camera from the center of
//             interest.  This only works in the HTPF_CAM_COI mode where
//             the camera always points at the robot.
// Parameters: change in distance to robot, in meters
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::updateRadius(float delta_radius)
{
   setRadius(m_radius + delta_radius);
}

//----------------------------------------------------------------------------
//   Function: setRadius
//    Summary: Change the distance of the camera from the center of
//             interest.  This only works in the CAM_ROBOT mode where
//             the camera always points at the robot.
// Parameters: change in distance to robot, in meters
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::setRadius(float new_radius)
{
   m_radius = new_radius;
   if (m_radius < 0.001f)           // don't let it go negative or zero
   {
      m_radius = 0.001f;
   }
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: set absolute position of the camera wrt to world/Performer
//             coordinate system.
// Parameters: 3D vector specifying the new position
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::updateCOI(float delta_coi[3])
{
   m_pos_coi[0] += delta_coi[0];
   m_pos_coi[1] += delta_coi[1];
   m_pos_coi[2] += delta_coi[2];
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: set absolute position of the camera wrt to world/Performer
//             coordinate system.
// Parameters: 3D vector specifying the new position
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::setCOI(float pos_coi[3])
{
   m_pos_coi[0] = pos_coi[0];
   m_pos_coi[1] = pos_coi[1];
   m_pos_coi[2] = pos_coi[2];
}

//----------------------------------------------------------------------------
//   Function: setCOI
//    Summary: set absolute position of the camera wrt to world/Performer
//             coordinate system.
// Parameters: 3D vector specifying the new position
//    Returns: none
//----------------------------------------------------------------------------
void dmGLPolarCamera::setCOI(float x, float y, float z)
{
   m_pos_coi[0] = x;
   m_pos_coi[1] = y;
   m_pos_coi[2] = z;
}

//----------------------------------------------------------------------------
//   Function:
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLPolarCamera::update(dmGLMouse *mouse)
{
   static int first_time = 1;
   static int last_button = 0x0;
   static int last_pos[2], delta[2] = {0,0};

   if (mouse->in_window_flag)
   {
      if (mouse->button_flags & (MOUSE_LEFT_DOWN|
                                 MOUSE_MIDDLE_DOWN|
                                 MOUSE_RIGHT_DOWN))
      {
         if (first_time)
         {
            last_pos[0] = mouse->xwin;
            last_pos[1] = mouse->ywin;
            first_time = 0;
         }

         delta[0] = mouse->xwin - last_pos[0];
         delta[1] = mouse->ywin - last_pos[1];

         last_button = mouse->button_flags;
      }
      last_pos[0] = mouse->xwin;
      last_pos[1] = mouse->ywin;
   }

   // change viewpoint if delta is non-zero
   if ((delta[0]*delta[0] + delta[1]*delta[1]) > 0.0)
      spinScene(delta, last_button);
}
