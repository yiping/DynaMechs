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
 *     File: dmGLFlyCamera.cpp
 *   Author: Scott McMillan
 *  Created:
 *  Summary:
 *****************************************************************************/

#include "dmGLFlyCamera.hpp"

const float DEGTORAD = (float)(M_PI/180.0);

//============================================================================
// class dmGLFlyCamera : public dmGLCamera
//============================================================================

//----------------------------------------------------------------------------
//    Summary: constructor
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmGLFlyCamera::dmGLFlyCamera() : dmGLCamera()
{
   reset();
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmGLFlyCamera::~dmGLFlyCamera()
{
#ifdef DEBUG
   cerr << "destructing dmGLFlyCamera" << endl;
#endif
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::reset()
{
   dmGLCamera::reset();

   m_pos_ff[0] = m_pos_ff[1] = m_pos_ff[2] = 0.0;
   setAzimuth(0.0);
   setElevation(0.0);
   m_trans_scale = 1.0;
}

//----------------------------------------------------------------------------
//   Function:
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::outputState()
{
   cerr << "dmGLFlyCamera state:" << endl;
   cerr << "Azimuth:    " << m_azimuth << endl;
   cerr << "Elevation:  " << m_elevation << endl;
   cerr << "Camera pos: " << m_pos_ff[0] << ", "
                          << m_pos_ff[1] << ", "
                          << m_pos_ff[2] << endl;
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::updateAzimuth(float delta_azimuth)
{
   setAzimuth(m_azimuth + delta_azimuth);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::setAzimuth(float new_azimuth)
{
   m_azimuth = new_azimuth;

   // allow it to wrap around
   while (m_azimuth > 180.0)
   {
      m_azimuth -= 360.0;
   }
   while (m_azimuth < -180.0)
   {
      m_azimuth += 360.0;
   }

   double tmp = m_azimuth*DEGTORAD;
   m_sin_az = sin(tmp);
   m_cos_az = cos(tmp);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::updateElevation(float delta_elevation)
{
   setElevation(m_elevation + delta_elevation);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::setElevation(float elevation)
{
   m_elevation = elevation;

   // allow it to wrap around
   while (m_elevation > 180.0)
   {
      m_elevation -= 360.0;
   }
   while (m_elevation < -180.0)
   {
      m_elevation += 360.0;
   }

   double tmp = m_elevation*DEGTORAD;
   m_sin_el = sin(tmp);
   m_cos_el = cos(tmp);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::updateFlyPos(float delta_pos[3])
{
   m_pos_ff[0] += delta_pos[0];
   m_pos_ff[1] += delta_pos[1];
   m_pos_ff[2] += delta_pos[2];
}

void dmGLFlyCamera::updateFlyPos(float delta_x, float delta_y, float delta_z)
{
   m_pos_ff[0] += delta_x;
   m_pos_ff[1] += delta_y;
   m_pos_ff[2] += delta_z;
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::update(dmGLMouse *mouse)
{
   static float delta_pos[3];

   if (mouse->in_window_flag)
   {
      if (mouse->button_flags & (MOUSE_LEFT_DOWN|
                                 MOUSE_MIDDLE_DOWN|
                                 MOUSE_RIGHT_DOWN))
      {
         if (mouse->button_flags & MOUSE_LEFT_DOWN)
         {
            // mouse x position controls azimuth
            // mouse y position controls elevation
            updateAzimuth(2.0*mouse->xchan);
            updateElevation(2.0*mouse->ychan);
         }

         delta_pos[0] = delta_pos[1] = delta_pos[2] = 0.0;

         if (mouse->button_flags & MOUSE_MIDDLE_DOWN)
         {
            delta_pos[0] = 2.0*m_trans_scale*mouse->xchan;
            delta_pos[2] = 2.0*m_trans_scale*mouse->ychan;
         }

         if (mouse->button_flags & MOUSE_RIGHT_DOWN)
         {
            delta_pos[1] = -2.0*fabs(mouse->ychan)*mouse->ychan;
         }

         m_pos_ff[0] += delta_pos[0]*m_cos_az
                      - delta_pos[1]*m_sin_az*m_cos_el
                      + delta_pos[2]*m_sin_az*m_sin_el;

         m_pos_ff[1] += delta_pos[0]*m_sin_az
                      + delta_pos[1]*m_cos_az*m_cos_el
                      - delta_pos[2]*m_cos_az*m_sin_el;

         m_pos_ff[2] += delta_pos[1]*m_sin_el
                      + delta_pos[2]*m_cos_el;
      }
   }
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLFlyCamera::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   m_look_at[0] =  m_sin_az*m_cos_el;
   m_look_at[1] =  m_sin_el;
   m_look_at[2] = -m_cos_az*m_cos_el;

   m_up[0] = -m_cos_az;
   m_up[1] = 0.0;
   m_up[2] = -m_sin_az;

   gluLookAt(m_pos_ff[0],
             m_pos_ff[1],
             m_pos_ff[2],
             m_pos_ff[0] + m_look_at[0],
             m_pos_ff[1] + m_look_at[1],
             m_pos_ff[2] + m_look_at[2],
             m_up[0], m_up[1], m_up[2]);
}
