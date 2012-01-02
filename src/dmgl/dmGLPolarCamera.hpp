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
 *     File: dmGLPolarCamera.hpp
 *   Author: Scott McMillan
 *  Created: 21 March 1997
 *  Summary:
 *****************************************************************************/

#ifndef _DMGL_POLAR_CAMERA_HPP
#define _DMGL_POLAR_CAMERA_HPP

#include "dmGL.h"
#include "dmGLMouse.hpp"
#include "dmGLCamera.hpp"

class DMGL_DLL_API dmGLPolarCamera : public dmGLCamera
{
public:
   dmGLPolarCamera();
   virtual ~dmGLPolarCamera();

   // return to the initial state.
   void reset();
   void outputState();
   void setTranslationScale(float scale) {m_trans_scale = scale;}

   // Lower level functions to affect the same sorts of changes.
   void updateAzimuth(float delta_azimuth);
   void updateElevation(float delta_elevation);
   void updateRadius(float delta_radius);
   void updateCOI(float delta_coi[3]);

   void setAzimuth(float new_azimuth);
   void setElevation(float new_elevation);
   void setRadius(float new_radius);
   void setCOI(float pos_coi[3]);
   void setCOI(float x, float y, float z);

   // the update() functions set the current m_view_mat that defines the
   // camera's position with respect to the Performer base coordinate system.
   //   update() computes the matrix according to the currently defined member
   //            variables below.
   //   update(mouse *) changes the camera position and orientation
   //            according to the mouse position relative to the screen and
   //            which mouse buttons are being pressed, and computes the new
   //            matrix from these values:
   //            - the coi position can be moved in the current
   //              view plane with the middle mouse button and the distance
   //              from the center of the window using
   //            - the radius can be modified using the right mouse
   //              button and the vertical distance from the center of the
   //              window
   //            - the heading and pitch can be modified using the left mouse
   //              button and the horizontal and vertical distances from the
   //              center of the window respectively.
   //
   // NOTE that in either case applyView() from HtPfCamera must be called to
   // push the changes to the channel view matrix.
   virtual void spinScene(int delta[2], int button_flags) = 0;
   virtual void update(dmGLMouse *mouse);
   virtual void applyView() {};

protected:
   float m_pos_coi[3];  // lookat position for center of interest

   float m_radius;      // distance from center of interest
   float m_elevation, m_sin_el, m_cos_el;  // elevation angle in degrees
   float m_azimuth,   m_sin_az, m_cos_az;  // azimuth angle in degrees

   float m_trans_scale;

   float m_look_from[3], m_up[3];  // for the gluLookAt command

   dmGLPolarCamera(const dmGLPolarCamera&);  // copy constructor nono
};

#endif
