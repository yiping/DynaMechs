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
 *     File: dmGLFlyCamera.hpp
 *   Author: Scott McMillan
 *  Created: 21 March 1997
 *  Summary: free-fly camera sub-class definition
 *****************************************************************************/

#ifndef _DMGL_FLY_CAMERA_HPP
#define _DMGL_FLY_CAMERA_HPP

#include "dmGL.h"
#include "dmGLCamera.hpp"
#include "dmGLMouse.hpp"

class DMGL_DLL_API dmGLFlyCamera : public dmGLCamera
{
public:
   dmGLFlyCamera();
   virtual ~dmGLFlyCamera();

   void reset();
   void outputState();
   void setTranslationScale(float scale) {m_trans_scale = scale;}

   // Lower level functions to affect camera state changes
   void updateAzimuth(float delta_azimuth);
   void updateElevation(float delta_elevation);
   void updateFlyPos(float delta_pos_ff[3]);
   void updateFlyPos(float delta_x, float delta_y, float delta_z);

   void setAzimuth(float azimuth);
   void setElevation(float elevation);
   void setFlyPos(float pos_ff[3]);
   void setFlyPos(float x, float y, float z);

   // the update() function sets the current m_view_mat that defines the
   // camera's position with respect to the GAPI's base coordinate system.
   //
   //   update(mouse *) changes the camera position and orientation
   //            according to the mouse position relative to the screen and
   //            which mouse buttons are being pressed, and computes the new
   //            matrix from these values:
   //            - the camera position can be moved right/left and up/down with
   //              the middle mouse button and the distance
   //              from the center of the window using
   //            - the position is modified forward/backward using the right
   //              mouse button and the vertical distance from the center of
   //              the window
   //            - the heading and pitch can be modified using the left mouse
   //              button and the horizontal and vertical distances from the
   //              center of the window respectively.
   //
   // NOTE that in either case applyView() from HtPfCamera must be called to
   // push the changes to the channel view matrix.
   void update(dmGLMouse *mouse);

   void applyView();

private:
   float m_pos_ff[3];                        // current camera position
   float m_look_at[3], m_up[3];
   float m_elevation, m_sin_el, m_cos_el;    // elevation angle in degrees
   float m_azimuth,   m_sin_az, m_cos_az;    // azimuth angle in degrees

   float m_trans_scale;

   dmGLFlyCamera(const dmGLFlyCamera&);      // copy constructor forbidden
};

#endif
