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
 *     File: dmGLCamera.hpp
 *   Author: Scott McMillan
 *  Created: 21 March 1997
 *  Summary: base camera class definition
 *****************************************************************************/

#ifndef _DMGL_CAMERA_HPP
#define _DMGL_CAMERA_HPP

#include "dmGL.h"
#include "dmGLMouse.hpp"

class DMGL_DLL_API dmGLCamera
{
public:
   dmGLCamera();
   virtual ~dmGLCamera();

   void reset();

   void setPerspective(GLfloat fov, GLfloat aspect,
                       GLfloat near_clip, GLfloat far_clip);
   void applyPerspective();

   // set/get/update view functions
   void setViewMat(GLfloat mat[4][4]);
   void getViewMat(GLfloat mat[4][4]);
   void updateViewMat(GLfloat delta_mat[4][4]);

   // functions for subclasses to build on
   virtual void update(dmGLMouse *) {};
   virtual void applyView();

protected:
   GLfloat m_fov;                     // field of view angle, degrees
   GLfloat m_aspect;
   GLfloat m_near_clip, m_far_clip;   // near and far clipping planes

   GLfloat m_view_mat[4][4];   // eyepoint position and orientation wrt scene.

   dmGLCamera(const dmGLCamera&);     // copy constructor  - invalid
};

#endif
