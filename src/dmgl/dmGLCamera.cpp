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
 *     File: dmGLCamera.cpp
 *   Author: Scott McMillan
 *  Created: 22 March 1997
 *  Summary: camera base class for viewing objects
 *****************************************************************************/

#include "dmGLCamera.hpp"

//============================================================================
// class dmGLCamera
//============================================================================

//----------------------------------------------------------------------------
//   Function: dmGLCamera Constructor
//    Summary: This will initialize a camera that will view a scene
//             constructed in a Performer scene graph
// Parameters: Performer channel this camera will be responsible for
//    Returns: none
//----------------------------------------------------------------------------
dmGLCamera::dmGLCamera()
{
   reset();
}

//----------------------------------------------------------------------------
//   Function: dmGLCamera Destructor
//    Summary:
// Parameters:
//    Returns: none
//----------------------------------------------------------------------------
dmGLCamera::~dmGLCamera()
{
#ifdef DEBUG
   cerr << "destructing dmGLCamera" << endl;
#endif
}

//----------------------------------------------------------------------------
//   Function: reset
//    Summary: Reset most of the camera position, orientation, and FOV
//             parameters.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmGLCamera::reset()
{
   for (int i=0; i<4; i++)
   {
      for (int j=0; j<4; j++)
      {
         m_view_mat[i][j] = 0.0;
      }
      m_view_mat[i][i] = 1.0;
   }

   setPerspective(45.0, 1.0, 1.0, 100.0);
}


//----------------------------------------------------------------------------
//   Function: setPerspective
//    Summary: set field of view, and keep it in the
//             range [22.5, 120]
// Parameters: new field of view, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void dmGLCamera::setPerspective(GLfloat fov, GLfloat aspect,
                                  GLfloat near_clip, GLfloat far_clip)
{
   m_fov = fov;
   m_aspect = aspect;
   m_near_clip = near_clip;
   m_far_clip = far_clip;

   applyPerspective();
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLCamera::applyPerspective()
{
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(m_fov, m_aspect, m_near_clip, m_far_clip);
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLCamera::setViewMat(GLfloat mat[4][4])
{
   for (int i=0; i<4; i++)
   {
      for (int j=0; j<4; j++)
      {
         m_view_mat[i][j] = mat[i][j];
      }
   }
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLCamera::getViewMat(GLfloat mat[4][4])
{
   for (int i=0; i<4; i++)
   {
      for (int j=0; j<4; j++)
      {
         mat[i][j] = m_view_mat[i][j];
      }
   }
}

//----------------------------------------------------------------------------
//   Function: updateView
//    Summary: adds an arbitrary position and orientation change to the
//             existing m_view_mat.
// Parameters: The change is specified either by another
//             pfMatrix, pfCoord, pos/hpr vector pair, or six floats
//    Returns: none
//----------------------------------------------------------------------------
void dmGLCamera::updateViewMat(GLfloat (*)[4])  //delta_mat[4][4])
{
   cerr << "dmGLCamera::updateViewMat: not implemented" << endl;

   //m_view_mat.preMult(delta_mat);
}

//----------------------------------------------------------------------------
//   Function: applyView
//    Summary: Transfer the camera position and orientation data stored in the
//             class to the channel.  The new view will take affect in the
//             Performer simulation only after this function is called
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmGLCamera::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadMatrixf((GLfloat *)m_view_mat);
}
