/*****************************************************************************
 *     File: wxDMGLCamera.cpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary: camera base class for viewing objects
 *****************************************************************************/

#include "wxDMGLCamera.hpp"

//============================================================================
// class wxDMGLCamera
//============================================================================

wxDMGLCamera::wxDMGLCamera()
{
   reset();
}

wxDMGLCamera::~wxDMGLCamera()
{
#ifdef DEBUG
   cerr << "destructing wxDMGLCamera" << endl;
#endif
}

//----------------------------------------------------------------------------
//   Function: reset
//    Summary: Reset most of the camera position, orientation, and FOV 
//             (field of view) parameters.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLCamera::reset()
{
   for (int i=0; i<4; i++)
   {
      for (int j=0; j<4; j++)
      {
         m_view_mat[i][j] = 0.0;
      }
      m_view_mat[i][i] = 1.0;
   }
   // Tip: Don't do anything OpenGL in contstructors!
   //setPerspective(45.0, 1.0, 1.0, 100.0);
}


//----------------------------------------------------------------------------
//   Function: setPerspective
//    Summary: set field of view, and keep it in the
//             range [22.5, 120]
// Parameters: new field of view, in degrees
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLCamera::setPerspective(GLfloat fov, GLfloat aspect,
                                  GLfloat near_clip, GLfloat far_clip)
{
   m_fov = fov;
   m_aspect = aspect;
   m_near_clip = near_clip;
   m_far_clip = far_clip;

   applyPerspective();
}


void wxDMGLCamera::applyPerspective()
{
   glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
   //set up perspective projection matrix
   gluPerspective(m_fov, m_aspect, m_near_clip, m_far_clip);
}


void wxDMGLCamera::setViewMat(GLfloat mat[4][4])
{
   for (int i=0; i<4; i++)
   {
      for (int j=0; j<4; j++)
      {
         m_view_mat[i][j] = mat[i][j];
      }
   }
}


void wxDMGLCamera::getViewMat(GLfloat mat[4][4])
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
//   Function: applyView
//    Summary: Mount the camera at the designated position and orientation. 
//             The new view will take affect in simulation only after this 
//             function is called
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void wxDMGLCamera::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadMatrixf((GLfloat *)m_view_mat);
}
