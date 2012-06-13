/*****************************************************************************
 *     File: wxDMGLCamera.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary: base camera class definition
 *****************************************************************************/

#ifndef _WXDMGL_CAMERA_HPP
#define _WXDMGL_CAMERA_HPP

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"

class WXDMGL_DLL_API wxDMGLCamera
{
public:
   wxDMGLCamera();
   virtual ~wxDMGLCamera();

   void reset();

   void setPerspective(GLfloat fov, GLfloat aspect,
                       GLfloat near_clip, GLfloat far_clip);
   void applyPerspective();

   // set/get view functions
   void setViewMat(GLfloat mat[4][4]);
   void getViewMat(GLfloat mat[4][4]);

   // functions for subclasses to build on
   virtual void update(wxDMGLMouse *) {};
   virtual void applyView();

protected:
   GLfloat m_fov;                     // field of view angle, degrees
   GLfloat m_aspect;
   GLfloat m_near_clip, m_far_clip;   // near and far clipping planes

   GLfloat m_view_mat[4][4];   // eyepoint position and orientation wrt scene.

   wxDMGLCamera(const wxDMGLCamera&);     // copy constructor  - invalid ??
};

#endif
