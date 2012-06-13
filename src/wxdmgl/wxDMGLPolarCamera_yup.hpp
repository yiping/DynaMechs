/*****************************************************************************
 *     File: wxDMGLPolarCamera_yup.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#ifndef _WXDMGL_POLAR_CAMERA_YUP_HPP
#define _WXDMGL_POLAR_CAMERA_YUP_HPP

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"
#include "wxDMGLPolarCamera.hpp"

class WXDMGL_DLL_API wxDMGLPolarCamera_yup : public wxDMGLPolarCamera
{
public:
   wxDMGLPolarCamera_yup();
   virtual ~wxDMGLPolarCamera_yup() {};

   virtual void spinScene(int delta[2], int button_flags);
   virtual void applyView();

private:
   wxDMGLPolarCamera_yup(const wxDMGLPolarCamera_yup &);
};

#endif
