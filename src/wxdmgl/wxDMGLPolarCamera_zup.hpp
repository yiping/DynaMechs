/*****************************************************************************
 *     File: wxDMGLPolarCamera_zup.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#ifndef _WXDMGL_POLAR_CAMERA_ZUP_HPP
#define _WXDMGL_POLAR_CAMERA_ZUP_HPP

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"
#include "wxDMGLPolarCamera.hpp"

class WXDMGL_DLL_API wxDMGLPolarCamera_zup : public wxDMGLPolarCamera
{
public:
   wxDMGLPolarCamera_zup();
   virtual ~wxDMGLPolarCamera_zup() {};

   virtual void spinScene(int delta[2], int button_flags);
   virtual void applyView();

private:
   wxDMGLPolarCamera_zup(const wxDMGLPolarCamera_zup &);
};

#endif
