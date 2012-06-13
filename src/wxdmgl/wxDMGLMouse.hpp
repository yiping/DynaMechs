/*****************************************************************************
 *     File: wxDMGLMouse.hpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#ifndef _WXDMGL_MOUSE_HPP
#define _WXDMGL_MOUSE_HPP

#include "wxDMGL.h"

//----------------------------------------------------------------------------

enum {MOUSE_L_DN   = 0x01,
      MOUSE_M_DN = 0x02,
      MOUSE_R_DN  = 0x04};

//----------------------------------------------------------------------------

class WXDMGL_DLL_API wxDMGLMouse
{
public:
	wxDMGLMouse();
	int   win_size_x;
	int   win_size_y;

	bool  in_canvas_flag; // true if mouse is in window
	int   button_flags;   // bitmasks 4,2,1 for right,middle,left buttons
	int   xwin, ywin;     // position within window
	float xchan, ychan;   // normalized window position   
};

#endif
