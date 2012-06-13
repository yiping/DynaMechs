/*****************************************************************************
 *     File: wxDMGLMouse.cpp
 *   Author: 
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#include "wxDMGL.h"
#include "wxDMGLMouse.hpp"




wxDMGLMouse::wxDMGLMouse()
{
	in_canvas_flag = false;
	button_flags = 0x0;
	xwin = ywin = 0;
	xchan = ychan = 0.0;
	win_size_x = win_size_y = 1;
}



