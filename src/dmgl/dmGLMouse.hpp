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
 *     File: dmGLMouse.hpp
 *   Author: Scott McMillan
 *  Created: 22 March 1997
 *  Summary:
 *****************************************************************************/

#ifndef _DMGL_MOUSE_HPP
#define _DMGL_MOUSE_HPP

#include "dmGL.h"

//----------------------------------------------------------------------------

enum {MOUSE_LEFT_DOWN   = 0x01,
      MOUSE_MIDDLE_DOWN = 0x02,
      MOUSE_RIGHT_DOWN  = 0x04};

//----------------------------------------------------------------------------

struct DMGL_DLL_API dmGLMouse
{
   int   window;         // window for which this mouse class has been init'ed
   int   win_size_x;
   int   win_size_y;

   bool  in_window_flag; // true if mouse is in window
   int   button_flags;   // bitmasks 4,2,1 for right,middle,left buttons
   int   xwin, ywin;     // position within window
   float xchan, ychan;   // normalized window position

   static dmGLMouse *dmInitGLMouse();
};

#endif
