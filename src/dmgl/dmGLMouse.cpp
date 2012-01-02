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
 *     File: dmGLMouse.cpp
 *   Author: Scott McMillan
 *  Created: 22 March 1997
 *  Summary:
 *****************************************************************************/

#include "dmGL.h"
#include "dmGLMouse.hpp"

void myHandleEntry(int state);
void myHandleMouse(int button, int state, int x, int y);
void myHandleMotion(int x, int y);
void myHandlePassiveMotion(int x, int y);

dmGLMouse *dm_mouse = NULL;

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmGLMouse *dmGLMouse::dmInitGLMouse()
{
   // create one if not yet created; otherwise, return the previous one
   if (!dm_mouse)
   {
      dm_mouse = (dmGLMouse *) malloc(sizeof(dmGLMouse));

      glutEntryFunc(myHandleEntry);
      glutMouseFunc(myHandleMouse);
      glutMotionFunc(myHandleMotion);

      glutPassiveMotionFunc(myHandlePassiveMotion);

      dm_mouse->window = glutGetWindow();
      dm_mouse->in_window_flag = false;
      dm_mouse->button_flags = 0x0;
      dm_mouse->xwin = dm_mouse->ywin = 0;
      dm_mouse->xchan = dm_mouse->ychan = 0.0;
      dm_mouse->win_size_x = dm_mouse->win_size_y = 1;
   }

   return dm_mouse;
}


//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void myHandleEntry(int state)
{
   if (state == GLUT_ENTERED)
   {
      dm_mouse->in_window_flag = true;
      //cout << "mouse entered window" << endl;
   }
   else if (state == GLUT_LEFT)
   {
      dm_mouse->in_window_flag = false;
      //cout << "mouse left window" << endl;
   }
   else
   {
      cerr << "myHandleEntry Error: got unknown state: " << state << endl;
   }
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void myHandleMouse(int button, int state, int x, int y)
{
   if (state == GLUT_DOWN)
   {
      switch (button)
      {
         case GLUT_LEFT_BUTTON:
            dm_mouse->button_flags |= MOUSE_LEFT_DOWN;
            //cout << "left button pressed" << endl;
            break;

         case GLUT_MIDDLE_BUTTON:
            dm_mouse->button_flags |= MOUSE_MIDDLE_DOWN;
            //cout << "middle button pressed" << endl;
            break;

         case GLUT_RIGHT_BUTTON:
            dm_mouse->button_flags |= MOUSE_RIGHT_DOWN;
            //cout << "right button pressed" << endl;
            break;
      }
   }
   else // GLUT_UP
   {
      switch (button)
      {
         case GLUT_LEFT_BUTTON:
            dm_mouse->button_flags &= ~MOUSE_LEFT_DOWN;
            //cout << "left button released" << endl;
            break;

         case GLUT_MIDDLE_BUTTON:
            dm_mouse->button_flags &= ~MOUSE_MIDDLE_DOWN;
            //cout << "middle button released" << endl;
            break;

         case GLUT_RIGHT_BUTTON:
            dm_mouse->button_flags &= ~MOUSE_RIGHT_DOWN;
            //cout << "right button released" << endl;
            break;
      }
   }

   dm_mouse->xwin = x;
   dm_mouse->ywin = y;
   dm_mouse->xchan = (2.0*((GLfloat) dm_mouse->xwin) - dm_mouse->win_size_x)/
                     (GLfloat) dm_mouse->win_size_x;
   dm_mouse->ychan = (2.0*((GLfloat) dm_mouse->ywin) - dm_mouse->win_size_y)/
                     (GLfloat) dm_mouse->win_size_y;

   //cout << "Pos: " << x << ", " << y << "  Button state: "
   //     << dm_mouse->button_flags << endl;
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void myHandleMotion(int x, int y)
{
   dm_mouse->xwin = x;
   dm_mouse->ywin = y;

   dm_mouse->xchan = (2.0*((GLfloat) dm_mouse->xwin) - dm_mouse->win_size_x)/
                     (GLfloat) dm_mouse->win_size_x;
   dm_mouse->ychan = (2.0*((GLfloat) dm_mouse->ywin) - dm_mouse->win_size_y)/
                     (GLfloat) dm_mouse->win_size_y;

   //cout << "Motion Pos: " << x << ", " << y << endl;
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void myHandlePassiveMotion(int x, int y)
{
   dm_mouse->xwin = x;
   dm_mouse->ywin = y;

   dm_mouse->xchan = (2.0*((GLfloat) dm_mouse->xwin) - dm_mouse->win_size_x)/
                     (GLfloat) dm_mouse->win_size_x;
   dm_mouse->ychan = (2.0*((GLfloat) dm_mouse->ywin) - dm_mouse->win_size_y)/
                     (GLfloat) dm_mouse->win_size_y;

   //cout << "Passive Motion Pos: " << x << ", " << y << endl;
}
