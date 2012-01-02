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
 *     File: dmGLPolarCamera_zup.cpp
 *   Author: Scott McMillan
 *  Created: 1998/05/01
 *  Summary:
 *****************************************************************************/

#include "dmGL.h"
#include "dmGLPolarCamera.hpp"
#include "dmGLPolarCamera_zup.hpp"

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmGLPolarCamera_zup::dmGLPolarCamera_zup() :
      dmGLPolarCamera()
{
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLPolarCamera_zup::spinScene(int delta[2], int button)
{
   if (button & MOUSE_LEFT_DOWN)
   {
      // mouse x position controls azimuth
      // mouse y position controls elevation
      updateAzimuth(-delta[0]*0.5);
      updateElevation(-delta[1]*0.5);
   }

   if (button & MOUSE_MIDDLE_DOWN)
   {
      static float delta_pos[3];

      delta_pos[0] =-m_trans_scale*delta[0];
      delta_pos[1] = m_trans_scale*delta[1];
      //delta_pos[2] =  0.0;

      m_pos_coi[0] += delta_pos[0]*m_cos_az
                   + delta_pos[1]*m_sin_az*m_sin_el;
                 //+ delta_pos[2]*m_sin_az*m_cos_el;

      m_pos_coi[1] += delta_pos[0]*m_sin_az
                   - delta_pos[1]*m_cos_az*m_sin_el;
                 //- delta_pos[2]*m_cos_az*m_cos_el;

      m_pos_coi[2] += delta_pos[1]*m_cos_el;
                 //- delta_pos[2]*m_sin_el;

   }

   if (button & MOUSE_RIGHT_DOWN)
   {
      updateRadius(-m_trans_scale*delta[1]);
   }
}

//----------------------------------------------------------------------------
//   Function:
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void dmGLPolarCamera_zup::applyView()
{
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   m_look_from[0] = m_pos_coi[0] + m_radius*m_sin_az*m_cos_el;
   m_look_from[1] = m_pos_coi[1] - m_radius*m_cos_az*m_cos_el;
   m_look_from[2] = m_pos_coi[2] - m_radius*m_sin_el;

   m_up[0] = m_sin_az*m_sin_el;
   m_up[1] =-m_cos_az*m_sin_el;
   m_up[2] = m_cos_el;

   gluLookAt(m_look_from[0], m_look_from[1], m_look_from[2],
             m_pos_coi[0], m_pos_coi[1], m_pos_coi[2],
             m_up[0], m_up[1], m_up[2]);

}
