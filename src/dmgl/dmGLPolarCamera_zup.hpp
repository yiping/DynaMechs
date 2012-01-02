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
 *     File: dmGLPolarCamera_zup.hpp
 *   Author: Scott McMillan
 *  Created: 1998/05/01
 *  Summary:
 *****************************************************************************/

#ifndef _DMGL_POLAR_CAMERA_ZUP_HPP
#define _DMGL_POLAR_CAMERA_ZUP_HPP

#include "dmGL.h"
#include "dmGLMouse.hpp"
#include "dmGLPolarCamera.hpp"

class DMGL_DLL_API dmGLPolarCamera_zup : public dmGLPolarCamera
{
public:
   dmGLPolarCamera_zup();
   virtual ~dmGLPolarCamera_zup() {};

   virtual void spinScene(int delta[2], int button_flags);
   virtual void applyView();

private:
   dmGLPolarCamera_zup(const dmGLPolarCamera_zup &);
};

#endif
