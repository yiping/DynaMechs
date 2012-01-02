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
 *     File: glLoadModels.h
 *   Author: Scott McMillan
 *  Created:
 *  Summary:
 *****************************************************************************/

#ifndef _GL_LOAD_MODELS_H
#define _GL_LOAD_MODELS_H

#include "dmu.h"
#include <GL/gl.h>

DMU_DLL_API GLuint dmGLLoadFile_scm(char *filename);
DMU_DLL_API GLuint dmGLLoadFile_xan(char *filename);
DMU_DLL_API GLuint dmGLLoadFile_cmb(char *filename);

DMU_DLL_API GLuint glLoadModel(char *filename);

#endif
