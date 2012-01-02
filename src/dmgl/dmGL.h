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
 *     File: dmGL.h
 *   Author: Scott McMillan
 *  Created:
 *  Summary:
 *****************************************************************************/

#ifndef _DM_GL_H
#define _DM_GL_H

#if defined(WIN32) && defined(_DLL)
// The next define will come from the makefile for archive objects.
#ifdef dmGL_DLL_FILE
#define DMGL_DLL_API __declspec(dllexport)
#else
#define DMGL_DLL_API __declspec(dllimport)
#endif
#else
#define DMGL_DLL_API
#endif

#if defined(WIN32)
#include <windows.h>
#endif

// a bunch of hacks to select standard conforming iostream stuff if available
// on the platform

#if defined(WIN32) || (defined(sgi) && defined(_STANDARD_C_PLUS_PLUS)) || (defined(__GNUC__) && (__GNUC__>=2) && (__GNUC_MINOR__>=91))
#include <iostream>
#include <iomanip>
#else
#include <iostream>
#include <iomanip>
#endif

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif



#include <math.h>
#include <stdlib.h>

#ifndef NULL
#define NULL 0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Use the std namespace. To do this we must first guarantee that it exists.
#if defined(__sgi) || defined(__WIN32_) || defined(WIN32)
namespace std {}
using namespace std;
#endif

#endif

using namespace std;
