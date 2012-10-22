 /*
 *****************************************************************************
 *     File: wxDMGL.h
 *   Author:  
 *  Created: Jun 12, 2012
 *  Summary:
 *****************************************************************************/

#ifndef _WXDMGL_H
#define _WXDMGL_H


#if defined(WIN32) && defined(_DLL)
	// The next define will come from the makefile for archive objects.
	#ifdef wxDMGL_DLL_FILE
		#define WXDMGL_DLL_API __declspec(dllexport)
	#else
		#define WXDMGL_DLL_API __declspec(dllimport)
	#endif
#else
	#define WXDMGL_DLL_API
#endif


#if defined(WIN32)
#include <windows.h>
#endif

// a bunch of hacks to select standard conforming iostream stuff if available
// on the platform

#if defined(WIN32) || (defined(sgi) && defined(_STANDARD_C_PLUS_PLUS)) || (defined(__GNUC__) && (__GNUC__>=2) && (__GNUC_MINOR__>=91))
#include <iostream>
#include <iomanip>
#include <fstream>
#else
#include <iostream>
#include <iomanip>
#include <fstream>
#endif

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glut.h>
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
