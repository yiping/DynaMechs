/*****************************************************************************
 * Copyright 1999, Scott McMillan
 *****************************************************************************
 *     File:
 *   Author: Scott McMillan
 *  Created: 23 May 1999
 *  Summary: defining decl_spec's for NT port
 *****************************************************************************/

#ifndef __GAIT_HPP
#define __GAIT_HPP

#if defined(WIN32) && defined(_DLL)
// The next define will come from the makefile for archive objects.
#ifdef aquarobot_DLL_FILE
#define GAIT_DLL_API __declspec(dllexport)
#else
#define GAIT_DLL_API __declspec(dllimport)
#endif
#else
#define GAIT_DLL_API
#endif

#if defined(WIN32)
#pragma warning (disable : 4786 4251 4661)
#include <windows.h>
#endif

#if defined(WIN32) || (defined(sgi) && defined(_STANDARD_C_PLUS_PLUS)) || (defined(__GNUC__) && (__GNUC__>=2) && (__GNUC_MINOR__>=91))
#include <iostream>
#include <iomanip>
#include <fstream>
#else
#include <iostream>
#include <iomanip>
#include <fstream>
#endif

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// Use the std namespace. To do this we must first guarantee that it exists.
#if defined(__sgi) || defined(__WIN32_) || defined(WIN32)
namespace std {}
using namespace std;
#endif

#endif
