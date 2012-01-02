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
 *     File: dmLoadFile.h
 *   Author: Scott McMillan
 *  Created: 27 April 1997
 *  Summary:
 *****************************************************************************/

#ifndef _DM_LOAD_FILE_H
#define _DM_LOAD_FILE_H

#include "../dm/dm.h"
#include "dmu.h"
#include "../dm/dmSystem.hpp"

const char COMMENT_CHAR = '#';
const char BLOCK_BEGIN_CHAR = '{';
const char BLOCK_END_CHAR = '}';

enum actuatorTypes {NOMOTOR = 0, DCMOTOR = 1};

// functions from dmLoadFile_dm.cpp
char *getNextToken(ifstream &cfg_ptr,
                   int &line_num,
                   const char *delim = " \n\t\r");
void parseToBlockBegin(ifstream &cfg_ptr, int &line_num);
void parseToBlockEnd(ifstream &cfg_ptr, int &line_num);

dmSystem *dmLoadFile_dm203(ifstream &cfg_ptr);
dmSystem *dmLoadFile_dm21(ifstream &cfg_ptr);
dmSystem *dmLoadFile_dm30(ifstream &cfg_ptr);
dmSystem *dmLoadFile_dm40(ifstream &cfg_ptr);

#endif
