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
 *     File: dmLoadfile_dm.cpp
 *   Author: Scott McMillan
 *  Created: 30 September 1997
 *  Summary: load files and piece together a complete dmSystem for simulation.
 *****************************************************************************/

#include "../dm/dm.h"
#include "dmLoadFile.h"

//----------------------------------------------------------------------------
char *getNextToken(ifstream &cfg_ptr, int &line_num, const char *delim)
{
   static char line[256] = "\0";    // initialize the line to empty

   if (cfg_ptr.eof())
   {
      cerr << "dmLoadfile_dm::getNextToken error: unexpected EOF encountered"
           << endl;
      exit(1);
   }

   char *tok = strtok(NULL, delim);

   while ((tok == NULL) || (tok[0] == COMMENT_CHAR))
   {
      if (!cfg_ptr.eof())
      {
         cfg_ptr.getline(line, 256);
         line_num++;
         tok = strtok(line, delim);
      }
      else
      {
         cerr << "dmLoadfile_dm::getNextToken error: "
              << "unexpected EOF encountered" << endl;
         exit(1);
      }
   }

   return tok;
}

//----------------------------------------------------------------------------
void parseToBlockBegin(ifstream &cfg_ptr, int &line_num)
{
   char *tok = getNextToken(cfg_ptr, line_num);

   while (tok[0] != BLOCK_BEGIN_CHAR)
   {
      tok = getNextToken(cfg_ptr, line_num);
   }
}

//----------------------------------------------------------------------------
void parseToBlockEnd(ifstream &cfg_ptr, int &line_num)
{
   int indent_level = 1;
   char *tok;

   do
   {
      tok = getNextToken(cfg_ptr, line_num);

      if (tok[0] == BLOCK_END_CHAR)
      {
         indent_level--;
      }
      else if (tok[0] == BLOCK_BEGIN_CHAR)
      {
         indent_level++;
      }
   } while ((tok[0] != BLOCK_END_CHAR) || (indent_level != 0));
}

//----------------------------------------------------------------------------
dmSystem *dmuLoadFile_dm(char *filename)
{
   dmSystem *robot = NULL;

   ifstream cfg_ptr(filename);
   if (!cfg_ptr)
   {
      cerr << "Unable to open robot configuration file" << endl;
      exit(7);
   }

   char line[256] = "\0";    // initialize the line to empty
   cfg_ptr.getline(line, 256);

   // Right now I support a number of formats and use the initial comment
   // string to determine which format the remainder of the file is in.

   // WIN32 and GCC 2.91.66 (RedHat 6 - Mandrake) have a problem with
   // initializing strtok.  So the following is necessary
   strtok("","");

   if (strcmp(line, "# DynaMechs V 4.0 ascii") == 0)
   {
      robot = dmLoadFile_dm40(cfg_ptr);
   }
   else if (strcmp(line, "# DynaMechs V 3.0 ascii") == 0)
   {
      robot = dmLoadFile_dm30(cfg_ptr);
   }
   else if (strcmp(line, "# DynaMechs V 2.1 ascii") == 0)
   {
      robot = dmLoadFile_dm21(cfg_ptr);
   }
   else if (strcmp(line, "# DynaMechs V 2.0.3 ascii") == 0)
   {
      robot = dmLoadFile_dm203(cfg_ptr);
   }
   else
   {
      cerr << "dmLoadFile_dm() error: unknown format on line 1"
           << ": \"" << line << "\"" << endl;
   }

   cfg_ptr.close();
   return robot;
}
