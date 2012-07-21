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
 *     File: dmLoadFile.cpp
 *   Author: Scott McMillan
 *  Created: 30 September 1997
 *  Summary: Miscellaneous config file parsing functions
 *****************************************************************************/

#include "../dm/dm.h"
#include "dmLoadFile.h"

//----------------------------------------------------------------------------
//    Summary: scan file until first double quote enclosed string is found,
//             and return this string.
// Parameters: cfg_ptr - ifstream reference to file to be scanned
//             filename - string scanned between double quotes (on return)
//    Returns: true if a double-quote enclosed string is found, else false
//----------------------------------------------------------------------------
bool readFilename(ifstream &cfg_ptr, char *filename)
{
   if (cfg_ptr.getline(filename, FILENAME_SIZE, '\042')) {
      if (cfg_ptr.getline(filename, FILENAME_SIZE, '\042')) {
         if (strlen(filename) > 0) {
            return true;
         }
      }
   }
   return false;
}

//----------------------------------------------------------------------------
//    Summary: scan a file from the current position until a particular string
//             is found.  It will exit (not return) if the EOF is encountered
//             before the search string is found.
// Parameters: cfg_ptr - ifstream reference to file being scanned
//             label - string to be searched for
//    Returns: none
//----------------------------------------------------------------------------
void readConfigParameterLabel(ifstream &cfg_ptr, const char *label)
{
   register int i;
   register unsigned char c = '\0';
   char line[80];

   bool stop = false;

   // Strip off blank lines and comment lines.
   while (!stop && ((i = cfg_ptr.get()) != EOF))
   {
      c = (unsigned char) i;

      if ((c == '\n') || (c == COMMENT_CHAR))
      {
         while ((c != '\n') && ((i = cfg_ptr.get()) != EOF))
         {
            c = (unsigned char) i;
         }
      }
      else
      {
         stop = true;
      }
   }

   if (!stop) {
      cerr << "Error: Parameters file EOF encountered before " << label <<
              " found.\n";
      exit(4);
   }

   cfg_ptr.putback(c);

   // Read in the strings until label is found or EOF encountered.
   while ((cfg_ptr >> line))
   {
      if ((line[0] != COMMENT_CHAR) && (line[0] != '\n'))
      {
         if (strncmp(line, label, strlen(label)) == 0)
         {
            return;
         }
         else
         {
            cerr << "Warning: skipped unrecognized parameter: "
                 << line << endl;
            cerr << "   Wanted: " << label << endl;
         }
      }
      c = '\0';
      while ((c != '\n') && ((i = cfg_ptr.get()) != EOF))
      {
         c =  (unsigned char) i;
      }
   }

   cerr << "Error: Parameters file EOF encountered before " << label
        << " found.\n";
   exit(4);
}



//-------------------------------------------------------------------

bool readConfigParameterLabelNonRecursive(ifstream &cfg_ptr, const char *label)
{

	register int i;
	register unsigned char c = '\0';
	char line[80];
	bool onceMore = true;

	while (onceMore )
	{
		cfg_ptr >> line; // will automatically skip all the spaces and enters
		//cout<<"[unprocessed line] "<<line<<endl;

		if ((line[0] != COMMENT_CHAR) && (line[0] != '\n'))
		{		
			//cout<<"[meaningful line] "<<line<<endl;
			if (strncmp(line, label, strlen(label)) == 0)
			{
				cout<<"found ["<<label<<"] label"<<endl;
				return true;
			}
			else
			{
				cerr << "Tried and didn't find ["<< label <<"] label"<< endl;
				//cerr << "But, buddy, no worries, we can proceed without ["<<label<<"]"<<endl;
				cerr << "Proceed without ["<<label<<"]"<<endl;	
				onceMore = false;
				// wait, before you quit searching, put that label back in the stream!
				// cout<<"rewinding ifstream ..."<<endl;
				int sl = strlen(line);
				//cout<<sl<<endl;
				for (int j=0; j<strlen(line);j++)
				{
					cfg_ptr.putback(line[sl-1-j]);
					//cout<<line[sl-1-j];
				}
				cout<<endl<<"-------"<<endl;
				return false;
			}
		}
		else //filter all the comments
		{
			for (int k =0;k<strlen(line);k++)
			{
				c = line[k];
				if (c == '\n')
					cout<<"[ENTER]"<<endl;
				else
					cout<<c;
			}
			while ((c != '\n') && ((i = cfg_ptr.get()) != EOF))
			{
				c =  (unsigned char) i;
				if (c ==' ')
					cout<<"[_]";
				else if (c=='\n')
					cout<<"[ENTER] "<<endl;
				else
					cout<<c;
			}
			// what you just read in are useless commments, so try one more time;
			onceMore = true;
		}
	}

}

