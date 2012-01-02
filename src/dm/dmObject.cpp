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
 *     File: dmObject.cpp
 *   Author: Scott McMillan
 *  Summary: used to name and attach user data to all classes
 *****************************************************************************/

#include "dm.h"
#include "dmObject.hpp"

//============================================================================
// class dmObject
//============================================================================

//----------------------------------------------------------------------------
dmObject::dmObject()
      : m_user_data(NULL),
        m_name(NULL)
{
}

//----------------------------------------------------------------------------
dmObject::~dmObject()
{
   if (m_name)
   {
      free(m_name);
   }

   /* FIXME - without reference counting and not knowing which mechanism
              (malloc or new) was used I cannot dealloc the user data. */
}

//----------------------------------------------------------------------------
void dmObject::setName(const char *name)
{
   // free any previously existing name
   if (m_name)
   {
      free(m_name);
      m_name = NULL;
   }

   // if parameter is non-NULL copy new name
   if (name)
   {
      m_name = (char *) malloc(strlen(name)+1);
      memcpy(m_name, name, strlen(name)+1);
   }
}
