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
 *     File: dmObject.hpp
 *   Author: Scott McMillan
 *  Summary: Toplevel mechanism graph node class
 *****************************************************************************/

#ifndef _DM_OBJECT_HPP
#define _DM_OBJECT_HPP

#include "dm.h"

//============================================================================
/**

This is supposed to be a base class for all others, but with the multiple
inheritance this gets a little complicated.  In the current incarnation, all
but the {\tt dmRigidBody} class inherits (either directly or indirectly) from
this one.  It started out as a class to hold variables needed for the 3D
graphics API to render the scene, but after reimplementing it five times (for
IRIS GL, Performer, Inventor, Xanimate, and finally OpenGL), I finally got
smart and decided I needed a general storage class to store any kind of data
with no dependencies on a specific 3D API.  The default constructor is the only
one available which initializes the storage location to NULL.

The general storage capability of this class is implemented with the {\tt
setUserData} and {\tt getUserData} functions and allows the user to associate
application specific data with the various objects created.  The former takes a
void pointer to some arbitrary data that the user wants to assign to this
object which can be retrieved with the latter.  Only one pointer to data can be
stored with this class so standard use is to allocate a struct containing all
of the necessary data and calling {\tt setUserData} with a pointer to the
struct (appropriately cast to a {\tt void *}).  {\em A possible bug (potential
memory leak) with this class is that when an object derived from this class is
destructed no user data pointed to it is deleted.  Nor is the old user data
deleted when a subsequent call {\tt setUserData} is called.  This might be
solved in the future through a reference counting mechanism, and/or providing
a delete callback function.}

As an afterthought, I have also added a capability to name each {\tt dmObject}
with a single string.  The string can be set using {\tt setName}, in which the
string is duplicated inside the class.  Calling this function with a new name,
or passing a {\tt NULL} parameter will free the space held by any previous
names.  Use {\tt getName} to get a pointer to this string.  Since the string is
duplicated (rather than just copying the pointer) no memory leak problems are
associated with this mechanism (other than the fact the {\tt getName} returns a
pointer to this memory).

 */

class DM_DLL_API dmObject
{
public:
   ///
   dmObject();
   ///
   virtual ~dmObject();

   ///
   void setUserData(void *data) {m_user_data = data;}
   ///
   void *getUserData() const {return m_user_data;}

   ///
   void setUserData2(void *data) {m_user_data_2 = data;}
   ///
   void *getUserData2() const {return m_user_data_2;}

   ///
   void setName(const char *name);
   ///
   const char *getName() const {return m_name;}

protected:
   // not implemented
   dmObject(const dmObject &);
   dmObject &operator=(const dmObject &);

protected:
   void *m_user_data;
   void *m_user_data_2;	// for rendering the second graphical model of a single link
   char *m_name;
};

#endif
