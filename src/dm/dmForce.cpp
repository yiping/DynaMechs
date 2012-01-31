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
 *     File: dmForce.cpp
 *   Author: Scott McMillan
 *  Summary: The base force "widget" class really only defines the interface
 *****************************************************************************/

#include "dm.h"
#include "dmForce.hpp"

// static variable...all force objects share this flag.
// NOT thread safe
bool dmForce::m_boundary_flag = false;

//----------------------------------------------------------------------------
dmForce::dmForce() : dmObject()
{
}

//----------------------------------------------------------------------------
dmForce::~dmForce()
{
}

void dmForce::computeForce(const dmRNEAStruct &val, SpatialVector force)
{
	//dummy
}
