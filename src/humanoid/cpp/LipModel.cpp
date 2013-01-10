/*
 *  LipModel.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/9/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "LipModel.h"

void LipModel::integrate(Float dt)
{
	dynamics();
	pos += vel*dt;
	vel += acc*dt;
}
void LipModel::dynamics()
{	
	acc = g/height*(pos - anchor);
}