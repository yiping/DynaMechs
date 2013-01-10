/*
 *  SlipModel.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/8/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "SlipModel.h"

void SlipModel::integrate(Float dt)
{
	dynamics();
	pos += vel*dt;
	vel += acc*dt;
}
void SlipModel::dynamics()
{
	Vector2F relPos = pos-anchor;
	Float length = relPos.norm();
	relPos /= length;
	
	acc = springConst * (restLength-length)*relPos / mass;
	acc(1)-=9.8;
}