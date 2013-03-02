/*
 *  3DSlipModel.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 2/18/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "SlipModel3D.h"
#include <Eigen/Dense>
#include "dmTime.h"
#include "GlobalFunctions.h"

void SlipModel3D::integrate(Float dt)
{
	dynamics();
	pos += vel*dt;
	vel += acc*dt;
	anchor+= anchorVel*dt;
	time +=dt;
}
void SlipModel3D::dynamics()
{
	Vector3F relPos = pos-anchor;
	length = relPos.norm();
	relPos /= length;
	
	acc = springConst * (restLength-length)*relPos / mass;
	
	expandRate = relPos.dot(vel);
	
	acc(2)-=9.8;
}