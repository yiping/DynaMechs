/*
 *  CubicSplineTrajectory.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/13/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "CubicSplineTrajectory.h"


void CubicSplineTrajectory::eval(Float t, VectorXF & x)
{
	if(t<0)
		t=0;
	if(t>tMax)
		t=tMax;
	
	for (int i=0; i<size; i++) {
		x(i) = splines[i].eval(t);
	}
}

void CubicSplineTrajectory::evalRate(Float t, VectorXF & xdot)
{
	if(t<0)
		t=0;
	if(t>tMax)
		t=tMax;
	
	for (int i=0; i<size; i++) {
		xdot(i) = splines[i].evalRate(t);
	}
}

void CubicSplineTrajectory::setSize(int s) {
	size = s;
	splines.resize(s);
}

void CubicSplineTrajectory::init(const VectorXF& xo,const VectorXF & xo_dot, const VectorXF & xf,const VectorXF & xf_dot,Float T) {
	for (int i=0; i<size; i++) {
		splines[i].init(xo(i), xo_dot(i), xf(i), xf_dot(i),T);
	}
}

void CubicSplineTrajectory::reInit(const Float to, const VectorXF& xf,const VectorXF& xf_dot,Float T) {
	for (int i=0; i<size; i++) {
		splines[i].reInit(to, xf(i), xf_dot(i),T);
	}
}

void CubicSplineTrajectory::eval(Float t,  VectorXF & x,  VectorXF & xdot,  VectorXF & xddot) {
	for (int i=0; i<size; i++) {
		splines[i].eval(t,x(i), xdot(i), xddot(i));
	}	
}