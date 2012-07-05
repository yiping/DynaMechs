/*
 *  CubicSplineTrajectory.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/13/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __CUBIC_SPLINE_TRAJECTORY_H__
#define __CUBIC_SPLINE_TRAJECTORY_H__

#include "CubicSpline.h"
#include "dm.h"
#include <vector>
using namespace std;


typedef vector<CubicSpline> SplineVector;

class CubicSplineTrajectory
{
public:
	CubicSplineTrajectory(int s);
	void init(const VectorXF & xo, const VectorXF & xo_dot, const VectorXF & xf,const VectorXF & xf_dot, const Float T);
	void reInit(const Float to, const VectorXF & xf, const VectorXF & xf_dot,const Float T);
	void eval(Float t, VectorXF & x);
	void evalRate(Float t, VectorXF & x);
	void eval(Float t, VectorXF & x, VectorXF & xdot, VectorXF & xddot);
	void setSize(int);
	
private:
	SplineVector splines;
	Float tMax;
	int size;
};

#endif