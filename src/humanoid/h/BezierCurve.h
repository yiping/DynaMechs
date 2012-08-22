/*
 *  BezierCurve.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 8/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __BEZIER_CURVE_H__
#define __BEZIER_CURVE_H__

#include "dm.h"

typedef vector<VectorXF> WayPointVector;

class BezierCurve
{
public:
	BezierCurve(int dOrder = 2);
	void init(WayPointVector & v, const Float t);
	void eval(Float t, VectorXF & x);
	void evalRate(Float t, VectorXF & x);
	void eval(Float t, VectorXF & x, VectorXF & xdot, VectorXF & xddot);
private:
	static long nchoosek(const long n, const long k);
	//static long fact(const long n);
	static long factCutoff(const long n, const long k);
	
	Float tMax;
	int numPts;
	int derivOrder;
	WayPointVector Ps;
	BezierCurve * deriv;
};


#endif

