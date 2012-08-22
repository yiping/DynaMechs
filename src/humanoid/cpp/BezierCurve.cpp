/*
 *  BezierCurve.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 8/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "BezierCurve.h"

BezierCurve::BezierCurve(int dOrder)
{
	derivOrder = dOrder;
	if (dOrder > 0) {
		deriv = new BezierCurve(dOrder-1);
	}
}

void BezierCurve::init(WayPointVector & v, const Float t)
{
	tMax = t;
	numPts = v.size();
	int n = numPts-1;
	Ps = v;
	
	if (derivOrder > 0) {
		WayPointVector dv;
		dv.resize(numPts-1);
		for (int i=0; i<(numPts-1); i++) {
			dv[i].resize(v[0].size());
			dv[i] = n * (v[i+1] - v[i]);
		}
		deriv->init(dv, t);
	}
}

void BezierCurve::eval(Float t, VectorXF & x)
{
	if (t>tMax) {
		t = tMax;
	}
	t/=tMax;
	
	int n=numPts-1;
	
	//cout << "Evaluating curve of polynomial order " << n << " at t=" << t << endl;
	
	x.setZero(Ps[0].size());
	//cout << "x = " << endl; 
	for (int i=0; i <= n; i++) {
		x+=nchoosek(n, i) * Ps[i] * pow(1-t,n-i) * pow(t,i);
		
		//cout << nchoosek(n, i) << " (1-t)^(" << n-i << ") t^" << i << " P[" << i << "] + ";
		//cout << nchoosek(n, i) * pow(1-t,n-i) * pow(t,i) << " (" << Ps[i].transpose() << ") + ";
	}
	//cout << endl;
	//cout << "x= " << x.transpose() << endl;
}
void BezierCurve::evalRate(Float t, VectorXF & x) {
	deriv->eval(t,x);
}
void BezierCurve::eval(Float t, VectorXF & x, VectorXF & xdot, VectorXF & xddot)
{
	eval(t,x);
	deriv->eval(t,xdot);
	xdot/=tMax;
	deriv->evalRate(t, xddot);
	xddot/=pow(tMax,2);
	if (t>tMax) {
		xddot.setZero();
	}
}

 long BezierCurve::nchoosek(const long n, const long k) {
	long npk;
	if (k > (n-k)) {
		npk = factCutoff(n, k);
		for (int i=2; i<=(n-k); i++) {
			npk/=i;
		}
	}
	else {
		npk= factCutoff(n, n-k);
		for (int i=2; i<=k; i++) {
			npk/=i;
		}
	}

	return npk;
}

 long BezierCurve::factCutoff(const long n, const long k)
{
	if (n <= k ) {
		return 1;
	}
	else {
		return n * factCutoff(n-1, k);
	}
}