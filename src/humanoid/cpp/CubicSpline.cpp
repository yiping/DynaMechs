#include "CubicSpline.h"

Float CubicSpline::eval(Float t)
{
	if(t<0)
		t=0;
	if(t>tMax)
		t=tMax;
		
	return ((a[3]*t+a[2])*t+a[1])*t+a[0];
}

Float CubicSpline::evalRate(Float t)
{
	if(t<0)
		t=0;
	if(t>tMax)
		t=tMax;
		
	return (3*a[3]*t+2*a[2])*t+a[1];
}

void CubicSpline::init(Float xo,Float xo_dot, Float xf,Float xf_dot,Float T)
{
	a[0]=xo;
	a[1]=xo_dot;
	a[2]=-(T*xf_dot + 2*xo_dot*T-3*xf+3*xo)/(T*T);
	a[3]=(-2*xf+2*xo+xo_dot*T+xf_dot*T)/(T*T*T);
	tMax=T;
}

void CubicSpline::reInit(Float to, Float xf,Float xf_dot,Float T)
{
	Float xo=eval(to);
	Float xo_dot=evalRate(to);
	init(xo,xo_dot,xf,xf_dot,T);
}

void CubicSpline::eval(Float t, Float & x, Float & xdot, Float & xddot) {
	if (t<0 || t>tMax) {
		xdot = 0;
		xddot = 0;
	}
	else {
		xdot  = (3*a[3]*t+2*a[2])*t+a[1];
		xddot = 6*a[3]*t+2*a[2];
	}
	
	if(t<0)
		t=0;
	if(t>tMax)
		t=tMax;
	
	x     = ((a[3]*t+a[2])*t+a[1])*t+a[0];
}