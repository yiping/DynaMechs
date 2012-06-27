#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include "dm.h"

class CubicSpline
{
	public:
		void init(Float xo,Float xo_dot, Float xf,Float xf_dot,Float T);
		void reInit(Float to, Float xf,Float xf_dot,Float T);
		double eval(Float t);
		double evalRate(Float t);
		void eval(Float t, Float & x, Float & xdot, Float & xddot);
	
	private:
		Float a[4];
		Float tMax;
};
#endif /*CUBICSPLINE_H_*/
