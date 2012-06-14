#ifndef SPLINEMANAGER_H_
#define SPLINEMANAGER_H_

#include "CubicSpline.h"

class SplineManager
{

public:
//	SplineManager();
	SplineManager(int sign,double offset);
	void set(double pos,double opp);
	void set(double ang);
	double getSplineTime();
	void setOffset(double);
	
	void reInit(double pos,double opp,double T);
	void reInitVelocity(double v,double angle,double T);
	void reInit(double angle,double T);
	double eval(double dt);	
private:
	int oppSign;
	CubicSpline spline;
	double splineTime;
	double offset;
};


#endif /*SPLINEMANAGER_H_*/
