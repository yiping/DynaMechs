#include "SplineManager.h"

/*SplineManager::SplineManager()
{
	SplineManager(1,0);	
}*/
SplineManager::SplineManager(int sign,double off)
{
	if(sign>0)
	{
		oppSign=1;	
	}
	else
	{
		oppSign=-1;
	}
	offset=off;
	splineTime=0;
}
void SplineManager::set(double pos,double opp)
{
	set(pos+oppSign*opp+offset);
}
void SplineManager::setOffset(double o)
{
	offset=o;	
}
void SplineManager::set(double ang)
{
	spline.init(ang,0,ang,0,1);	
}
double SplineManager::eval(double dt)
{
	double val = spline.eval(splineTime);
	splineTime+=dt;	
	return val;
}
void SplineManager::reInit(double pos,double opp,double T)
{
	reInit(pos+oppSign*opp+offset,T);	
}
void SplineManager::reInit(double angle,double T)
{
	spline.reInit(splineTime,angle,0,T);
	splineTime=0;
}
void SplineManager::reInitVelocity(double v,double angle,double T)
{
	double xo=spline.eval(splineTime);
	spline.init(xo,v,angle,0,T);
	splineTime=0;
}
double SplineManager::getSplineTime()
{
	return splineTime;	
}
