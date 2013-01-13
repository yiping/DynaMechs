/*
 *  SlipModel.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/8/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef ___SLIP_MODEL_H___
#define ___SLIP_MODEL_H___

#include "dm.h"


class SlipModel {
	
public:
	
	void integrate(Float dt);
	void dynamics();
	void findParams();
	
	void simulatePeriod(const VectorXF& ps);
	void evalResidual(const VectorXF& ps, VectorXF &fs, Float & o);
	void evalJac(const VectorXF& ps, const VectorXF & f0, const Float & obj0, MatrixXF & J, VectorXF &g);
	
	void armijoStep(VectorXF&x,const VectorXF& d, Float & obj, VectorXF & f, const VectorXF& g);
	
	void optimize();
	
	Vector2F pos, vel, acc;
	Vector2F anchor;
	Float length;
	Float springConst;
	Float restLength;
	Float mass;
	Float time;
	Float tContact,tFlight;
	
	
	Float tContactDes;
	Float tFlightDes;
	Float vxFlightDes;
	Float vyFlightDes;
	
	VectorXF params;
	
};



#endif