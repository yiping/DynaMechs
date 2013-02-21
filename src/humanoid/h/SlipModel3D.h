/*
 *  3DSlipModel.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 2/18/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "dm.h"


class SlipModel3D {
	
public:
	
	void integrate(Float dt);
	void dynamics();
	
	
	Vector3F pos, vel, acc;
	Vector3F anchor, anchorVel;
	
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
