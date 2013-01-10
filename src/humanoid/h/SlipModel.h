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
	
	Vector2F pos, vel, acc;
	Vector2F anchor;
	Float springConst;
	Float restLength;
	Float mass;
	
};



#endif