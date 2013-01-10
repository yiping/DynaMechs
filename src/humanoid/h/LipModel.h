/*
 *  LipModel.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/9/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ___LIP_MODEL_H___
#define ___LIP_MODEL_H___

#include "dm.h"

class LipModel {
	
public:
	
	void integrate(Float dt);
	void dynamics();
	
	Vector2F pos, vel, acc;
	Vector2F anchor;
	Float height;
	Float g;	
};



#endif