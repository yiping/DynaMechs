/*
 *  StateMachine.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/28/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include "GlobalTypes.h"



class StateMachine 
{
public:
	StateMachine(int size);
	virtual void StateControl(ControlInfo & ci) = 0;

protected:
	int state;
	double stateTime;
	bool transitionFlag;
	const int numStates;
	
	StringVector stateNames;
	
};

#endif