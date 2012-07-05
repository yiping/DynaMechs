/*
 *  StateMachine.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/28/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "StateMachine.h"

StateMachine::StateMachine(int size) :  numStates(size)
{
	stateNames.resize(numStates);
	state = 0;
	transitionFlag = true;
}