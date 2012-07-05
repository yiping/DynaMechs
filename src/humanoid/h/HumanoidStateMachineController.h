/*
 *  HumanoidStateMachineController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 7/2/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HUMANOID_STATE_MACHINE_CONTROLLER_H__
#define __HUMANOID_STATE_MACHINE_CONTROLLER_H__

#include "StateMachine.h"
#include "HumanoidController.h"

class HumanoidStateMachineController : public StateMachine, public HumanoidController
{
public:
	HumanoidStateMachineController(dmArticulation * robot, int size);
};

#endif