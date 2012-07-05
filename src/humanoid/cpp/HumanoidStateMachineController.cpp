/*
 *  HumanoidStateMachineController.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 7/2/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "HumanoidStateMachineController.h"

HumanoidStateMachineController::HumanoidStateMachineController(dmArticulation * robot, int size) 
: HumanoidController(robot), StateMachine(size)  { }