// HumanoidControllerStateMachine.cpp
// Nov 28, 2012
// YL

#include "HumanoidControllerStateMachine.h"

HumanoidControllerStateMachine::HumanoidControllerStateMachine(dmArticulation * robot, int size) 
: HumanoidController(robot), numStates(size)
{ 
	stateNames.resize(numStates);
	state = 0;
	transitionFlag = true;
}
