// HumanoidControllerStateMachine.h
// Nov 27, 2012
// YL

#ifndef __HUMANOID_CONTROLLER_STATE_MACHINE_H__
#define __HUMANOID_CONTROLLER_STATE_MACHINE_H__


#include "HumanoidController.h"

class HumanoidControllerStateMachine : public HumanoidController
{
public:
	HumanoidControllerStateMachine(dmArticulation * robot, int size);
	virtual void StateControl() = 0;
	int state;
protected:
	
	double stateTime;
	bool transitionFlag;
	const int numStates;
	
	StringVector stateNames;

};

#endif
