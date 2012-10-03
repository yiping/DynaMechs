

#ifndef __STATE_MACHINE_CONTROLLER_A_H__
#define __STATE_MACHINE_CONTROLLER_A_H__

#include "control_globals.h"
#include "dmArticulation.hpp"
#include "TaskSpaceControllerL.h"
#include "globalFunctions.h"

class StateMachineControllerA : public TaskSpaceControllerL
{
public:	
	StateMachineControllerA(dmArticulation * robot);

	void StateControl();
	int state;
	enum StatesA 
	{
		DROP,
		BALANCE_MIDDLE,
		NUM_STATES
	};
	void Drop();
	void BalanceMiddle();

	Float controlTime;
	Float sm_dt;

protected:
	double stateTime;
	bool transitionFlag;
	StringVector stateNames;

	typedef void (StateMachineControllerA::*StateFuncPtr)();	// function pointer
	vector<StateFuncPtr> stateFunctions;

private:
	

};
#endif
