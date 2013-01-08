/*
 *  RunningStateMachine.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/7/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef __RUNNING_STATE_MACHINE_H__
#define __RUNNING_STATE_MACHINE_H__


#include "CubicSplineTrajectory.h"
#include "HumanoidDataLogger.h"

class RunningStateMachine : public HumanoidDataLogger
{
public:
	
	RunningStateMachine(dmArticulation * robot);
	
	enum RunStates {
		FLOATING,
		DROP,
		BALANCE_MIDDLE,
		SQUAT,
		THRUST,
		FLIGHT,
		LAND,
		BALANCE_LEFT,
		RAISE_FOOT,
		KICK,
		NUM_JUMP_STATES
	};
	
	enum FootServoType {
		GLOBAL_SERVO,
		COM_SERVO,
		HIP_SERVO
	};
	
	virtual void StateControl(ControlInfo & ci);
	
	
private:
	
	void Floating();
	void Drop();
	void BalanceMiddle();
	void Squat();
	void Thrust();
	void Flight();
	
	void BalanceLeft();
	void Land();
	
	void RaiseFoot();
	void Kick();
	
	vector<FootServoType> footServos;
	
	typedef void (RunningStateMachine::*RunStateFuncPtr)();
	vector<RunStateFuncPtr> stateFunctions;
	
	CubicSplineTrajectory ComTrajectory;
	Float kpCM, kdCM, kdAM;
};





#endif

