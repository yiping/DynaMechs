/*
 *  BalanceDemoStateMachine.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/27/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __JUMPING_STATE_MACHINE_H__
#define __JUMPING_STATE_MACHINE_H__

#include "CubicSplineTrajectory.h"
#include "HumanoidDataLogger.h"

class JumpingStateMachine : public HumanoidDataLogger
{
	public:
	
	JumpingStateMachine(dmArticulation * robot);
	
	enum JumpStates {
		DROP,
		BALANCE_MIDDLE,
		SQUAT,
		THRUST,
		FLIGHT,
		LAND,
		BALANCE_LEFT,
		KICK,
		NUM_JUMP_STATES
	};
	virtual void StateControl(ControlInfo & ci);
	

	private:
	
	void Drop();
	void BalanceMiddle();
	void Squat();
	void Thrust();
	void Flight();
	
	void BalanceLeft();
	void Land();
	
	void Kick();
	
	
	typedef void (JumpingStateMachine::*JumpStateFuncPtr)();
	vector<JumpStateFuncPtr> stateFunctions;
	
	CubicSplineTrajectory ComTrajectory;
	Float kpCM, kdCM, kdAM;
};

#endif