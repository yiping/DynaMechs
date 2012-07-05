/*
 *  BalanceDemoStateMachine.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/27/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __BALANCE_DEMO_STATE_MACHINE_H__
#define __BALANCE_DEMO_STATE_MACHINE_H__

#include "CubicSplineTrajectory.h"
#include "HumanoidDataLogger.h"

class BalanceDemoStateMachine : public HumanoidDataLogger
{
	public:
	
	BalanceDemoStateMachine(dmArticulation * robot);
	
	enum BalanceStates {
		DROP,
		BALANCE_MIDDLE,
		BALANCE_LEFT,
		BALANCE_RIGHT,
		WEIGHT_SHIFT,
		LIFT_FOOT,
		CYCLE,
		WALK_PREP,
		FIRST_STEP_LEFT,
		STEP_LEFT,
		DS_LEFT,
		STEP_RIGHT,
		DS_RIGHT,
		NUM_BALANCE_STATES
	};
	virtual void StateControl(ControlInfo & ci);
	

	private:
	
	void Drop();
	void BalanceMiddle();
	void BalanceLeft();
	void BalanceRight();
	void WeightShift();
	void LiftFoot();
	void Cycle();
	
	void WalkPrep();
	void FirstStepLeft();
	void StepLeft();
	void DoubleSupportLeft();
	void StepRight();
	void DoubleSupportRight();
	
	
	typedef void (BalanceDemoStateMachine::*BalanceStateFuncPtr)();
	vector<BalanceStateFuncPtr> stateFunctions;
	
	VectorXF aComDes;
	CubicSplineTrajectory ComTrajectory;
	Vector3F pLeftCom, pRightCom, pMiddleCom, pWalkPrep;
	VectorXF pFootEnd;
	Float kpCM, kdCM;
};

#endif