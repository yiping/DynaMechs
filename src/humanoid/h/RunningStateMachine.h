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
#include "SlipModel.h"
#include "LipModel.h"

class RunningStateMachine : public HumanoidDataLogger
{
public:
	
	RunningStateMachine(dmArticulation * robot);
	
	enum RunStates {
		FLOATING,
		DROP,
		PRE_HOP,
		STANCE1,
		STANCE2,
		FLIGHT1,
		FLIGHT2,
		/*BALANCE_MIDDLE,
		SQUAT,
		THRUST,
		FLIGHT,
		LAND,
		BALANCE_LEFT,
		RAISE_FOOT,
		KICK,*/
		NUM_RUN_STATES
	};
	
	enum FootServoType {
		GLOBAL_SERVO,
		COM_SERVO,
		HIP_SERVO
	};
	
	virtual void StateControl(ControlInfo & ci);
	
	Float vDesDisplay;
	Float vActDisplay;
	
private:
	
	void Floating();
	void Drop();
	
	void PreHop();
	void Stance1();
	void Stance2();
	void Flight1();
	void Flight2();
	
	/*void BalanceMiddle();
	void Squat();
	void Thrust();
	void Flight();
	
	void BalanceLeft();
	void Land();
	
	void RaiseFoot();
	void Kick();*/
	
	Float touchDownLength;
	Float touchDownAngle;
	Float legSpringConstant;
	Float maxSLIPHeight;
	Float flightTime, stanceTime;
	Float forwardVelocity;
	Float stepWidth;
	Float restWidth;
	
	vector<vector<Float> > SlipData;
	
	
	vector<FootServoType> footServos;
	
	typedef void (RunningStateMachine::*RunStateFuncPtr)();
	vector<RunStateFuncPtr> stateFunctions;
	
	CubicSplineTrajectory ComTrajectory;
	Float kpCM, kdCM, kdAM;
	
	SlipModel SLIP;
	LipModel LIP;
	
	int stanceLeg, flightLeg;
};





#endif

