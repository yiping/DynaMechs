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
#include "SlipModel3D.h"
#include <cstdio>

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
	
	volatile bool pushRequest;
	volatile bool pushDirection;
	Float pushTime;
	bool pushActive;
	
private:
	
	void Floating();
	void Drop();
	
	void PreHop();
	void Stance1();
	void Stance2();
	void Flight1();
	void Flight2();
	

	typedef struct StepDataStruct {
		Float touchDownLength;
		Float touchDownAngle1;
		Float touchDownAngle2;
		Float k;
		Float k2;
		Float k1;
		Float h0;
		Float vx0;
		Float vy0;
		Float flightTime, stanceTime;
		Float footLength, CoPInitOffset, CoPVel;
	
		Vector3F pToF;
		Vector3F vToF;
		//Vector3F vInit;
		
		
		MatrixXF feedBack;
	} StepData;
	
	
	StepData thisStep;
	
	Float hipWidth;
	
	vector<StepData> SlipData;
	
	
	vector<FootServoType> footServos;
	
	typedef void (RunningStateMachine::*RunStateFuncPtr)();
	vector<RunStateFuncPtr> stateFunctions;
	
	CubicSplineTrajectory ComTrajectory;
	Float kpCM, kdCM, kdAM;
	
	
	SlipModel3D SLIP;
	//SlipModel SLIP, prevSLIP;
	//bool transitionStep;
	//LipModel LIP;
	
	int stanceLeg, flightLeg;
	
	
	//
	Float prevTouchDownLength;
	Float prevTouchDownAngle;
	Float prevLegSpringConstant;
	Float prevMaxSLIPHeight;
	Float prevFlightTime, prevStanceTime;
	Float prevForwardVelocity;
	
	bool transitionController;
	bool velocityTest;
	
	FILE * pTOFData;
	
	VectorXF footAngles;
	Matrix3F initRDesFoot;
	
	vector<StepData> ruleBase; 
	
 
	
};





#endif

