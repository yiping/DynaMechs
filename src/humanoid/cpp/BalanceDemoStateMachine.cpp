/*
 *  BalanceDemoStateMachine.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/27/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"
#include "BalanceDemoStateMachine.h"

BalanceDemoStateMachine::BalanceDemoStateMachine(dmArticulation * robot) 
	: HumanoidDataLogger(robot, NUM_BALANCE_STATES), ComTrajectory(3)
{
	stateFunctions.resize(numStates);
	
	stateNames[DROP] = "Drop";
	stateFunctions[DROP] = &BalanceDemoStateMachine::Drop;
	
	stateNames[BALANCE_LEFT] = "Balance Left";
	stateFunctions[BALANCE_LEFT] = &BalanceDemoStateMachine::BalanceLeft;
	
	stateNames[BALANCE_RIGHT] = "Balance Right";
	stateFunctions[BALANCE_RIGHT] = &BalanceDemoStateMachine::BalanceRight;
	
	stateNames[BALANCE_MIDDLE] = "Balance Middle";
	stateFunctions[BALANCE_MIDDLE] = &BalanceDemoStateMachine::BalanceMiddle;
	
	stateNames[WEIGHT_SHIFT] = "Weight Shift";
	stateFunctions[WEIGHT_SHIFT] = &BalanceDemoStateMachine::WeightShift;
	
	stateNames[LIFT_FOOT] = "Lift Foot";
	stateFunctions[LIFT_FOOT] = &BalanceDemoStateMachine::LiftFoot;
	
	stateNames[CYCLE] = "Cycle";
	stateFunctions[CYCLE] = &BalanceDemoStateMachine::Cycle;
	
	
	stateNames[WALK_PREP] = "Walk Prep";
	stateFunctions[WALK_PREP] = &BalanceDemoStateMachine::WalkPrep;
	
	stateNames[FIRST_STEP_LEFT] = "First Step Left";
	stateFunctions[FIRST_STEP_LEFT] = &BalanceDemoStateMachine::FirstStepLeft;
	
	
	stateNames[STEP_LEFT] = "Step Left";
	stateFunctions[STEP_LEFT] = &BalanceDemoStateMachine::StepLeft;
	
	stateNames[DS_LEFT] = "DS Left";
	stateFunctions[DS_LEFT] = &BalanceDemoStateMachine::DoubleSupportLeft;
	
	stateNames[STEP_RIGHT] = "Step Right";
	stateFunctions[STEP_RIGHT] = &BalanceDemoStateMachine::StepRight;
	
	stateNames[DS_RIGHT] = "DS Right";
	stateFunctions[DS_RIGHT] = &BalanceDemoStateMachine::DoubleSupportRight;
	
	stateNames[FALL] = "Fall";
	stateFunctions[FALL] = &BalanceDemoStateMachine::Fall;
	
	stateNames[DYN_STEP_LEFT] = "Dyn Step Left";
	stateFunctions[DYN_STEP_LEFT] = &BalanceDemoStateMachine::DynamicStepLeft;
	
	stateNames[DYN_SUPPORT_LEFT] = "Dyn Sup Left";
	stateFunctions[DYN_SUPPORT_LEFT] = &BalanceDemoStateMachine::DynamicSupportLeft;

	ComTrajectory.setSize(3);
	aComDes.resize(3);
	pFootEnd.resize(3);
	pWalkPrep.resize(3);
	
	pMiddleCom << 2.03, 2.0, .48;
	
	pLeftCom   << 2.14, 2-.07, .50;
	pFootEnd << 2.18,2,.1;
	
	
	kpFoot.resize(NS);
	kdFoot.resize(NS);
	aDesFoot.resize(NS);
	pDesFoot.resize(NS);
	vDesFoot.resize(NS);
	RDesFoot.resize(NS);
	
	for (int i=0; i<NS; i++) {
		aDesFoot[i].resize(6);
		vDesFoot[i].resize(6);
		
		aDesFoot[i].setZero();
		vDesFoot[i].setZero();
		pDesFoot[i].setZero();
	}
	aComDes.resize(3);
	aComDes.setZero();
	
	vComDes.resize(3);
	vComDes.setZero();
	
	pComDes.resize(3);
	pComDes.setZero();
	
	kComDes.resize(3);
	kComDes.setZero();
	
	
	state = DROP;
	transitionFlag = true;
	
}
void BalanceDemoStateMachine::Drop() {
	if (transitionFlag) {
		for (int i=0; i<NS; i++) {
			kpFoot[i]=0;
			kdFoot[i]=0;
			aDesFoot[i] << 0,0,0,0,0,0;
			
			RDesFoot[i] <<  0, 1,  0, 
			0, 0, -1,
			-1, 0,  0;
			
		}
		pDesFoot[0] << 2,2,.01;
		pDesFoot[1] << 2.18,2,.01;		
	}
	if (stateTime > .2) {
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}
	transitionFlag = false;
}
void BalanceDemoStateMachine::BalanceMiddle() {
	/*if (stateTime > 1.8) {
		state = DYN_STEP_LEFT;
		transitionFlag = true;
		return;
	}*/
	kpCM = 15;
	kdCM = 25;
	kdAM = 25;
	pComDes = pMiddleCom;
	vComDes.setZero();
	aComDes.setZero();
	kDotDes.setZero();
	kComDes.setZero();
	transitionFlag = false;
}
void BalanceDemoStateMachine::BalanceLeft() {	
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd  = pLeftCom;
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,.8);
		transitionFlag = false;
	}
	if (stateTime > 2) {
		state = BALANCE_RIGHT;
		transitionFlag =true;
		return;
	}
	
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
	kpCM = 90/5;
	kdCM = 2*sqrt(kpCM)/5;
	kdAM = kdCM;
	
}
void BalanceDemoStateMachine::BalanceRight() {	
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd  << 2.00, 2-.07, .50;
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,1.2);
		transitionFlag = false;
	}
	if (stateTime > 2) {
		state = LIFT_FOOT;
		transitionFlag = true;
		return;
	}
	
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
	kpCM = 90/5;
	kdCM = 2*sqrt(kpCM)/5;
	kdAM = kdCM;
	
}
void BalanceDemoStateMachine::WeightShift() {
	static double maxLoad;
	
	if (transitionFlag) {
		transitionFlag = false;
		maxLoad = grfInfo.fCoPs[1](2);
	}
	if (stateTime > 1) {
		state = LIFT_FOOT;
		transitionFlag = true;
		return;
	}
	
	
	 if (stateTime < 1) {
		 AssignFootMaxLoad(1,maxLoad*(1-stateTime));
	 }
	 else {
		 AssignFootMaxLoad(1,0);
	 }
	
	vComDes.setZero();
	aComDes.setZero();
	
}
void BalanceDemoStateMachine::LiftFoot() {
	
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	if (transitionFlag) {
		
		VectorXF vFootEnd(3);
		vFootEnd.setZero();
		
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootEnd, vFootEnd,1);
		
		transitionFlag = false;
	}
	if (stateTime > 1.0) {
		state = CYCLE;
		transitionFlag = true;
	}
	
	
	AssignFootMaxLoad(1,0);
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	VectorXF pDes3(3);
	footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	
	vDesFoot[1].setZero();
	aDesFoot[1].setZero();
	
	pDesFoot[1] = pDes3;
	vDesFoot[1].tail(3) = vDes3;
	aDesFoot[1].tail(3) = aDes3;
	
	//cout << "Lift " << pDesFoot[1].transpose() << endl;
}
void BalanceDemoStateMachine::Cycle() {
	static Vector3F vDes3(3), aDes3(3);
	if (transitionFlag) {
		transitionFlag = false;
	}
	if (stateTime > 4) {
		//state= WALK_PREP;
		//transitionFlag = true;
		//return;
	}
	
	AssignFootMaxLoad(1,0);
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	
	Float om = -2 * (2*M_PI);
	Float ampz = .05, ampy = .05;
	Float som = sin(om*stateTime);
	Float com = cos(om*stateTime);
	
	pDesFoot[1] << 0 , ampy*som, -ampz*com+ampz;
	pDesFoot[1]+=pFootEnd;
	
	vDes3 << 0,ampy*com,ampz*som;
	vDes3 *=om;
	
	aDes3 << 0,-ampy*som,ampz*com;
	aDes3 *= om*om;
	
	vDesFoot[1].setZero();
	aDesFoot[1].setZero();
	vDesFoot[1].tail(3) = vDes3;
	aDesFoot[1].tail(3) = aDes3;
}

const Float StepTime = .5;
const Float SupportTime = .8/1.3;
const Float StepHeight = .06;
const Float CoMxSway = .05;
const Float StepLength = .1;
const Float DesiredVelocity = StepLength / (StepTime + SupportTime);
const Float CoMHeight = .515;
//const Float CoMHeight = .35;

void BalanceDemoStateMachine::WalkPrep()
{
	
	if (stateTime > 1.5) {
		state = FIRST_STEP_LEFT;
		transitionFlag = true;
		return;
	}
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd << 2.09-.08, 2-.07, CoMHeight;
		//pComEnd << 2.09-.08, 2-.11, CoMHeight;
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,1);
		transitionFlag = false;
	}
	kpCM = 15;
	kdCM = 25;
	kdAM = kdCM;
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}



void BalanceDemoStateMachine::FirstStepLeft()
{
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	static VectorXF vFootEnd1(3), vFootEnd2(3),pFootStep1(3),pFootStep2(3);
	
	static bool halfwayFlag = true;
	
	if (transitionFlag) {
		frame->logDataCheckBox->SetValue(true);
		vFootEnd1.setZero();
		vFootEnd1(1) = -StepLength/2 / StepTime * 2;
		pFootStep1 << 2.18, pFoot[1](1) - StepLength/4,StepHeight;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep1, vFootEnd1,StepTime/2);
		
		transitionFlag = false;
		halfwayFlag = true;
	}
	if (stateTime >= StepTime/2 && halfwayFlag  ) {
		
		vFootEnd2.setZero();
		pFootStep2 << 2.18, pFootStep1(1) - StepLength/4,0.01;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep2, vFootEnd2,StepTime/2);
		halfwayFlag = false;
	}
	if (stateTime > StepTime) {
		state = DS_LEFT;
		transitionFlag = true;
		return;
	}
	
	AssignFootMaxLoad(1,0);
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	VectorXF pDes3(3);
	if (stateTime < StepTime/2) {
		footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	}
	else {
		footSpline.eval(stateTime-StepTime/2, pDes3, vDes3, aDes3);
	}

	vDesFoot[1].setZero();
	aDesFoot[1].setZero();
	
	pDesFoot[1] = pDes3;
	vDesFoot[1].tail(3) = vDes3;
	aDesFoot[1].tail(3) = aDes3;
	
	//cout << "Lift " << pDesFoot[1].transpose() << endl;
}


void BalanceDemoStateMachine::StepLeft()
{
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	static VectorXF vFootEnd1(3), vFootEnd2(3),pFootStep1(3),pFootStep2(3);
	
	static bool halfwayFlag = true;
	
	if (transitionFlag) {
		frame->logDataCheckBox->SetValue(true);
		vFootEnd1.setZero();
		vFootEnd1(1) = -StepLength / StepTime * 2;
		pFootStep1 << 2.18, pFoot[1](1) - StepLength/2,StepHeight;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep1, vFootEnd1,StepTime/2);
		
		transitionFlag = false;
		halfwayFlag = true;
	}
	if (stateTime >= StepTime/2 && halfwayFlag  ) {
		
		vFootEnd2.setZero();
		pFootStep2 << 2.18, pFootStep1(1) - StepLength/2,0.01;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep2, vFootEnd2,StepTime/2);
		halfwayFlag = false;
	}
	if (stateTime > StepTime) {
		state = DS_LEFT;
		transitionFlag = true;
		return;
	}
	
	AssignFootMaxLoad(1,0);
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	VectorXF pDes3(3);
	if (stateTime < StepTime/2) {
		footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	}
	else {
		footSpline.eval(stateTime-StepTime/2, pDes3, vDes3, aDes3);
	}
	
	vDesFoot[1].setZero();
	aDesFoot[1].setZero();
	
	pDesFoot[1] = pDes3;
	vDesFoot[1].tail(3) = vDes3;
	aDesFoot[1].tail(3) = aDes3;
	
}
void BalanceDemoStateMachine::DoubleSupportLeft()
{
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd << 2.09+.08, pFoot[1](1), CoMHeight;
		
		kpFoot[1] = 0;
		kdFoot[1] = 0;
		aDesFoot[1].setZero();
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,SupportTime);
		transitionFlag = false;
	}
	if (stateTime > SupportTime) {
		state = STEP_RIGHT;
		transitionFlag = true;
	}
	kpCM = 15;
	kdCM = 25;
	kdAM = kdCM;
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}

void BalanceDemoStateMachine::StepRight()
{
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	static VectorXF vFootEnd1(3), vFootEnd2(3),pFootStep1(3),pFootStep2(3);
	
	static bool halfwayFlag = true;
	
	if (transitionFlag) {
		frame->logDataCheckBox->SetValue(true);
		vFootEnd1.setZero();
		vFootEnd1(1) = -StepLength / StepTime * 2;
		pFootStep1 << 2.0, pFoot[0](1) - StepLength/2,StepHeight;
		footSpline.init(pFoot[0], vFoot[0].tail(3), pFootStep1, vFootEnd1,StepTime/2);
		
		transitionFlag = false;
		halfwayFlag = true;
	}
	if (stateTime >= StepTime/2 && halfwayFlag  ) {
		
		vFootEnd2.setZero();
		pFootStep2 << 2.0, pFootStep1(1) - StepLength/2,0.01;
		footSpline.init(pFoot[0], vFoot[0].tail(3), pFootStep2, vFootEnd2,StepTime/2);
		halfwayFlag = false;
	}
	if (stateTime > StepTime) {
		state = DS_RIGHT;
		transitionFlag = true;
		
		return;
	}
	
	AssignFootMaxLoad(0,0);
	
	kpFoot[0] = 50;
	kdFoot[0] = 150;
	VectorXF pDes3(3);
	if (stateTime < StepTime/2) {
		footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	}
	else {
		footSpline.eval(stateTime-StepTime/2, pDes3, vDes3, aDes3);
	}
	
	vDesFoot[0].setZero();
	aDesFoot[0].setZero();
	
	pDesFoot[0] = pDes3;
	vDesFoot[0].tail(3) = vDes3;
	aDesFoot[0].tail(3) = aDes3;
}
void BalanceDemoStateMachine::DoubleSupportRight()
{
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd << 2.09-.08, pFoot[0](1), CoMHeight;
		
		kpFoot[0] = 0;
		kdFoot[0] = 0;
		aDesFoot[0].setZero();
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,SupportTime);
		transitionFlag = false;
	}
	if (stateTime > SupportTime) {
		state = STEP_LEFT;
		transitionFlag = true;
	}
	kpCM = 15;
	kdCM = 25;
	kdAM = kdCM;
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}

void BalanceDemoStateMachine::Fall()
{
	static Float r, Iyy, thetaDes, thetaDotDes;
	
	if (transitionFlag) {
		simThread->idt/=10;
		simThread->cdt = .0005;
		r = sqrt(pow(pCom(0)-2.02,2) + pow(pCom(2),2));
		thetaDes = atan2(pCom(0)-2.02, pCom(2));;
		thetaDotDes = vCom.norm()/r;
		
		transitionFlag = false;
	}
	if (grfInfo.fCoPs[1](2) < 5) {
		simThread->idt = .0001/1500;
	}
	
	Float theta = atan2(pCom(0)-2.02, pCom(2));
	Float thetaDot = vCom.norm()/r;
	
	Iyy = IBarC0(1,1);
	
	
	frame->logDataCheckBox->SetValue(true);
	kpCM = 15;
	kdCM = 25;
	kdAM = 25;
	
	
	Float thetaDDot = totalMass * 9.81 * r * sin(theta) / (totalMass * pow(r,2) + Iyy);
	
	pComDes(0) = r*sin(thetaDes)+2.02;
	pComDes(2) = r*cos(thetaDes);
	
	vComDes(0) = r*cos(thetaDes)*thetaDotDes;
	vComDes(2) = -r*sin(thetaDes)*thetaDotDes;
	
	aComDes(0) = - r * sin(thetaDes) * pow(thetaDotDes,2) + r * cos(thetaDes) * thetaDDot;
	aComDes(1) = 0;
	aComDes(2) = -r * cos(thetaDes) * pow(thetaDotDes,2) - r*sin(thetaDes)*thetaDDot;
	
	kComDes << 0,(Iyy)*thetaDotDes,0;
	kDotDes << 0,Iyy * thetaDDot ,0;
	
	thetaDes += simThread->cdt*thetaDotDes;
	thetaDotDes += simThread->cdt*thetaDDot;
}

void BalanceDemoStateMachine::DynamicStepLeft()
{
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	static VectorXF vFootEnd1(3), vFootEnd2(3),pFootStep1(3),pFootStep2(3);
	
	static bool halfwayFlag = true;
	
	if (transitionFlag) {
		frame->logDataCheckBox->SetValue(true);
		simThread->idt/=20;
		vFootEnd1.setZero();
		vFootEnd1(0) = +StepLength / StepTime * 2;
		pFootStep1 << pFoot[1](0)+StepLength/2, pFoot[1](1),StepHeight;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep1, vFootEnd1,StepTime/2);
		
		transitionFlag = false;
		halfwayFlag = true;
	}
	if (stateTime >= StepTime/2 && halfwayFlag  ) {
		//cout << setprecision(5);
		//cout << "Halfway " << simThread->sim_time << endl;
		vFootEnd2.setZero();
		pFootStep2 << pFootStep1(0)+StepLength/2, pFootStep1(1),0.01;
		footSpline.init(pFoot[1], vFoot[1].tail(3), pFootStep2, vFootEnd2,StepTime/2);
		halfwayFlag = false;
	}
	if (stateTime > StepTime) {
		state = DYN_SUPPORT_LEFT;
		transitionFlag = true;
		return;
	}
	
	AssignFootMaxLoad(1,0);
	taskConstrActive.tail(6).setZero();
	taskOptimActive.tail(6).setOnes();
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	VectorXF pDes3(3);
	if (stateTime < StepTime/2) {
		footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	}
	else {
		footSpline.eval(stateTime-StepTime/2, pDes3, vDes3, aDes3);
	}
	
	vDesFoot[1].setZero();
	aDesFoot[1].setZero();
	
	pDesFoot[1] = pDes3;
	vDesFoot[1].tail(3) = vDes3;
	aDesFoot[1].tail(3) = aDes3;
}
void BalanceDemoStateMachine::DynamicSupportLeft()
{
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd << 2.09+.08, pFoot[1](1), CoMHeight;
		
		kpFoot[1] = 0;
		kdFoot[1] = 0;
		aDesFoot[1].setZero();
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,SupportTime);
		transitionFlag = false;
	}
	if (stateTime > SupportTime) {
		state = STEP_RIGHT;
		transitionFlag = true;
	}
	kpCM = 15;
	kdCM = 25;
	kdAM = kdCM;
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}

void BalanceDemoStateMachine::StateControl(ControlInfo & ci)
{
	
	this->HumanoidController::ControlInit();
	taskOptimActive.setOnes(6+NJ+12);
	taskConstrActive.setZero(6+NJ+12);
	
	taskOptimActive.segment(6+NJ, 12).setZero();
	taskConstrActive.segment(6+NJ, 12).setOnes();
	
	TaskWeight.setOnes(6+NJ+12);
	
	// Do at least one state call, and more if it transitioned out
	do {
		if (transitionFlag) {
			stateTime = 0;
		}
		(this->*stateFunctions[state])();
	} while (transitionFlag);
	
	if (state != DROP) {
		int taskRow = 0;
		// Compute Centroidal Task Information
		{
			//Our task jacobain will be a desired pose + centroidal quantities
			TaskJacobian.block(0,0,6,NJ+6) = CentMomMat;
			
			// Compute Task Bias from RNEA Fw Kin
			TaskBias.segment(0,6) = -cmBias;
			
			hDes.head(3) = kComDes;
			hDes.tail(3) = vComDes*totalMass;
			
			Vector3F linMomDotDes = totalMass*aComDes + totalMass*kpCM*(pComDes - pCom) + kdCM*(vComDes*totalMass - centMom.tail(3));
			Vector3F angMomDotDes = kDotDes + kdAM*(kComDes-centMom.head(3));
			hDotDes.head(3) = angMomDotDes;
			hDotDes.tail(3) = linMomDotDes;
			
			// Dampen angular and PD CoM
			TaskBias.segment(0,3) += angMomDotDes;
			TaskBias.segment(3,3) += linMomDotDes;
			
			TaskWeight.segment(0,3).setConstant(10);
			TaskWeight.segment(3,3).setConstant(100);
			
	#ifdef CONTROL_DEBUG
			{
				//cout << "eCom " << (pComDes - pCom).transpose() << endl;
				//cout << "veCom " << (vComDes - centMom.tail(3) / totalMass) << endl;
				//cout << "adCom " << aComDes.transpose() << endl;
				cout << "pCoM " << pCom.transpose() << endl;
				cout << "cm " << centMom.transpose() << endl;
				cout << "vt " << qd.head(6).transpose() << endl;
				
				//cout << "Total Mass " << totalMass << endl;
				cout << "ldotdes " << linMomDotDes.transpose() << endl;
				//cout << "kdotdes " << angMomDotDes << endl;
			}
	#endif
			
			taskRow +=6;
		}
		
		//Compute Joint Task Information
		{
			Float discountFactor = 1;
			TaskJacobian.block(taskRow,0,NJ,6) = MatrixXF::Zero(NJ,6);
			TaskJacobian.block(taskRow,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ)*discountFactor;
			
			// Compute task bias based on PD on joint angles
			VectorXF qDes = VectorXF::Zero(NJ+6); 
			//0-5 6-11 12-17 18-21 22-25
			qDes << 0,0,0,0,0,0,  0,0,0,1.27,0,0   ,0,0,0,1.27,0,0,   0,2.5,-1.57, 0,    0, -2.4, -1.57, 0;
			Float Kp = 20, Kd = 12;
			
			// Joint PDs
			int k=6;
			int kdm=7;
			for (int i=1; i<artic->getNumLinks(); i++) {
				LinkInfoStruct * bodyi = artic->m_link_list[i];
				if (bodyi->dof) {
					if (bodyi->dof == 1) {
						TaskBias(taskRow+k-6) = (Kp * (qDes(k) - q(kdm)) - Kd * qd(k))*discountFactor;
					}
					if (k <18) {
						taskOptimActive.segment(taskRow+k-6,bodyi->dof).setZero();
					}
				}
				
				kdm+=bodyi->link->getNumDOFs();
				k+=bodyi->link->getTrueNumDOFs();
			}
			
			
			// Handle Orientation Error for Shoulder Joints
			Matrix3F tmpR, i_R_pi_mat, RAct, RDes[2];
			Vector3F eOmega, om;
			
			tmpR << 0,0,1,  1,0,0,  0,1,0;
			RDes[0] = tmpR.transpose();
			RDes[1] = RDes[0];
			
			CartesianTensor i_R_pi;
			CartesianVector pShoulder;
			
			const int linkNums[] = {10,12};
			const int jointNums[] = {12,16};
			for (int i=0; i<2; i++) {
				const int linkNum = linkNums[i];
				const int jointNum = jointNums[i];
				
				artic->getLink(linkNum)->getPose(i_R_pi,pShoulder);
				copyRtoMat(i_R_pi, i_R_pi_mat);
				
				RAct = i_R_pi_mat.transpose();
				
				matrixLogRot(RDes[i]*RAct.transpose(),eOmega);
				
				// Note: omega is in "i" coordinates, error in p(i) coordinates
				Vector3F omega = qd.segment(6+jointNum,3);
				
				// Note: Velocity bias accel is omega cross omega
				TaskBias.segment(taskRow+jointNum,3) = (i_R_pi_mat*(Kp *eOmega) - Kd * omega)*discountFactor - omega.cross(omega);
			}
			
			
			// Scale Rows Accordingly...
			// Legs
			
			// Left and Right Arms
			TaskWeight.segment(taskRow+12,4).setConstant(10);
			TaskWeight.segment(taskRow+16,4).setConstant(10);
			
			taskRow += NJ;
		}
		
		//Compute Foot Task Information
		{
			vector<MatrixXF > footJacs(NS);
			vector<Vector6F > footBias(NS);
			
			MatrixX6F X;
			X.resize(6,6);  X.block(0,3,3,3).setZero(); X.block(3,0,3,3).setZero();
			//Matrix3F R;
			Matrix3F eR;

			Vector3F eOmega;
			Vector6F aCom;
			int constraintRow = 0;
			
			for (int i=0; i<NS; i++) {
				Vector6F aCom;
				
				int jointIndex = SupportIndices[i];
				LinkInfoStruct * link = artic->m_link_list[jointIndex];

				// Compute the orientation Error
				eR = RDesFoot[i] * RFoot[i].transpose();
				matrixLogRot(eR,eOmega);
				
				// Use a PD to create a desired acceleration (everything here is in inertial coordinates)
				aCom.head(3) = kpFoot[i] * eOmega;
				aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - pFoot[i]);
				aCom += aDesFoot[i] + kdFoot[i] * (vDesFoot[i] - vFoot[i]);
				
				if (kdFoot[i] == 0) {
					aCom = - 10 * vFoot[i];
					//aCom.setZero();
				}

				
				// Compute Task Information
				X.block(0,0,3,3) = RFoot[i]; 
				X.block(3,3,3,3) = RFoot[i];
				artic->computeJacobian(jointIndex,X,footJacs[i]);
				computeAccBiasFromFwKin(link->link_val2,footBias[i]);
				
				// Load Task Information
				TaskJacobian.block(taskRow,0,6,NJ+6) = footJacs[i];
				TaskBias.segment(taskRow,6)          = aCom - footBias[i];
				
				aDesFoot[i] = aCom;
				
				// Option to Scale Linear/Angular Position Control
				TaskWeight.segment(taskRow  ,6).setConstant(10);
				TaskWeight.segment(taskRow+3,3).setConstant(1000);
				
				taskRow+=6;
			}
		}
		//cout << "Total Tasks! " << taskRow << endl;
		//cout << "Size " << TaskBias.size() << endl;
		HumanoidController::HumanoidControl(ci);
		//exit(-1);
	}
	
	
	stateTime += simThread->cdt;
}