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
	
	
	

	ComTrajectory.setSize(3);
	aComDes.resize(3);
	pFootEnd.resize(3);
	pWalkPrep.resize(3);
	
	pMiddleCom << 2.0-.07, 2.08, .40;
	
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
		state = WALK_PREP;
		transitionFlag = true;
		return;
	}*/
	kpCM = 15;
	kdCM = 25;
	pComDes = pMiddleCom;
	vComDes.setZero();
	aComDes.setZero();
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
const Float StepLength = .2;
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
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}



void BalanceDemoStateMachine::StateControl(ControlInfo & ci)
{
	
	this->HumanoidController::ControlInit();
	
	// Do at least one state call, and more if it transitioned out
	do {
		if (transitionFlag) {
			stateTime = 0;
		}
		(this->*stateFunctions[state])();
	} while (transitionFlag);
	//cout << setprecision(4);
	//cout << state << " " << pComDes.transpose() << endl;
	
	if (state != DROP) {
		int taskRow = 0;
		// Compute Centroidal Task Information
		{
			//Our task jacobain will be a desired pose + centroidal quantities
			TaskJacobian.block(0,0,6,NJ+6) = CentMomMat;
			
			// Compute Task Bias from RNEA Fw Kin
			TaskBias.segment(0,6) = -cmBias;
			
			Vector3F linMomDotDes = totalMass*aComDes + totalMass*kpCM*(pComDes - pCom) + kdCM*(vComDes*totalMass - centMom.tail(3));
			Vector3F angMomDotDes = -kdCM*centMom.head(3);
			hDotDes.head(3) = angMomDotDes;
			hDotDes.tail(3) = linMomDotDes;
			
			//cout << "eCom " << (pComDes - pCom).transpose() << endl;
			//cout << "veCom " << (vComDes - centMom.tail(3) / totalMass) << endl;
			//cout << "adCom " << aComDes.transpose() << endl;
			
			// Dampen angular and PD CoM
			TaskBias.segment(0,3) += angMomDotDes;
			TaskBias.segment(3,3) += linMomDotDes;
			
			TaskJacobian.block(3,0,3,NJ+6)*=100;
			TaskBias.segment(3,3)*=100;
			
			
	#ifdef CONTROL_DEBUG
			{
				cout << "pCoM " << pCom.transpose() << endl;
				cout << "cm " << centMom.transpose() << endl;
				cout << "vt " << qd.head(6).transpose() << endl;
				
				//cout << "Total Mass " << totalMass << endl;
				cout << "ldotdes " << linMomDotDes.transpose() << endl;
				//cout << "kdotdes " << angMomDotDes << endl;
			}
	#endif
			
			// Turn off momentum control
			//tsc->TaskJacobian.block(0,0,6,NJ+6).setZero();
			//tsc->TaskBias.segment(0,6).setZero();
			
			taskRow +=6;
		}
		
		//Compute Joint Task Information
		{
			Float discountFactor = 1;
			TaskJacobian.block(taskRow,0,NJ,6) = MatrixXF::Zero(NJ,6);
			TaskJacobian.block(taskRow,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ)*discountFactor;
			
			// Compute task bias based on PD on joint angles
			VectorXF qDes = VectorXF::Zero(NJ); 					  
			qDes << 0,0,0,0,0,0,0,0,0,0,0,0, 0,2.5,-1.57, 0, 0, -2.4, -1.57, 0;
			Float Kp = 20, Kd = 12;
			TaskBias.segment(taskRow,NJ) = (Kp * (qDes - q.segment(7,NJ)) - Kd * qd.segment(6,NJ))*discountFactor;
			
			// Handle Orientation Error for Shoulder Joints
			Matrix3F RAct;
			Matrix3F RDes[2];
			Vector3F eOmega;
			Vector3F om;
			
			om << 0 , -.9*M_PI, -.1*M_PI;
			matrixExpOmegaCross(om, RDes[0]);
			
			om << 0 , .9*M_PI, .1*M_PI;
			matrixExpOmegaCross(om, RDes[1]);
			
			
			RDes[0].setIdentity();
			RDes[1].setIdentity();
			
			CartesianTensor RShoulder;
			CartesianVector pShoulder;
			
			const int linkNums[] = {10,12};
			const int jointNums[] = {12,16};
			for (int i=0; i<2; i++) {
				const int linkNum = linkNums[i];
				const int jointNum = jointNums[i];
				
				G_robot->getLink(linkNum)->getPose(RShoulder,pShoulder);
				copyRtoMat(RShoulder, RAct);
				matrixLogRot(RDes[i]*RAct.transpose(),eOmega);
				eOmega *=-1;
				
				TaskBias.segment(taskRow+jointNum,3) = (Kp * eOmega - Kd * qd.segment(6+jointNum,3))*discountFactor;
			}
			// Scale Rows Accordingly...
			// Legs
			TaskJacobian.block(taskRow,0,12,NJ+6) *= 0;
			TaskBias.segment(taskRow,12)        *= 0;
			
			// Right Arm
			TaskJacobian.block(taskRow+12,0,4,NJ+6) *= 10;
			TaskBias.segment(taskRow+12,4)		   *= 10;
			
			
			// Left Arm
			TaskJacobian.block(taskRow+16,0,4,NJ+6) *= 10;
			TaskBias.segment(taskRow+16,4)		   *= 10;
			
			taskRow += NJ;
		}
		
		//Compute Foot Task Information
		{
			vector<MatrixXF > footJacs(NS);
			vector<Vector6F > footBias(NS);
			
			MatrixX6F X;
			X.resize(6,6);  X.block(0,3,3,3).setZero(); X.block(3,0,3,3).setZero();
			Matrix3F R;
			Matrix3F eR;

			Vector3F eOmega, pAct;
			Vector6F aCom, vAct;
			
			for (int i=0; i<NS; i++) {
				Vector6F aCom;
				
				int jointIndex = SupportIndices[i];
				LinkInfoStruct * link = G_robot->m_link_list[jointIndex];
				copyRtoMat(link->link_val2.R_ICS, R);
				COPY_P_TO_VEC(link->link_val2.p_ICS, pAct);
				
				// Compute the orientation Error
				eR = RDesFoot[i] * R.transpose();
				matrixLogRot(eR,eOmega);
				
				// Compute the link velocity in the ICS
				vAct.head(3) = R * link->link_val2.v.head(3);
				vAct.tail(3) = R * link->link_val2.v.tail(3);
				
				
				// Use a PD to create a desired acceleration
				aCom.head(3) = kpFoot[i] * eOmega;
				aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - pAct);
				aCom += aDesFoot[i] + kdFoot[i] * (vDesFoot[i] - vAct);
				
				// Compute Task Information
				X.block(0,0,3,3) = R; 
				X.block(3,3,3,3) = R;
				G_robot->computeJacobian(jointIndex,X,footJacs[i]);
				computeAccBiasFromFwKin(link->link_val2,footBias[i]);
				
				// Load Task Information
				TaskJacobian.block(taskRow,0,6,NJ+6) = footJacs[i];
				TaskBias.segment(taskRow,6)          = aCom - footBias[i];
				
				// Option to Scale Linear Position Control
				TaskJacobian.block(taskRow+3,0,3,NJ+6) *=100; 
				TaskBias.segment(taskRow+3,3) *= 100;
				
				
				// Option to Scale whole task
				TaskJacobian.block(taskRow,0,6,NJ+6)*=10;
				TaskBias.segment(taskRow,6)*=10;
				
				taskRow+=6;
			}
		}
		HumanoidController::HumanoidControl(ci);
	}
	
	
	stateTime += simThread->cdt;
}