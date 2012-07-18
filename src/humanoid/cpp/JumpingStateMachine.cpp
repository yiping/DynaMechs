/*
 *  BalanceDemoStateMachine.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/27/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"
#include "JumpingStateMachine.h"

JumpingStateMachine::JumpingStateMachine(dmArticulation * robot) 
	: HumanoidDataLogger(robot, NUM_JUMP_STATES), ComTrajectory(3)
{
	stateFunctions.resize(numStates);
	
	stateNames[DROP] = "Drop";
	stateFunctions[DROP] = &JumpingStateMachine::Drop;
	
	stateNames[BALANCE_MIDDLE] = "Balance Middle";
	stateFunctions[BALANCE_MIDDLE] = &JumpingStateMachine::BalanceMiddle;

	stateNames[SQUAT] = "Squat";
	stateFunctions[SQUAT] = &JumpingStateMachine::Squat;
	
	stateNames[THRUST] = "Thrust";
	stateFunctions[THRUST] = &JumpingStateMachine::Thrust;
	
	stateNames[FLIGHT] = "Flight";
	stateFunctions[FLIGHT] = &JumpingStateMachine::Flight;
	
	stateNames[LAND] = "Land";
	stateFunctions[LAND] = &JumpingStateMachine::Land;
	

	ComTrajectory.setSize(3);
	//aComDes.resize(3);
	
	
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
void JumpingStateMachine::Drop() {
	if (transitionFlag) {
		for (int i=0; i<NS; i++) {
			kpFoot[i]=0;
			kdFoot[i]=0;
			aDesFoot[i] << 0,0,0,0,0,0;
			
			RDesFoot[i] <<  0, 0,  1, 
			0, 1, 0,
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
void JumpingStateMachine::BalanceMiddle() {
	if (stateTime > 1) {
		state = SQUAT;
		transitionFlag = true;
		return;
	}
	kpCM = 15;
	kdCM = 25;
	kdAM = 25;
	pComDes << 2.03, 2.0, .48;
	vComDes.setZero();
	aComDes.setZero();
	kDotDes.setZero();
	kComDes.setZero();
	transitionFlag = false;
}

void JumpingStateMachine::Squat()
{
	const Float SquatTime = .5;
	
	if(transitionFlag)
	{
		VectorXF pComInit = pCom;
		VectorXF vComInit = vCom;
		
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		
		pComEnd << 2.07, 2.0, .35;
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd, SquatTime);
		transitionFlag = false;
	}
	if (stateTime > SquatTime) {
		state = THRUST;
		transitionFlag = true;
		return;
	}
	
	kpCM = 15;
	kdCM = 25;
	kdAM = 25;

	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
	kDotDes.setZero();
	kComDes.setZero();
}

void JumpingStateMachine::Thrust()
{
	//cout << "Thrust" << endl;
	if (pCom(2) > .50) {
		state = FLIGHT;
		transitionFlag = true;
		return;
	}
	kpCM = 155;
	kdCM = 25;
	kdAM = 125;

	pComDes << 2.1, 2.0, .55;
	vComDes << .5,0,2;
	transitionFlag = false;
	
}

void JumpingStateMachine::Flight()
{
	static Vector3F pFootRel0, pFootRel1;
	static CubicSpline xFootSpline;
	
	if (transitionFlag) {
		//simThread->idt/=10;
		//simThread->cdt/=1;
		
		pFootRel0 = pFoot[0] - pCom;
		pFootRel1 = pFoot[1] - pCom;
		
		transitionFlag =false;
		xFootSpline.init(0, 0, .1, 0, .2);
		
	}
	taskOptimActive.head(6).setZero();
	taskConstrActive.tail(12).setZero();
	taskOptimActive.tail(12).setOnes();
	
	Float x, xd, xdd;
	xFootSpline.eval(stateTime, x,xd, xdd);
	
	pDesFoot[0] = pFootRel0 + pCom;
	pDesFoot[1] = pFootRel1 + pCom;
	pDesFoot[0](0) += x;
	pDesFoot[1](0) += x;
	
	
	vDesFoot[0].setZero();
	vDesFoot[1].setZero();
	
	
	vDesFoot[0].tail(3) = vCom;
	vDesFoot[1].tail(3) = vCom;
	
	vDesFoot[0](3) += xd;
	vDesFoot[1](3) += xd;
	
	AssignFootMaxLoad(1,0);
	AssignFootMaxLoad(0,0);
	
	kpFoot[0] = 50;
	kdFoot[0] = 150;
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	
	aDesFoot[0] << 0,0,0,xdd,0,-9.81;
	aDesFoot[1] << 0,0,0,xdd,0,-9.81;
	

	if (contactState[0]>=10 && contactState[1]>=10 && stateTime>.1) {
		cout << "Landed" << endl;
		state = LAND;
		transitionFlag = true;
	}
}

void JumpingStateMachine::Land()
{
	static int i = 0;
	
	//cout << "Land Time " << ++i << endl;
	
	if (transitionFlag) {
		//simThread->idt/=10;
		//simThread->cdt/=10;
		
		cout << pCom << endl;
		
		pComDes = pCom;
		pComDes(2) = .45; 
		vComDes.setZero();
		aComDes.setZero();
		transitionFlag = false;
	}
	kpCM = 15;
	kdCM = 25;
	
	kdAM = 25;
	kpFoot[0] = 0;
	kdFoot[0] = 0;
	
	kpFoot[1] = 0;
	kdFoot[1] = 0;
	
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	
}

void JumpingStateMachine::StateControl(ControlInfo & ci)
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
			
			TaskWeight.segment(0,3).setConstant(100);
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