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
	
	int numLinks = artic->getNumLinks();
	artic->getTrueNumDOFs();
	
	RDesJoint.resize(numLinks);
	posDesJoint.resize(numLinks);
	rateDesJoint.resize(numLinks);
	accDesJoint.resize(numLinks);
	kpJoint.resize(numLinks);
	kdJoint.resize(numLinks);
	
	for (int i=0; i<numLinks; i++) {
		LinkInfoStruct * bodyi = artic->m_link_list[i];
		int dof = bodyi->dof;
		if (dof != 0) {
			if (dof == 6) {
				posDesJoint[i].setZero(3);
			}
			else if (dof ==3)
			{
				// Do nothing, just and RDes which is of right size
			}
			else if (dof == 1)
			{
				posDesJoint[i].setZero(1);
			}
			rateDesJoint[i].setZero(dof);
			accDesJoint[i].setZero(dof);
		}
	}
	
	state = DROP;
	transitionFlag = true;
	
}
void JumpingStateMachine::Drop() {
	if (transitionFlag) {
		cout << setprecision(8) << endl;
		cout << pCom.transpose() << endl;
		
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
		
		int numLinks = artic->getNumLinks();
		for (int i=0; i<numLinks; i++) {
			LinkInfoStruct * bodyi = artic->m_link_list[i];
			int dof = bodyi->dof;
			if (dof != 0) {
				if (dof == 6) {
					Vector3F om;
					om << 0,M_PI/20,0;
					matrixExpOmegaCross(om,RDesJoint[i]);
				}
				else if (dof ==3)
				{
					CartesianTensor i_R_pi;
					CartesianVector pShoulder;
					Matrix3F i_R_pi_mat;
					bodyi->link->getPose(i_R_pi,pShoulder);
					copyRtoMat(i_R_pi, i_R_pi_mat);
					RDesJoint[i] = i_R_pi_mat.transpose();
				}
				else if (dof == 1)
				{
					Float qLink[0],qdLink[0];
					bodyi->link->getState(qLink, qdLink);
					posDesJoint[i](0) = qLink[0]; 
				}
			}
		}
	}
	if (stateTime > .20) {
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}
	transitionFlag = false;
}
void JumpingStateMachine::BalanceMiddle() {
	if (transitionFlag) {
		cout << pCom << endl;
	}
	if (stateTime > 1) {
		state = SQUAT;
		transitionFlag = true;
		return;
	}
	kpCM = 30;
	kdCM = 2*sqrt(kpCM);
	kdAM = 25;
	pComDes << 2.00, 2.0, .48;
	//pComDes << 2.00, 2.0, .72;
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
		cout << "Squat" << endl;
		VectorXF pComInit = pCom;
		VectorXF vComInit = vCom;
		
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		
		pComEnd << 2.02, 2.0, .32;
		
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
	if (transitionFlag) {
		cout << "Thrust" << endl;
	}
	//cout << pCom(2) << endl;
	//cout << "Thrust" << endl;
	if (q(11) < .75) {
	//if (pCom(2) > .485) {
		state = FLIGHT;
		transitionFlag = true;
		return;
	}
	kpCM = 155;
	kdCM = 2*sqrt(kpCM);
	kdAM = 225;

	//pComDes << 2.06, 2.0, .55;
	//vComDes << .5,0,2;
	pComDes << 2.06, 2.0, .55;
	vComDes << .5,0,2;
	transitionFlag = false;
	
}

void JumpingStateMachine::Flight()
{
	static Vector3F pFootRel0, pFootRel1;
	
	//static CubicSpline transitionSpline;
	static CubicSplineTrajectory footSpline(3);
	
	if (transitionFlag) {
		cout << "Flight" << endl;
		//simThread->idt/=10000;
		//simThread->cdt/=1;
		
		pFootRel0 = pFoot[0] - pCom;
		pFootRel1 = pFoot[1] - pCom;
		
		transitionFlag =false;
		
		Vector3F pInit, vZero, pFinal,vInit;
		pInit.setZero();
		vZero.setZero();
		pFinal << .1, 0 , .03;
		
		//vInit = vFoot[0].tail(3) -vCom;
		vInit.setZero();
		
		footSpline.init(pInit, vInit, pFinal, vZero, .2);
		//transitionSpline.init(0, 0, 1, 0, .05);
	}
	taskOptimActive.head(6).setZero();
	taskConstrActive.tail(12).setZero();
	taskOptimActive.tail(12).setOnes();

	//TaskWeight.tail(12).setConstant(10);
	VectorXF p(3), pd(3), pdd(3);
	footSpline.eval(stateTime, p,pd, pdd);
	
	pDesFoot[0] = pFootRel0 + pCom + p;
	pDesFoot[1] = pFootRel1 + pCom + p;
	
	
	
	vDesFoot[0].setZero();
	vDesFoot[1].setZero();
	
	
	vDesFoot[0].tail(3) = vCom + pd;
	vDesFoot[1].tail(3) = vCom + pd;
	
	AssignFootMaxLoad(1,0);
	AssignFootMaxLoad(0,0);
	
	kpFoot[0] = 50;
	kdFoot[0] = 150;
	
	kpFoot[1] = 50;
	kdFoot[1] = 150;
	
	aDesFoot[0] << 0,0,0,0,0,-9.81;
	aDesFoot[1] << 0,0,0,0,0,-9.81;
	
	aDesFoot[0].tail(3) += pdd;
	aDesFoot[1].tail(3) += pdd;
	
	//cout << taskConstrActive.transpose() << endl;

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
		pComDes(0) +=.05;
		pComDes(2) = .49; 
		vComDes.setZero();
		aComDes.setZero();
		transitionFlag = false;
	}
	//kpCM = 15;
	//kdCM = 25;
	
	kpCM = 20;
	kdCM = 2*sqrt(kpCM);
	
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
	dmTimespec controlStart, controlEnd;
	dmGetSysTime(&controlStart);
	
	this->HumanoidController::ControlInit();
	taskOptimActive.setOnes(6+NJ+6+12);
	taskConstrActive.setZero(6+NJ+6+12);
	
	taskOptimActive.tail(12).setZero();
	taskConstrActive.tail(12).setOnes();
	
	TaskWeight.setOnes(6+NJ+6+12);
	
	TaskWeight.tail(12).setConstant(1000);
	//TaskWeight.segment(taskRow  ,6).setConstant(10);
	//TaskWeight.segment(taskRow+3,3).setConstant(1000);
	
	
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
			TaskJacobian.block(taskRow,0,NJ+6,NJ+6) = MatrixXF::Identity(NJ+6,NJ+6);
			
			// Compute task bias based on PD on joint angles
			{
				Float kpAnk, kpKnee, kpHip, kpShould, kpElbow, kpTorso;
				Float wAnk, wKnee, wHip, wShould, wElbow, wTorso;
				
				kpAnk = 120;
				wAnk  = 1;
				
				kpKnee = 240;
				wKnee   = 1;
				
				kpHip = 120;
				wHip  = 1;
				
				kpShould = 200;
				wShould  = 1.5;
				
				kpElbow = 240;
				wElbow  = 1;
				
				kpTorso = 120./10.;
				wTorso = 1/2.;
				
				{
					kpJoint[0] = kpTorso;
					kdJoint[0] = 2*sqrt(kpTorso);
					TaskWeight.segment(taskRow,3).setConstant(wTorso);
					////////////////////////////////////////////////////////
					kpJoint[2] = kpHip;
					kdJoint[2] = 2*sqrt(kpHip);
					TaskWeight.segment(taskRow+6,3).setConstant(wHip);
					
					kpJoint[3] = kpKnee;
					kdJoint[3] = 2*sqrt(kpKnee);
					TaskWeight(taskRow+9) = wKnee;
					
					kpJoint[4] = kpAnk;
					kpJoint[5] = kpAnk;
					
					kdJoint[4] = 2*sqrt(kpAnk);
					kdJoint[5] = 2*sqrt(kpAnk);
					
					TaskWeight.segment(taskRow+10,2).setConstant(wAnk);
					///////////////////////////////////////////////////////
					kpJoint[6] = kpHip;
					kdJoint[6] = 2*sqrt(kpHip);
					TaskWeight.segment(taskRow+12,3).setConstant(wHip);
					
					kpJoint[7] = kpKnee;
					kdJoint[7] = 2*sqrt(kpKnee);
					TaskWeight(taskRow+15) = wKnee;
					
					kpJoint[8] = kpAnk;
					kpJoint[9] = kpAnk;
					
					kdJoint[8] = 2*sqrt(kpAnk);
					kdJoint[9] = 2*sqrt(kpAnk);
					
					TaskWeight.segment(taskRow+16,2).setConstant(wAnk);
					/////////////////////////////////////////////////////////
					kpJoint[10] = kpShould;
					kdJoint[10] = 2*sqrt(kpShould);
					TaskWeight.segment(taskRow+18,3).setConstant(wShould);
					
					kpJoint[11] = kpElbow;
					kdJoint[11] = 2*sqrt(kpElbow);
					TaskWeight(taskRow+21) = wElbow;
					
					kpJoint[12] = kpShould;
					kdJoint[12] = 2*sqrt(kpShould);
					TaskWeight.segment(taskRow+22,3).setConstant(wShould);
					
					kpJoint[13] = kpElbow;
					kdJoint[13] = 2*sqrt(kpElbow);
					TaskWeight(taskRow+25) = wElbow;
					/////////////////////////////////////////////////////////
				}
				
				
				
			}
			
			
			
			
			
			//Float Kp = 120, Kd = 2*sqrt(Kp);
			
			
			// Joint PDs
			int jointIndexDm=0;
			CartesianTensor i_R_pi;
			CartesianVector pLink;
			Matrix3F tmpR, i_R_pi_mat, RAct;
			Vector3F eOmega, om;
			
			for (int i=0; i<artic->getNumLinks(); i++) {
				LinkInfoStruct * bodyi = artic->m_link_list[i];
				int jointIndex = bodyi->index_ext;
				int jointDof = bodyi->dof;
				
				if (jointDof) {
					
					Float Kp = kpJoint[i];
					Float Kd = kdJoint[i];
					if (i == 0) {
						TaskWeight.segment(taskRow,3).setConstant(10);
						taskOptimActive.segment(taskRow+3,3).setZero();
					}
					else if (bodyi->dof == 1) {
						TaskBias(taskRow+jointIndex) = Kp * (posDesJoint[i](0) - q(jointIndexDm)) 
															+ Kd * (rateDesJoint[i](0) - qd(jointIndex));
					}
					else if (jointIndex < 18) {
						TaskWeight.segment(taskRow+jointIndex,jointDof).setConstant(.1);
						//taskOptimActive.segment(taskRow+jointIndex,jointDof).setZero();
					}
					
					if (jointDof >= 3) {
						if (i >= 10) {
							/*if (i == 10) {
								tmpR << 0,-1,0, 0,0,1, 1,0,0;
							}
							else
							{
								tmpR << 0,1,0, 0,0,1, -1,0,0;
							}
							tmpR << 1,0,0, 0,0,1, 0,-1,0;*/
							//tmpR << 0,0,1,  1,0,0,  0,1,0;
							
							
							//RDesJoint[i] = tmpR.transpose();
						}
						bodyi->link->getPose(i_R_pi,pLink);
						copyRtoMat(i_R_pi, i_R_pi_mat);
						
						RAct = i_R_pi_mat.transpose();
						
						matrixLogRot(RDesJoint[i]*RAct.transpose(),eOmega);
						
						// Note: omega is in "i" coordinates, error in p(i) coordinates
						Vector3F omega = qd.segment(bodyi->index_ext,3);
						
						// Note: Velocity bias accel is omega cross omega
						TaskBias.segment(taskRow+jointIndex,3) = (i_R_pi_mat*(Kp *eOmega) + Kd * (rateDesJoint[i].head(3) - omega)) - omega.cross(omega);
					}
				}
				jointIndexDm+=bodyi->link->getNumDOFs();
			}
			
			taskRow += NJ+6;
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
				
				
				taskRow+=6;
			}
		}
		//cout << "Total Tasks! " << taskRow << endl;
		//cout << "Size " << TaskBias.size() << endl;
		HumanoidController::HumanoidControl(ci);
		
		dmGetSysTime(&controlEnd);
		
		controlTime = timeDiff(controlStart, controlEnd);
		
		if (frame->logDataCheckBox->IsChecked()) {
			logData();
		}
		//cout << "Control Complete " << simThread->sim_time << endl;
		//exit(-1);
	}
	
	stateTime += simThread->cdt;
}