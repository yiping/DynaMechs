/*
 *  RunningStateMachine.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 1/7/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "RunningStateMachine.h"
#include "GlobalDefines.h"
#include "BezierCurve.h"
#include "dmRigidBody.hpp"

RunningStateMachine::RunningStateMachine(dmArticulation * robot) 
: HumanoidDataLogger(robot, NUM_RUN_STATES), ComTrajectory(3)
{
	
	stateFunctions.resize(numStates);
	
	stateNames[FLOATING] = "Floating";
	stateFunctions[FLOATING] = &RunningStateMachine::Floating;
	
	stateNames[DROP] = "Drop";
	stateFunctions[DROP] = &RunningStateMachine::Drop;
	
	stateNames[PRE_HOP] = "Pre Hop";
	stateFunctions[PRE_HOP] = &RunningStateMachine::PreHop;
	
	stateNames[STANCE1] = "Stance 1";
	stateFunctions[STANCE1] = &RunningStateMachine::Stance1;
	
	
	stateNames[FLIGHT1] = "Flight 1";
	stateFunctions[FLIGHT1] = &RunningStateMachine::Flight1;
	
	
	
	/*stateNames[BALANCE_MIDDLE] = "Balance Middle";
	stateFunctions[BALANCE_MIDDLE] = &RunningStateMachine::BalanceMiddle;
	
	stateNames[SQUAT] = "Squat";
	stateFunctions[SQUAT] = &RunningStateMachine::Squat;
	
	stateNames[THRUST] = "Thrust";
	stateFunctions[THRUST] = &RunningStateMachine::Thrust;
	
	stateNames[FLIGHT] = "Flight";
	stateFunctions[FLIGHT] = &RunningStateMachine::Flight;
	
	stateNames[LAND] = "Land";
	stateFunctions[LAND] = &RunningStateMachine::Land;
	
	stateNames[BALANCE_LEFT] = "Balance Left";
	stateFunctions[BALANCE_LEFT] = &RunningStateMachine::BalanceLeft;
	
	stateNames[KICK] = "Kick";
	stateFunctions[KICK] = &RunningStateMachine::Kick;
	
	stateNames[RAISE_FOOT] = "Raise Foot";
	stateFunctions[RAISE_FOOT] = &RunningStateMachine::RaiseFoot;*/
	
	
	ComTrajectory.setSize(3);
	//aComDes.resize(3);
	footServos.resize(NS);
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
		
		footServos[i] = GLOBAL_SERVO;
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
	
	state = FLOATING;
	transitionFlag = true;
	
	// 2.5 m/s
	/*touchDownAngle = 0.345209;
	touchDownLength = .93;
	legSpringConstant = 13247.243210;
	maxSLIPHeight = 0.900325;
	forwardVelocity = 2.600000;
	
	stanceTime = .255;
	flightTime = 0.1434;*/
	
	
	// 3.5 m/s
	touchDownAngle = 0.396459;
	touchDownLength = .98;
	legSpringConstant = 12854.483189;
	maxSLIPHeight = 0.923557;
	forwardVelocity = 3.500000;
	
	stanceTime = .255;
	flightTime = 0.1434;
	
	// 3.5 m/s fast cadence
	/*touchDownAngle = 0.316856;
	touchDownLength = .98;
	legSpringConstant = 20372.460409;
	maxSLIPHeight = 0.966499;
	forwardVelocity = 3.5;
	stanceTime = 0.180000;
	flightTime = .17;
	
	
	// 4.4 m/s
	touchDownAngle=	0.429390;
	legSpringConstant = 	11994.503323;
	forwardVelocity = 	4.400000;
	maxSLIPHeight = 	0.969610;
	touchDownLength = 	1.050000;
	stanceTime = 	0.205000;
	flightTime = 	0.110800;
	
	// 6 m/s
	touchDownAngle=	0.320645;
	legSpringConstant = 	24445.777807;
	forwardVelocity = 	4.400000;
	maxSLIPHeight = 	0.975204;
	touchDownLength = 	1.000000;
	stanceTime = 	0.146300;
	flightTime = 	0.146300;*/
	
	
	
	/*touchDownAngle = 0;
	touchDownLength = .85;
	legSpringConstant = 13275.962156;
	maxSLIPHeight = 0.960249;
	forwardVelocity = 0;*/
	
	restWidth = .13;
	stepWidth = .05;
	vDesDisplay = forwardVelocity;
	
	FILE * fid = fopen("SlipData.txt","r");
	SlipData.resize(51);
	for(int i=0;i<51;i++) {
		SlipData[i].resize(7);
		int scans = fscanf(fid, "%lf%lf%lf%lf%lf%lf%lf",&SlipData[i][0],&SlipData[i][1],&SlipData[i][2]
						,&SlipData[i][3],&SlipData[i][4],&SlipData[i][5],&SlipData[i][6]);
		
		//cout << "Scans = " << scans << "v=" << SlipData[i][0] << endl;
	}
	fclose(fid);
	
	//cout << "SlipData[2][4] " << SlipData[2][4] << endl;
	
	
	//exit(-1);
}

void RunningStateMachine::Floating() {
		
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
					//om << 0,M_PI/20,0;
					om << 0,M_PI/10,0*0;
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
		
		
		
		
		
		
		
		footServos[0] = COM_SERVO;
		footServos[1] = COM_SERVO;
		
		
		//pDesFoot[0] << 2, 1.87,.45;
		//pDesFoot[1] << 2, 2.13,.45;
		pDesFoot[0] << -touchDownLength*sin(touchDownAngle), -restWidth,-touchDownLength*cos(touchDownAngle)+.25;
		pDesFoot[1] << touchDownLength*sin(touchDownAngle), stepWidth,-touchDownLength*cos(touchDownAngle);
		
		//pDesFoot[0] = pFoot[0];
		//pDesFoot[0](2) += .1;
		//pDesFoot[1] = pFoot[1];
		//pDesFoot[1](2) += .1;
		
		kpFoot[0]=50;
		kdFoot[0]=2*sqrt(50);
		
		kpFoot[1]=50;
		kdFoot[1]=2*sqrt(50);
		
	}
	
	//R = [1 0 0;0 0 1;0 -1 0]';
	//R2 = expm(cross([0 pi/2 0]));
	//R3 = expm(cross([-.2 0 0]));
	//quat = RtoQuat(R3*R2*R);
	
	
	//OptimizationSchedule.segment(6,3).setConstant(-1);
	OptimizationSchedule.head(6).setConstant(-1);
	OptimizationSchedule.tail(12).setConstant(1);
	
	kpCM = 30*0;
	kdCM = 2*sqrt(kpCM)*0;
	kdAM = 25*0;
	pComDes << 2.00, 2.0, .48*1.7;
	//pComDes << 2.00, 2.0, .72;
	vComDes.setZero();
	aComDes.setZero();
	kDotDes.setZero();
	kComDes.setZero();
	
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	vDesFoot[0].setZero();
	vDesFoot[1].setZero();
	
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	
	if(stateTime > 2 && stateTime <3)
	{
	
		Float fbQ[7], fbQd[7];
		artic->getLink(0)->getState(fbQ, fbQd);
		//cout << "Orig State " << fbQ[6] << endl;
		
		fbQ[6] += maxSLIPHeight - pCom(2);
		artic->getLink(0)->setState(fbQ, fbQd);
		//cout << "Set State " << fbQ[6] << endl;
		simThread->G_integrator->synchronizeState();
	}
	static bool startFlag = true;
	if(stateTime > 3 && startFlag)
	{
		startFlag = false;
		CartesianVector a_g = {0,0,-9.8};
		environment->setGravity(a_g);
		
		Float fbQ[7], fbQd[7];
		artic->getLink(0)->getState(fbQ, fbQd);
		fbQd[0] = 0; 
		fbQd[1] = 0;
		fbQd[2] = 0;
		fbQd[3] = forwardVelocity;
		fbQd[4] = 0;
		fbQd[5] = 0;
	
		artic->getLink(0)->setState(fbQ, fbQd);
		for (int i=1; i<artic->getNumLinks(); i++) {
			artic->getLink(i)->getState(fbQ, fbQd);
			fbQd[0]=0; fbQd[1]=0; fbQd[2]=0;
			artic->getLink(i)->setState(fbQ, fbQd);
		}
		
		
		simThread->G_integrator->synchronizeState();
		ControlInit();
		
		cout << " vCOM init " << vCom.transpose() << endl;
		cout << " pCom init " << pCom.transpose() << endl;
		cout << " pfoot init" << pFoot[1].transpose() << endl;
		state = PRE_HOP;
		transitionFlag = true;
		frame->logDataCheckBox->SetValue(true);
		
		return;
	}
	
	transitionFlag = false;
}

void RunningStateMachine::Drop() {
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
		state = PRE_HOP;
		transitionFlag = true;
		return;
	}
	transitionFlag = false;
}

void RunningStateMachine::PreHop()
{
	stanceLeg = 1;
	flightLeg = 0;
	
	cout << "vCom = " << vCom.transpose() << "h " << pCom(2) << " hfs " << pFoot[0](2) << "/ " << contactState[0] << ", " << pFoot[1](2) << "/" << contactState[1] << endl;
	
	//Retain gains from flight before
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	
	//Assign feet to no load
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	
	if (pFoot[0](2) < .01 || pFoot[1](2) < .01) 
	{
		
		cout << "Touchdown" << endl;
		transitionFlag=true;
		state=STANCE1;
		return;
	}
	
	
	transitionFlag = false;
}
void RunningStateMachine::Stance1()
{
	static CubicSplineTrajectory flightFootSpline(3);
	static CubicSplineTrajectory verticalSpline(3);
	
	static Float initHeight = 0;
	if (transitionFlag) {
		
		//Initilize right foot spline
		{
			VectorXF startPos(3), endPos(3), startVel(3), endVel(3);
			startPos = pFoot[flightLeg] - pCom;
			startVel = vFoot[flightLeg].tail(3) - vCom;
			int flightYSign = (flightLeg == 0 ? -1 : 1);
			
			endPos << touchDownLength*sin(touchDownAngle)*3./4., stepWidth * flightYSign, startPos(2);
			endVel.setZero();
			flightFootSpline.init(startPos, startVel, endPos, endVel, stanceTime*1.1);
			
		}
		
		// Initialize SLIP and LIP models
		{
			//simThread->idt/=10;
			//simThread->cdt/=10;
			
			LIP.pos = pCom.head(2);
			LIP.vel = vCom.head(2);
			LIP.g = 9.8;
			LIP.height = pCom(2);
			LIP.anchor = pFoot[stanceLeg].head(2);
			
			
			LIP.pos(1) = 2;
			LIP.anchor(1) = 2;
			
			SLIP.pos(0) = pCom(0);
			SLIP.pos(1) = pCom(2);
			
			SLIP.anchor(0) = pFoot[stanceLeg](0);
			SLIP.anchor(1) = 0;
			
			SLIP.pos(0) = SLIP.anchor(0) - touchDownLength*sin(touchDownAngle);
			SLIP.pos(1) = SLIP.anchor(1) + touchDownLength*cos(touchDownAngle);
			
			SLIP.vel(0) = forwardVelocity;
			SLIP.vel(1) = -sqrt(2*9.8*(maxSLIPHeight-touchDownLength*cos(touchDownAngle)));
			
			
			Vector2F relPos = SLIP.pos - SLIP.anchor;
			
			
			
			SLIP.restLength = relPos.norm();
			SLIP.mass = totalMass;
			SLIP.springConst = legSpringConstant;
			
			//SLIP.vel(0) = vCom(0);
			//SLIP.vel(1) = vCom(2);
			
			initHeight = SLIP.pos(1);
			
			cout << "relPos\t" << -relPos.transpose() << endl;
			cout << "vSLIP\t" << SLIP.vel.transpose() << endl;
			cout << "vCOM\t" << vCom.transpose() << endl;
		}
		
		
		/*cout << pCom << endl;
		pComDes = pCom;
		pComDes(0) +=.25;
		pComDes(2) = .79; 
		vComDes.setZero();
		aComDes.setZero();*/
		transitionFlag = false;
	}
	
	SLIP.dynamics();
	LIP.dynamics();
	
	pComDes << SLIP.pos(0), LIP.pos(1), SLIP.pos(1);
	vComDes << SLIP.vel(0), LIP.vel(1), SLIP.vel(1);
	aComDes << SLIP.acc(0), LIP.acc(1), SLIP.acc(1);
	
	for (int i=0; i<100; i++) {
		SLIP.integrate(simThread->cdt/100.);
		LIP.integrate(simThread->cdt/100.);
		if(SLIP.pos(1) >= initHeight)
			break;
	}
	OptimizationSchedule.segment(0,3).setConstant(4);
	TaskWeight.segment(0,3).setConstant(200/10.);
	TaskWeight.segment(2,1).setConstant(90.);
	TaskWeight.segment(0,1).setConstant(200/10.);
	
	kpCM = 150;
	kdCM = 2*sqrt(kpCM);
	kdAM = 25;
	
	
	VectorXF p(3), pd(3), pdd(3);
	
	
	//kpCM = 15;
	//kdCM = 25;
	
	aDesFoot[flightLeg].setZero();
	
	AssignFootMaxLoad(flightLeg,0);
	
	kpFoot[flightLeg]=50;
	kdFoot[flightLeg]=2*sqrt(50);
	
	flightFootSpline.eval(stateTime, p,pd, pdd);
	pDesFoot[flightLeg] = p;
	vDesFoot[flightLeg].tail(3) = pd;
	aDesFoot[flightLeg].tail(3) = pdd;
	
	kpFoot[stanceLeg] = 0;
	kdFoot[stanceLeg] = 0;
	aDesFoot[stanceLeg].setZero();
	
	static bool slowFlag = true;
	if (SLIP.pos(1) >= initHeight &&slowFlag) {
		
		cout << "vCom\t" << vCom.transpose() << endl;
		cout << "vSLIP\t" << SLIP.vel.transpose() << endl;
		
		Vector2F pos = SLIP.pos - SLIP.anchor;
		cout << "Pos\t" << pos.transpose() << endl;
		
		pDesFoot[0] = pFoot[0]-pCom;
		pDesFoot[1] = pFoot[1]-pCom;
		
		//simThread->idt/=100000;
		
		state = FLIGHT1;
		transitionFlag = true;
		
		flightLeg = 1-flightLeg;
		stanceLeg = 1-stanceLeg;
		
		return;
	}
	transitionFlag = false;
	
}
void RunningStateMachine::Stance2()
{
	
	
	
}
void RunningStateMachine::Flight1()
{
	static CubicSplineTrajectory flightFootSpline(3);
	static CubicSplineTrajectory stanceFootSpline(3);
	static CubicSplineTrajectory verticalSpline(3);
	
	static bool firstHalf = true;
	
	int flightYSign = (flightLeg == 0 ? -1 : 1);
	int stanceYSign = (stanceLeg == 0 ? -1 : 1);
	
	
	if(transitionFlag) {
		//bool foundVel = false;
		/*int i=0;
		while(1 == 1)
		{
			float Diff = SlipData[i][0]-vDesDisplay;
			if (Diff*Diff < 1e-6 ) {
				break;
			}
			i++;
		}

		forwardVelocity = SlipData[i][0];
		stanceTime = SlipData[i][1];
		flightTime = SlipData[i][2];
		legSpringConstant = SlipData[i][3];
		touchDownAngle = SlipData[i][4];
		maxSLIPHeight = SlipData[i][5];
		touchDownLength = SlipData[i][6];*/
		
		
		
		firstHalf = true;
		VectorXF startPos(3),endPos(3),startVel(3), endVel(3);
		
		startPos = pFoot[stanceLeg] - pCom;
		endPos << touchDownLength*sin(touchDownAngle), stepWidth * stanceYSign,-touchDownLength*cos(touchDownAngle);
		startVel = vFoot[stanceLeg].tail(3) - vCom;
		endVel.setZero();
		stanceFootSpline.init(startPos, startVel, endPos, endVel, flightTime);
		
		startPos = pFoot[flightLeg] - pCom;
		endPos << -touchDownLength*sin(touchDownAngle), restWidth * flightYSign,-touchDownLength*cos(touchDownAngle)+.25;
		startVel.setZero();
		endVel.setZero();
		flightFootSpline.init(startPos, startVel, endPos, endVel, flightTime);
		
		/*startPos.setZero();
		endPos.setZero();
		endPos(2) = .2;
		startVel.setZero();
		endVel.setZero();
		
		verticalSpline.init(startPos,startVel,endPos,endVel,flightTime/2.);
		firstHalf = true;*/
	}
	
	if(firstHalf && vCom(2) < 0)
	{
		vActDisplay   = vCom(0);
		firstHalf = false;
	}
	
	
	/*if ((stateTime > flightTime/2.) && firstHalf) {
		
		VectorXF startPos(3),endPos(3),startVel(3), endVel(3);
		startPos.setZero();
		endPos.setZero();
		startVel.setZero();
		endVel.setZero();
		verticalSpline.reInit(stateTime, endPos, endVel, flightTime/2.);
		firstHalf = false;
	}*/
	
	
	vDesFoot[0].setZero();
	vDesFoot[1].setZero();
	
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	
	//Assign feet to no load
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	
	/*if (pFoot[0](2) > .01 && pFoot[1](2) > .01) {
		pDesFoot[0] << 0.254750, -.13,-0.894429;
		pDesFoot[1] << 0.254750, .13,-0.894429;
	}*/
	
	
	
	VectorXF p(3), pd(3), pdd(3);
	
	
	
	
	flightFootSpline.eval(stateTime, p,pd, pdd);
	
	pDesFoot[flightLeg] = p;
	vDesFoot[flightLeg].tail(3) = pd;
	aDesFoot[flightLeg].tail(3) = pdd;
	
	
	stanceFootSpline.eval(stateTime, p,pd, pdd);
	
	pDesFoot[stanceLeg] = p;
	vDesFoot[stanceLeg].tail(3) = pd;
	aDesFoot[stanceLeg].tail(3) = pdd;
	
	//pDesFoot[0] = pFoot[0];
	//pDesFoot[0](2) += .1;
	//pDesFoot[1] = pFoot[1];
	//pDesFoot[1](2) += .1;
	
	kpFoot[0]=50;
	kdFoot[0]=2*sqrt(50);
	
	kpFoot[1]=50;
	kdFoot[1]=2*sqrt(50);
	
	
	transitionFlag = false;
	
	OptimizationSchedule.head(6).setConstant(-1);
	OptimizationSchedule.tail(12).setConstant(1);
	
	if ( (pFoot[0](2) < .01 || pFoot[1](2) < .01) && stateTime > .05) {
		cout << "Touchdown" << endl;
		transitionFlag=true;
		state=STANCE1;
		return;
	}
}

void RunningStateMachine::Flight2()
{
	
	
}



void RunningStateMachine::StateControl(ControlInfo & ci)
{
	dmTimespec controlStart, controlEnd;
	dmGetSysTime(&controlStart);
	
	this->HumanoidController::ControlInit();
	
	OptimizationSchedule.setZero(6+NJ+6+12);
	OptimizationSchedule.segment(0,6).setConstant(3);
	//OptimizationSchedule.segment(0,3).setConstant(3);
	OptimizationSchedule.segment(0,6).setConstant(4);
	
	
	//OptimizationSchedule.segment(0,6).setConstant(2);
	OptimizationSchedule.segment(6,NJ+6).setConstant(4);
	//OptimizationSchedule.segment(6,NJ+6).setConstant(2);
	OptimizationSchedule.segment(6+3,3).setConstant(-1);
	//OptimizationSchedule.tail(12).setConstant(0);
	OptimizationSchedule.tail(12).setConstant(2);
	
	//taskOptimActive.setOnes(6+NJ+6+12);
	//taskConstrActive.setZero(6+NJ+6+12);
	//taskOptimActive.tail(12).setZero();
	//taskConstrActive.tail(12).setOnes();
	
	TaskWeight.setOnes(6+NJ+6+12);
	
	TaskWeight.segment(0,3).setConstant(10/8.);
	TaskWeight.segment(3,3).setConstant(100/4.);
	
	
	TaskWeight.segment(0,3).setConstant(200/10.);
	TaskWeight.segment(0,1).setConstant(200/600.);
	
	
	
	TaskWeight.tail(12).setConstant(1000);
	
	
	//TaskWeight.segment(taskRow  ,6).setConstant(10);
	//TaskWeight.segment(taskRow+3,3).setConstant(1000);
	
	
	/*cout << "===========================" << endl;
	
	Vector3F pRel = pFoot[0] - pCom;
	cout << "pCom = " << pCom.transpose() << endl;
	
	cout << "p_0f    = " << pFoot[0].transpose() << endl;
	cout << "p_0/COM = " << pRel.transpose() << endl;
	pRel = pFoot[1] - pCom;
	cout << "p_1f    = " << pFoot[1].transpose() << endl;
	cout << "p_1/COM = " << pRel.transpose() << endl;*/
	
	
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
				wAnk  = .1;
				
				kpKnee = 240;
				wKnee   = .5;
				
				kpHip = 120/3.;
				wHip  = .1;
				
				kpShould = 220;
				wShould  = 10.0/2.;
				
				kpElbow = 240;
				wElbow  = 10;
				
				kpTorso = 120.;
				wTorso = 10.;
				
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
					
					if (state == FLOATING) {
						OptimizationSchedule.segment(taskRow+18,3).setConstant(1);
						OptimizationSchedule.segment(taskRow+21,1).setConstant(1);
						OptimizationSchedule.segment(taskRow+22,3).setConstant(1);
						OptimizationSchedule.segment(taskRow+25,1).setConstant(1);
					}
					
					/////////////////////////////////////////////////////////
				}
				
				
				
			}
			
			{
				Float angle, rate;
				Vector3F relPos = pFoot[1] - pCom;
				Vector3F relVel = vFoot[1].tail(3) - vCom;
				relVel.setZero();
				angle = atan(-relPos(0)/relPos(2));
				
				if(abs(relVel(2)) > 1e-4) {
					rate = 1./(1+pow(-relPos(0)/relPos(2),2))*-relVel(0)/relVel(2);
				}
				else {
					rate = 0;
				}
				
				
				Matrix3F R1, R2,dR2,ddR2, R3, RDot, Rddot;
				Vector3F angAx,omega, omegaDot,rateDes;
				
				R1 << 1,0,0,0,0,-1,0,1,0;
				angAx << 0,M_PI/2 - angle+.25,0;
				matrixExpOmegaCross(angAx, R2);
				omega << 0, rate, 0;
				
				dR2 = cr3(omega)*R2;
				ddR2 = cr3(omegaDot)*R2 + cr3(omega)*dR2;
				
				angAx<< -.2,0,0;
				matrixExpOmegaCross(angAx, R3);
				
				RDesJoint[10] = R3*R2*R1;
				RDot = R3*dR2*R1;
				Rddot = R3*ddR2*R1;
				crossExtract(RDot*RDesJoint[10].transpose(),rateDes);
				rateDesJoint[10] = rateDes;
				accDesJoint[10].setZero(3);
				//cout << "Ra = " << angle << " Rv " << rate << endl;
				
				
				relPos = pFoot[0] - pCom;
				relVel = vFoot[0].tail(3) - vCom;
				relVel.setZero();
				angle = atan(-relPos(0)/relPos(2));
				
				if(abs(relVel(2)) > 1e-4) {
					rate = 1./(1+pow(-relPos(0)/relPos(2),2))*-relVel(0)/relVel(2);
				}
				else {
					rate = 0;
				}
				
				//cout << "la = " << angle << " lv " << rate << endl;
				
				angAx << 0,M_PI/2 - angle+.25,0;
				matrixExpOmegaCross(angAx, R2);
				omega << 0, rate, 0;
				dR2 = cr3(omega)*R2;
				
				angAx<< .2,0,0;
				matrixExpOmegaCross(angAx, R3);
				
				RDesJoint[12] = R3*R2*R1;
				RDot = R3*dR2*R1;
				
				crossExtract(RDot*RDesJoint[12].transpose(),rateDes);
				
				rateDesJoint[12] = rateDes ;
				accDesJoint[12].setZero(3) ;
				
				
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
						TaskWeight.segment(taskRow,3).setConstant(70);
						TaskWeight.segment(taskRow,1).setConstant(70/4.);
						TaskWeight.segment(taskRow+1,1).setConstant(90);
						TaskWeight.segment(taskRow+2,1).setConstant(70/5.);
						
						/*if (state == STANCE1) {
							TaskWeight.segment(taskRow,3).setConstant(180);
						}*/
						//taskOptimActive.segment(taskRow+3,3).setZero();
					}
					else if (bodyi->dof == 1) {
						TaskBias(taskRow+jointIndex) = Kp * (posDesJoint[i](0) - q(jointIndexDm)) 
						+ Kd * (rateDesJoint[i](0) - qd(jointIndex));
						
						if (i==3 || i==7) {
							//cout << "Joint " << i << " = " << q(jointIndexDm) << " dot=" << qd(jointIndex) << endl;
							if (q(jointIndexDm) < .05 && qd(jointIndex)<0)  {
								//TaskBias(taskRow+jointIndex) = - pow(qd(jointIndex),2)/(2*q(jointIndexDm))*1.5;
								//OptimizationSchedule(taskRow+jointIndex) = 1;
								//cout << "Joint limit for " << i << endl;
								int dummyVar;
								//cin >> dummyVar;
							}
						}
					}
					//else if (jointIndex < 18) {
				//		TaskWeight.segment(taskRow+jointIndex,jointDof).setConstant(.1);
						//taskOptimActive.segment(taskRow+jointIndex,jointDof).setZero();
					//}
					
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
						
						//Vector3F ve = rateDesJoint[i].head(3) - omega;
					
						//cout << "Joint " << i << "e = " << eOmega.transpose() << " ve = " << ve.transpose() << endl;
						
						// Note: Velocity bias accel is omega cross omega
						Vector3F wDes = i_R_pi_mat*rateDesJoint[i].head(3);
						Vector3F wdotDes = i_R_pi_mat*accDesJoint[i].head(3);
						
						TaskBias.segment(taskRow+jointIndex,3) = (i_R_pi_mat*(Kp *eOmega) + Kd * (wDes - omega) + wdotDes - omega.cross(wDes) );
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
				
				if (footServos[i] == COM_SERVO) {
					aCom.head(3) += aDesFoot[i].head(3) + kdFoot[i] * (vDesFoot[i].head(3) - vFoot[i].head(3));
					aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - (pFoot[i]-pCom));
					aCom.tail(3) += aDesFoot[i].tail(3) + kdFoot[i] * (vDesFoot[i].tail(3) - (vFoot[i].tail(3) - vCom));
				}
				else {
					aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - pFoot[i]);
					aCom += aDesFoot[i] + kdFoot[i] * (vDesFoot[i] - vFoot[i]);
				}

				
				
				if (kdFoot[i] == 0) {
					aCom = - 20 * vFoot[i];
					//aCom.setZero();
				}
				
				
				// Compute Task Information
				X.block(0,0,3,3) = RFoot[i]; 
				X.block(3,3,3,3) = RFoot[i];
				X.block(3,0,3,3) = -RFoot[i]*cr3(pFootCenter[i]);
				
				artic->computeJacobian(jointIndex,X,footJacs[i]);
				computeAccBiasFromFwKin(link->link_val2,pFootCenter[i],footBias[i]);
				
				// Load Task Information
				TaskJacobian.block(taskRow,0,6,NJ+6) = footJacs[i];
				TaskBias.segment(taskRow,6)          = aCom - footBias[i];
				
				if (footServos[i] == COM_SERVO) {
					TaskJacobian.block(taskRow+3,0,3,NJ+6) -= TaskJacobian.block(3,0,3,NJ+6)/totalMass;
					TaskBias.segment(taskRow+3,3) += cmBias.tail(3)/totalMass;
				}
				
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
		
		
		//cout << "Control Complete " << simThread->sim_time << endl;
		//exit(-1);
		
	}
	else {
		tau.setZero(26);
		qdd.setZero(26);
		qddA.setZero(26);
		fs.setZero(12);
	}
	
	/*if(state == STANCE1)
	{
		VectorXF jointInput(10*NJ);
		jointInput.setZero();
		artic->setJointInput(jointInput.data());
		
	}*/
	
	
	
	if (frame->logDataCheckBox->IsChecked()) {
		logData();
	}
	
	
	stateTime += simThread->cdt;
}