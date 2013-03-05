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
#include "CoordinatedCubicSpline.h"

//const Float liftHeight1 = .30;
const Float liftHeight1 = .30;
const Float dropHeight2 = .18;
const Float reverseOffset = .04;
const Float angleFactor = 1;

const Float kpFootRotGlobal = 250;
const Float kpFootGlobal    = 50;

const Float GRAVITY = 9.8;

const bool useRuleBase = true;
const bool useFeedback = true;

const bool transitionPauses = false;
const bool reportSchedule = false;


RunningStateMachine::RunningStateMachine(dmArticulation * robot) 
: HumanoidDataLogger(robot, NUM_RUN_STATES), ComTrajectory(3)
{
	stepNum = 0;
	pushActive = false;
	pushRequest = false;
	
	//SlipModel s;
	//s.optimize();
	//exit(-1);
	
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
	kpFootRot.resize(NS);
	kdFootRot.resize(NS);
	
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
	
	
	// 3.5 m/s test 3 (and transition controller)
	/*thisStep.touchDownAngle1=	0.280134;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	26745.050005;
	thisStep.vx0 = 	3.500000;
	thisStep.vy0 = 	0.227812;
	thisStep.h0 = 	0.977582;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.157500;
	thisStep.flightTime = 	0.192500;
	thisStep.stepWidth = 	0.149027;*/
	
	/*thisStep.touchDownAngle1=	0.280977;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	25332.191541;
	thisStep.vx0 = 	3.500000;
	thisStep.vy0 = 	0.228157;
	thisStep.h0 = 	0.977356;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.157500;
	thisStep.flightTime = 	0.192500;
	thisStep.footLength = 	0.139000;
	thisStep.CoPInitOffset = 	-0.069500;
	thisStep.CoPVel    = 	0.882540;
	thisStep.stepWidth = 	0.149060;*/
	
	thisStep.touchDownAngle1=	0.402427;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	12671.814106;
	thisStep.vx0 = 	3.500000;
	thisStep.vy0 = 	0.228576;
	thisStep.h0 = 	0.910240;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.225873;
	thisStep.flightTime = 	0.120308;
	
	
	/*thisStep.touchDownAngle1=	0.426134;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	14455.142824;
	thisStep.vx0 = 	4.500000;
	thisStep.vy0 = 	0.220399;
	thisStep.h0 = 	0.907485;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.183809;
	thisStep.flightTime = 	0.140642;
	
	
	
	thisStep.touchDownAngle1=	0.445964;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	15743.937369;
	thisStep.vx0 = 	5.500000;
	thisStep.vy0 = 	0.206071;
	thisStep.h0 = 	0.899709;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.155920;
	thisStep.flightTime = 	0.141643;
	
	
	thisStep.touchDownAngle1=	0.463120;
	thisStep.touchDownAngle2=	0.000000;
	thisStep.k = 	16423.631975;
	thisStep.vx0 = 	6.500000;
	thisStep.vy0 = 	0.188708;
	thisStep.h0 = 	0.889348;
	thisStep.touchDownLength = 	0.970000;
	thisStep.stanceTime = 	0.135960;
	thisStep.flightTime = 	0.132557;
	*/
	
	
	thisStep.k1 = thisStep.k;
	thisStep.k2 = thisStep.k;
	
	
	
	
	thisStep.CoPInitOffset = 	-0.069500*0;
	thisStep.CoPVel    = 	0.882540*0;
	
	
	hipWidth = 0.2542032;
	
	pTOFData = fopen("tofData.txt","w");
	
	vDesDisplay = thisStep.vx0;
	vActDisplay = thisStep.vx0;
	transitionController = false;
	velocityTest = false;
	
	footAngles.setZero(7);
	//footAngles << 0, M_PI/4, M_PI/2, 0, 0, 0, 0;
	footAngles << 0, M_PI/4, M_PI/2, M_PI/3, M_PI/6, -M_PI/40, 0;
	
	
	footAngles*=angleFactor;
	
	FILE * pFile = fopen("../matlab/3DSLIP/SlipRuleBaseInfo.txt","r");
	int numRules;
	fscanf(pFile, "%d",&numRules);
	fclose(pFile);
	
	cout << numRules << " rules " << endl;
	
	//exit(-1);
	
	pFile = fopen("../matlab/3DSLIP/SlipRuleBase.txt","r");
	
	ruleBase.resize(numRules);
	for (int i=0; i<numRules; i++) {
		fscanf(pFile, "%lf %lf %lf",&ruleBase[i].vx0,&ruleBase[i].k,&ruleBase[i].touchDownAngle1);
		fscanf(pFile, "%lf %lf %lf",&ruleBase[i].vy0,&ruleBase[i].h0,&ruleBase[i].touchDownLength);
		ruleBase[i].touchDownAngle2 = 0;
		ruleBase[i].CoPInitOffset = 0;
		ruleBase[i].CoPVel     = 0;
		
		ruleBase[i].k*=1e4;
		
		fscanf(pFile, "%lf %lf",&ruleBase[i].stanceTime,&ruleBase[i].flightTime);
		ruleBase[i].feedBack.resize(3,3);
		for (int j=0; j<3;j++) {
			for (int k=0; k<3; k++) {
				fscanf(pFile, "%lf ",&ruleBase[i].feedBack(j,k) );
			}
		}
		cout << "vx = " << ruleBase[i].vx0 << endl;
		cout << "K = " << endl << ruleBase[i].feedBack << endl;
	}
	
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
		initRDesFoot = RDesFoot[0];
		
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
					om << 0,M_PI/15,0*0;
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
		
		pDesFoot[0] << -thisStep.touchDownLength*sin(thisStep.touchDownAngle1), 
							-hipWidth/2,-thisStep.touchDownLength*cos(thisStep.touchDownAngle1)+liftHeight1;
		pDesFoot[1] << thisStep.touchDownLength*sin(thisStep.touchDownAngle1), 
							 hipWidth/2,-thisStep.touchDownLength*cos(thisStep.touchDownAngle1);
		
		
		kpFoot[0]=kpFootGlobal;
		kdFoot[0]=2*sqrt(kpFootGlobal);
		
		kpFootRot[0] = kpFootRotGlobal;
		kdFootRot[0] = 2*sqrt(kpFootRotGlobal);
		
		
		kpFoot[1]=kpFootGlobal;
		kdFoot[1]=2*sqrt(kpFootGlobal);
		
		kpFootRot[1] = kpFootRotGlobal;
		kdFootRot[1] = 2*sqrt(kpFootRotGlobal);
		
		
	}
	
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
	
	Vector3F angAx;
	angAx << 0, footAngles(2),0;
	matrixExpOmegaCross(angAx, RDesFoot[0]);
	RDesFoot[0] = RDesFoot[0]*initRDesFoot;
	
	//exit(-1);
	
	
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	
	if(stateTime > 2 && stateTime <3)
	{
	
		Float fbQ[7], fbQd[7];
		artic->getLink(0)->getState(fbQ, fbQd);
		//cout << "Orig State " << fbQ[6] << endl;
		
		fbQ[6] += thisStep.h0 - pCom(2);
		artic->getLink(0)->setState(fbQ, fbQd);
		//cout << "Set State " << fbQ[6] << endl;
		simThread->G_integrator->synchronizeState();
	}
	static bool startFlag = true;
	if(stateTime > 3 && startFlag)
	{
		startFlag = false;
		CartesianVector a_g = {0,0,-GRAVITY};
		environment->setGravity(a_g);
		
		Float fbQ[7], fbQd[7];
		artic->getLink(0)->getState(fbQ, fbQd);
		fbQd[0] = 0; 
		fbQd[1] = 0;
		fbQd[2] = 0;
		fbQd[3] = thisStep.vx0;
		fbQd[4] = thisStep.vy0;
		fbQd[5] = 0;
	
		artic->getLink(0)->setState(fbQ, fbQd);
		for (int i=1; i<artic->getNumLinks(); i++) {
			artic->getLink(i)->getState(fbQ, fbQd);
			fbQd[0]=0; fbQd[1]=0; fbQd[2]=0;
			artic->getLink(i)->setState(fbQ, fbQd);
		}
		
		
		simThread->G_integrator->synchronizeState();
		ControlInit();
		
		thisStep.pToF = pCom;
		thisStep.vToF = vCom;
		
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
	if (pFoot[0](2) < .002 || pFoot[1](2) < .002) 
	{
		
		cout << "Touchdown" << endl;
		transitionFlag=true;
		state=STANCE1;
		
		return;
	}
	
	slipSpringEnergy = 0;
	slipKineticEnergy = 0;
	slipPotentialEnergy = 0;
	
	stanceLeg = 1;
	flightLeg = 0;
	
	cout << "vCom = " << vCom.transpose() << "h " << pCom(2) << " hfs " << pFoot[0](2) << "/ " << contactState[0] << ", " << pFoot[1](2) << "/" << contactState[1] << endl;
	
	//Retain gains from flight before
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	
	//Assign feet to no load
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	

	
	
	transitionFlag = false;
}
void RunningStateMachine::Stance1()
{
	static bool bof = false;
	static CubicSplineTrajectory flightFootSpline(3);
	static CoordinatedCubicSpline footAngleSpline;
	
	static Float finishHeight = 0;
	
	static bool firstHalf = true;
	if (transitionFlag) {
		bof= false;
		
		
		
		//Initilize right foot spline
		{
			firstHalf = true;
			VectorXF startPos(3), endPos(3), startVel(3), endVel(3);
			startPos = pFoot[flightLeg] - pCom;
			startVel = vFoot[flightLeg].tail(3) - vCom;
			int flightYSign = (flightLeg == 0 ? -1 : 1);
			
			
			endPos << thisStep.touchDownLength*sin(thisStep.touchDownAngle1), 
							hipWidth/2 * flightYSign, startPos(2)-dropHeight2;

			
			endVel.setZero();
			flightFootSpline.init(startPos, startVel, endPos, endVel, thisStep.stanceTime*1.5);
			
			MatrixXF angles = footAngles.tail(5).transpose();
			VectorXF times(5);
			times << 0, thisStep.stanceTime/2, thisStep.stanceTime, 
			thisStep.stanceTime+thisStep.flightTime/2, 
			thisStep.stanceTime+thisStep.flightTime;
			
			VectorXF zeroRate(1),initRate(1);
			zeroRate(0) = 0;
			initRate(0) = vFoot[flightLeg](1);
			footAngleSpline.computeCoefficients(times, angles, initRate, zeroRate);
			
		}
		
		// Initialize SLIP and LIP models
		{
			int stanceYSign = (stanceLeg == 0 ? -1 : 1);
			
			SLIP.pos    = pCom;
			
			
			Float stepWidth;
			
			
			if (useFeedback) {
				stepWidth = hipWidth/2 + 
					thisStep.vToF(1)*stanceYSign*sqrt(2*(thisStep.pToF(2) - thisStep.touchDownLength*cos(thisStep.touchDownAngle1)) / GRAVITY) +
					thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*sin(thisStep.touchDownAngle2);
			}
			else {
				stepWidth = hipWidth/2 + 
				thisStep.vy0*sqrt(2*(thisStep.h0 - thisStep.touchDownLength*cos(thisStep.touchDownAngle1)) / GRAVITY) +
				thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*sin(thisStep.touchDownAngle2);
			}

			
			
			SLIP.anchor(0) = pFoot[stanceLeg](0);
			SLIP.anchor(1) = stepWidth*stanceYSign+thisStep.pToF(1);
			SLIP.anchor(2) = 0;
			
			SLIP.pos(0) = SLIP.anchor(0) - thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*cos(thisStep.touchDownAngle2);
			SLIP.pos(1) = SLIP.anchor(1) - stanceYSign*(hipWidth/2. + thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*sin(thisStep.touchDownAngle2));
			SLIP.pos(2) = SLIP.anchor(2) + thisStep.touchDownLength*cos(thisStep.touchDownAngle1);
			
			if (!useFeedback) {
				SLIP.vel(0) = thisStep.vx0;
				SLIP.vel(1) = thisStep.vy0*stanceYSign;
				SLIP.vel(2) = -sqrt(2*9.8*(thisStep.h0-SLIP.pos(2)));
			}
			else {
				SLIP.vel(0) = thisStep.vToF(0);
				SLIP.vel(1) = thisStep.vToF(1);
				SLIP.vel(2) = -sqrt(2*9.8*(thisStep.pToF(2)-SLIP.pos(2)));
				cout << "vTof = " << thisStep.vToF.transpose() << endl;
				cout << "pTof = " << thisStep.pToF.transpose() << endl;
				cout << "vNow = " << vCom.transpose() << endl;
				
				
			}
			
			Vector3F relPos = SLIP.pos - SLIP.anchor;
			SLIP.restLength = relPos.norm();
			SLIP.mass = totalMass;
			SLIP.springConst = thisStep.k1;
			
			
			SLIP.anchor(0) += thisStep.CoPInitOffset;
			SLIP.anchorVel.setZero();
			SLIP.anchorVel(0)+=thisStep.CoPVel;
			
			cout << "K = " << thisStep.k1 << endl;
			cout << "relPos\t" << relPos.transpose() << endl;
			cout << "vSLIP\t" << SLIP.vel.transpose() << endl;
			cout << "vCOM\t" << vCom.transpose() << endl;
			cout << "SLIP Pos \t"<< SLIP.pos.transpose() << endl;
			cout << "pCOM\t" << pCom.transpose() << endl;
			
		}
		
		
		/*cout << pCom << endl;
		pComDes = pCom;
		pComDes(0) +=.25;
		pComDes(2) = .79; 
		vComDes.setZero();
		aComDes.setZero();*/
		transitionFlag = false;
		simThread->paused_flag ^= transitionPauses;
	}
	
	
	SLIP.dynamics();

	
	int stanceYSign = (stanceLeg == 0 ? -1 : 1);
	
	slipSpringEnergy = 1/2. * SLIP.springConst * pow(SLIP.restLength-SLIP.length,2);
	slipKineticEnergy = 1/2. * SLIP.mass * SLIP.vel.dot(SLIP.vel);
	slipPotentialEnergy = SLIP.mass * 9.8 * SLIP.pos(2);

	
	pComDes = SLIP.pos;
	vComDes = SLIP.vel;
	aComDes = SLIP.acc;
	
	/*cout << "pComDes = " << pComDes.transpose() << endl;
	cout << "pCom    = " << pCom.transpose() << endl;
	
	cout << "vComDes = " << vComDes.transpose() << endl;
	cout << "vCom    = " << vCom.transpose() << endl;
	
	cout << "aComDes = " << aComDes.transpose() << endl;*/
	
	
	minfz =totalMass *(aComDes(2)+9.8)*.01;
	if (minfz < 0) {
		minfz = 0;
	}
	UpdateVariableBounds();
	
	
	
	for (int i=0; i<100; i++) {
		SLIP.integrate(simThread->cdt/100.);
		
		if (SLIP.expandRate > 0) {
			SLIP.springConst = thisStep.k2;
		}
		
		if((SLIP.length >= SLIP.restLength) && (SLIP.vel(2)>0))
			break;
	}
	
	
	
	
	//OptimizationSchedule.segment(3,3).setConstant(3);
	OptimizationSchedule.segment(0,3).setConstant(4);
	TaskWeight.segment(0,3).setConstant(200/10.);
	//TaskWeight.segment(2,1).setConstant(90.);
	TaskWeight.segment(0,1).setConstant(200/10.);
	TaskWeight.segment(1,1).setConstant(200/50.);
	
	
	kpCM = 150;
	kdCM = 2*sqrt(kpCM);
	kdAM = 25;
	
	Vector6F hLegs = CentMomMat.block(0,7,6,12) * qd.segment(7,12);
	kComDes(1) = hLegs(1);
	
	
	
	VectorXF p(3), pd(3), pdd(3);
	VectorXF theta(1), thetad(1), thetadd(1);
	

	aDesFoot[flightLeg].setZero();
	
	AssignFootMaxLoad(flightLeg,0);
	
	kpFoot[flightLeg]=kpFootGlobal;
	kdFoot[flightLeg]=2*sqrt(kpFoot[flightLeg]);
	
	kpFootRot[flightLeg]=kpFootRotGlobal;
	kdFootRot[flightLeg]=2*sqrt(kpFootRot[flightLeg]);
	
	flightFootSpline.eval(stateTime, p,pd, pdd);
	pDesFoot[flightLeg] = p;
	vDesFoot[flightLeg].tail(3) = pd;
	aDesFoot[flightLeg].tail(3) = pdd;

	footAngleSpline.eval(stateTime, theta, thetad, thetadd);
	Vector3F angAx;
	angAx << 0 , theta(0) , 0;
	matrixExpOmegaCross(angAx, RDesFoot[flightLeg]);
	vDesFoot[flightLeg].head(3) << 0,thetad(0),0;
	aDesFoot[flightLeg].head(3) << 0,thetadd(0),0;
	RDesFoot[flightLeg] = RDesFoot[flightLeg]*initRDesFoot;
	
	
	kpFoot[stanceLeg] = 0;
	kdFoot[stanceLeg] = 0;
	kpFootRot[stanceLeg]=0;
	kdFootRot[stanceLeg]=0;
	
	aDesFoot[stanceLeg].setZero();
	
	static bool slowFlag = true;
	if((SLIP.length >= SLIP.restLength) && (SLIP.vel(2)>0)) {

		
		cout << "Transition to Flight";
		cout << "vCom\t" << vCom.transpose() << endl;
		cout << "vSLIP\t" << SLIP.vel.transpose() << endl;
		
		Vector3F pos = SLIP.pos - SLIP.anchor;
		cout << "Pos\t" << pos.transpose() << endl;
		
		pDesFoot[0] = pFoot[0]-pCom;
		pDesFoot[1] = pFoot[1]-pCom;
		
		//simThread->idt/=100000;
		
		
		
		state = FLIGHT1;
		transitionFlag = true;
		
		flightLeg = 1-flightLeg;
		stanceLeg = 1-stanceLeg;
		

		//simThread->paused_flag = !simThread->paused_flag;
		
		return;
	}
	transitionFlag = false;
	
}
void RunningStateMachine::Stance2()
{
	
	
	
}
void RunningStateMachine::Flight1()
{
	slipSpringEnergy = 0;
	slipKineticEnergy = 0;
	slipPotentialEnergy = 0;
	static CubicSplineTrajectory flightFootSpline(3);
	static CubicSplineTrajectory stanceFootSpline(3);
	static CubicSplineTrajectory verticalSpline(3);
	static CoordinatedCubicSpline flightAngleSpline, stanceAngleSpline;
	
	static bool firstHalf = true;
	static Float timeSinceToF = 0;
	
	int flightYSign = (flightLeg == 0 ? -1 : 1);
	int stanceYSign = (stanceLeg == 0 ? -1 : 1);
	
	static Float prevvDes = 0;
	timeSinceToF += simThread->cdt;
	
	if(transitionFlag) {
		timeSinceToF = 0;
		
		if (useRuleBase) {
			int ruleIndex = 0;
			for (ruleIndex = 0; ruleIndex < ruleBase.size(); ruleIndex++) {
				if (abs(vDesDisplay - ruleBase[ruleIndex].vx0)<.001) {
					break;
				}
			}
			if (ruleIndex == ruleBase.size()) {
				ruleIndex--;
			}
			
			thisStep.touchDownAngle1=	ruleBase[ruleIndex].touchDownAngle1;
			thisStep.touchDownAngle2=	ruleBase[ruleIndex].touchDownAngle2;
			thisStep.k = 	ruleBase[ruleIndex].k;
			thisStep.vx0 = 	ruleBase[ruleIndex].vx0;
			thisStep.vy0 = 	ruleBase[ruleIndex].vy0;
			thisStep.h0 = 	ruleBase[ruleIndex].h0;
			thisStep.touchDownLength = 	ruleBase[ruleIndex].touchDownLength;
			thisStep.stanceTime = 	ruleBase[ruleIndex].stanceTime;
			thisStep.flightTime = 	ruleBase[ruleIndex].flightTime;
			
			
			
			
			
			
			cout<< "theta1 " << thisStep.touchDownAngle1 << endl;
			cout<< "theta2 " << thisStep.touchDownAngle2 << endl;
			cout<< "k " << thisStep.k << endl;
			cout<< "vx0 " << thisStep.vx0 << endl;
			cout<< "vy0 " << thisStep.vy0 << endl;
			cout<< "h0 " << thisStep.h0 << endl;
			cout<< "L0 " << thisStep.touchDownLength << endl;
			cout<< "stanceTime " << thisStep.stanceTime << endl;
			cout<< "flightTime " << thisStep.flightTime << endl;
			
			
			
			thisStep.k1 = thisStep.k;
			thisStep.k2 = thisStep.k;

			
			// Find State Feedback
			if (useFeedback) {
				VectorXF dParams(3), dState(3);
				
				Float timeToTOF = vCom(2)/GRAVITY;
				Float hToF = pCom(2) + vCom(2)*timeToTOF - 1/2.*GRAVITY*pow(timeToTOF,2);
				
				dState(0) = hToF- thisStep.h0;
				dState(1) = vCom(0) - thisStep.vx0;
				dState(2) = vCom(1)*stanceYSign - thisStep.vy0;
				
				dParams = ruleBase[ruleIndex].feedBack * dState;
				
				thisStep.touchDownAngle1 += dParams(0);
				thisStep.touchDownAngle2 += dParams(1);
				thisStep.k1 += dParams(2)*1e4;
				thisStep.k2 -= dParams(2)*1e4;
			}
			
			simThread->paused_flag ^= transitionPauses;
		}
		
		
		
		
		
		//bool foundVel = false;
		stepNum ++;
		
		firstHalf = true;
		VectorXF startPos(3),endPos(3),startVel(3), endVel(3);
		
		// Flight foot
		startPos = pFoot[flightLeg] - pCom;
		
		endPos << startPos(0)-reverseOffset, 
						startPos(1),-thisStep.touchDownLength*cos(thisStep.touchDownAngle1)+liftHeight1;
		
		startVel.setZero();
		startVel(1) = vFoot[flightLeg](4) -vCom(1);
		
		endVel.setZero();
		endVel(2) = liftHeight1/thisStep.flightTime/2;
		endVel(0) = sin(thisStep.touchDownAngle1)*thisStep.touchDownLength / thisStep.stanceTime/4;
		
		flightFootSpline.init(startPos, startVel, endPos, endVel, thisStep.flightTime);
		
		VectorXF flightTimes(7);
		flightTimes << 0, thisStep.flightTime/2., thisStep.flightTime, thisStep.flightTime+thisStep.stanceTime/2, 
		thisStep.flightTime+thisStep.stanceTime, 
		thisStep.stanceTime+thisStep.flightTime*3/2, 
		thisStep.stanceTime+thisStep.flightTime*2;
		MatrixXF flightAngles = footAngles.tail(7).transpose();
		
		VectorXF zeroRate(1),initRate(1);
		zeroRate(0) = 0;
		initRate(0) = vFoot[flightLeg](1);
		flightAngleSpline.computeCoefficients(flightTimes, flightAngles, initRate, zeroRate);
		
		
		VectorXF pos(1), vel(1), acc(1);
		
		/*FILE * splineData = fopen("testSpline.dat","w");
		for (Float t = 0; t<thisStep.stanceTime+thisStep.flightTime+thisStep.flightTime; t+=.001) {
			
			//flightFootSpline.eval(t, pos, vel, acc);
			flightAngleSpline.eval(t, pos, vel, acc);
			
			fprintf(splineData,"%f %f\n",t,pos(0));
		}
		fclose(splineData);*/
		
		
		
		
		startPos = pFoot[stanceLeg] - pCom;
		endPos << thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*cos(thisStep.touchDownAngle2), 
								(hipWidth/2. + thisStep.touchDownLength*sin(thisStep.touchDownAngle1)*sin(thisStep.touchDownAngle2)) * stanceYSign,
								-thisStep.touchDownLength*cos(thisStep.touchDownAngle1);
		
		startVel = vFoot[stanceLeg].tail(3) - vCom;
		endVel.setZero();
		stanceFootSpline.init(startPos, startVel, endPos, endVel, thisStep.flightTime);
		
		MatrixXF stanceAngles = footAngles.tail(3).transpose();
		VectorXF stanceTimes(3);
		stanceTimes << 0, thisStep.flightTime/2., thisStep.flightTime;
		initRate(0) = vFoot[stanceLeg](1);
		stanceAngleSpline.computeCoefficients(stanceTimes, stanceAngles, initRate, zeroRate);
		
		
	}
	
	if ( (pFoot[0](2) < .002 || pFoot[1](2) < .002 || grfInfo.fCoPs[0](2)>0 || grfInfo.fCoPs[1](2)>0) && stateTime > .05) {
		VectorXF p(3), pd(3), pdd(3);
		stanceFootSpline.eval(stateTime, p,pd, pdd);
		
		
		cout << "Touchdown" << endl;
		transitionFlag=true;
		state=STANCE1;
		cout << "Flight Time " << stateTime << " vs. Desired " << thisStep.flightTime << endl;
		cout << "ToF Time " << timeSinceToF << 
				" vs. Desired " << sqrt(2*(thisStep.pToF(2)-thisStep.touchDownLength*cos(thisStep.touchDownAngle1))/GRAVITY) << endl;
		
		
		cout << "PFoot " << pFoot[stanceLeg].transpose() - pCom.transpose() << " vs des. " << p.transpose() << endl;
		//simThread->paused_flag = !simThread->paused_flag;
		return;
	}
	
	
	if(firstHalf && vCom(2) < 0)
	{
		fprintf(pTOFData,"%.5lf %.5lf %.5lf %.5lf\n",thisStep.vx0, vCom(0), thisStep.h0,pCom(2));
		vActDisplay   = vCom(0);
		thisStep.pToF = pCom;
		thisStep.vToF = vCom;
		timeSinceToF = 0;
		firstHalf = false;
	}
	
	
	// This is for prettier graphs
	aComDes.setZero(3);
	aComDes(2) = -GRAVITY;
	vComDes += .002*aComDes;
	pComDes += .002*vComDes;
	kpCM = 0;
	kdCM = 0;
	
	
	vDesFoot[0].setZero();
	vDesFoot[1].setZero();
	
	aDesFoot[0].setZero();
	aDesFoot[1].setZero();
	
	//Assign feet to no load
	AssignFootMaxLoad(0,0);
	AssignFootMaxLoad(1,0);
	
	
	VectorXF p(3), pd(3), pdd(3);
	VectorXF theta(1), thetad(1), thetadd(1);
	
	
	// Flight Foot
	flightFootSpline.eval(stateTime, p,pd, pdd);
	
	pDesFoot[flightLeg] = p;
	vDesFoot[flightLeg].tail(3) = pd;
	aDesFoot[flightLeg].tail(3) = pdd;
	
	flightAngleSpline.eval(stateTime, theta, thetad, thetadd);
	Vector3F angAx;
	angAx << 0 , theta(0) , 0;
	matrixExpOmegaCross(angAx, RDesFoot[flightLeg]);
	vDesFoot[flightLeg].head(3) << 0,thetad(0),0;
	aDesFoot[flightLeg].head(3) << 0,thetadd(0),0;
	RDesFoot[flightLeg] = RDesFoot[flightLeg]*initRDesFoot;
	
	
	
	// Stance Foot
	stanceFootSpline.eval(stateTime, p,pd, pdd);
	
	pDesFoot[stanceLeg] = p;
	vDesFoot[stanceLeg].tail(3) = pd;
	aDesFoot[stanceLeg].tail(3) = pdd;
	
	stanceAngleSpline.eval(stateTime, theta, thetad, thetadd);
	angAx<< 0 , theta(0) , 0;
	matrixExpOmegaCross(angAx, RDesFoot[stanceLeg]);
	vDesFoot[stanceLeg].head(3) << 0,thetad(0),0;
	aDesFoot[stanceLeg].head(3) << 0,thetadd(0),0;
	RDesFoot[stanceLeg] = RDesFoot[stanceLeg]*initRDesFoot;
	
	
	
	kpFoot[0]=kpFootGlobal;
	kdFoot[0]=2*sqrt(kpFoot[0]);
	
	kpFootRot[0]=kpFootRotGlobal;
	kdFootRot[0]=2*sqrt(kpFootRot[0]);
	
	kpFoot[1]=kpFootGlobal;
	kdFoot[1]=2*sqrt(kpFootGlobal);
	
	kpFootRot[1]=kpFootRotGlobal;
	kdFootRot[1]=2*sqrt(kpFootRot[1]);
	
	
	transitionFlag = false;
	
	OptimizationSchedule.head(6).setConstant(-1);
	OptimizationSchedule.tail(12).setConstant(1);
	
}

void RunningStateMachine::Flight2()
{
	
	
}



void RunningStateMachine::StateControl(ControlInfo & ci)
{
	//frame->transitionController->SetValue(true);
	velocityTest = frame->velocityTest->IsChecked();
	transitionController = frame->transitionController->IsChecked();
	
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
	
	// These are set in the stance state
	//TaskWeight.segment(0,3).setConstant(10/8.);
	TaskWeight.segment(3,3).setConstant(100/4.);
	
	
	/*TaskWeight.segment(0,3).setConstant(200/10.);
	TaskWeight.segment(0,1).setConstant(200/50.);
	TaskWeight.segment(1,1).setConstant(200/5.);*/
	
	
	
	TaskWeight.tail(12).setConstant(1000);
	
	
	//TaskWeight.segment(taskRow  ,6).setConstant(10);
	//TaskWeight.segment(taskRow+3,3).setConstant(1000);
	
	//OptimizationSchedule.segment(3,1).setConstant(3);
	//OptimizationSchedule.segment(5,1).setConstant(3);
	
	
	/*cout << "===========================" << endl;
	
	Vector3F pRel = pFoot[0] - pCom;
	cout << "pCom = " << pCom.transpose() << endl;
	
	cout << "p_0f    = " << pFoot[0].transpose() << endl;
	cout << "p_0/COM = " << pRel.transpose() << endl;
	pRel = pFoot[1] - pCom;
	cout << "p_1f    = " << pFoot[1].transpose() << endl;
	cout << "p_1/COM = " << pRel.transpose() << endl;*/
	
	
	/*if (stateTime == simThread->cdt) {
		
		cout << "Prior to machine State = " << stateNames[state] << endl;
		cout << "Weights = " << endl << TaskWeight << endl;
		cout << "Schedule = " << endl << OptimizationSchedule << endl;
	}*/
	
	
	// Do at least one state call, and more if it transitioned out
	do {
		if (transitionFlag) {
			stateTime = 0;
		}
		(this->*stateFunctions[state])();
	} while (transitionFlag);
	
	if (stateTime == simThread->cdt && reportSchedule) {
		
		cout << "after machine State = " << stateNames[state] << endl;
		cout << "Weights = " << endl << TaskWeight << endl;
		cout << "Schedule = " << endl << OptimizationSchedule << endl;
	}
	
	
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
				
				kpKnee = 120;
				wKnee   = .5;
				
				kpHip = 120;
				wHip  = .1;
				//;//
				kpShould = 420/1.5;///3.;
				wShould  = 10.0*2.6/1.5;///3.;
				
				kpElbow = 240/1;///3.;
				wElbow  = 10*2/1;///3.;
				
				kpTorso = 440.;
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
					
					//if (state == FLOATING) {
					//	OptimizationSchedule.segment(taskRow+18,3).setConstant(1);
					//	OptimizationSchedule.segment(taskRow+21,1).setConstant(1);
					//	OptimizationSchedule.segment(taskRow+22,3).setConstant(1);
					//	OptimizationSchedule.segment(taskRow+25,1).setConstant(1);
					//}
					
					/////////////////////////////////////////////////////////
				}
				
				
				
			}
			
			 {
				 static Float prevAngle1, prevAngle2;
				 
				 Float offset = .2;
				 Float ampFactor = 1.5;
				 
				 offset = -.1;
				 ampFactor = .7;
				 
				 offset = -.1;
				 ampFactor = .85;
				 
				Float angle, rate;
				Vector3F relPos = pFoot[1] - pCom;
				Vector3F relVel = vFoot[1].tail(3) - vCom;
				//relVel.setZero();
				angle = atan(-relPos(0)/relPos(2));
				
				//if(abs(relVel(2)) > 1e-4) {
					rate = (-relVel(0)*relPos(2) + relVel(2)*relPos(0))/(pow(relPos(0),2)+pow(relPos(2),2));
				//}
				//else {
				//	rate = 0;
				//}
				angle*=ampFactor;
				rate*=ampFactor;
				
				Matrix3F R1, R2,dR2,ddR2, R3, RDot, Rddot;
				Vector3F angAx,omega, omegaDot,rateDes;
				
				R1 << 1,0,0,0,0,-1,0,1,0;
				angAx << 0,M_PI/2 - angle-offset,0;
				matrixExpOmegaCross(angAx, R2);
				omega << 0, -rate, 0;
				
				dR2 = cr3(omega)*R2;
				ddR2 = cr3(omegaDot)*R2 + cr3(omega)*dR2;
				
				angAx<< -.3,0,0;
				matrixExpOmegaCross(angAx, R3);
				
				RDesJoint[10] = R3*R2*R1;
				RDot = R3*dR2*R1;
				Rddot = R3*ddR2*R1;
				crossExtract(RDot*RDesJoint[10].transpose(),rateDes);
				rateDesJoint[10] = rateDes;
				accDesJoint[10].setZero(3);
				//cout << "Ra = " << angle << " Rv " << rate << endl;
				 
				// cout << "t1 " << angle << endl;
				 //cout << "r1 " << rate << " v " << (angle-prevAngle1)/simThread->cdt << endl;
				 prevAngle1 = angle;
				 
				
				relPos = pFoot[0] - pCom;
				relVel = vFoot[0].tail(3) - vCom;
				//relVel.setZero();
				angle = atan(-relPos(0)/relPos(2));
				
				//if(abs(relVel(2)) > 1e-4) {
					rate = (-relVel(0)*relPos(2) + relVel(2)*relPos(0))/(pow(relPos(0),2)+pow(relPos(2),2));
				//}
				//else {
				//	rate = 0;
				//}
				angle*=ampFactor;
				rate*=ampFactor;
				
				//cout << "la = " << angle << " lv " << rate << endl;
				
				angAx << 0,M_PI/2 - angle-offset,0;
				matrixExpOmegaCross(angAx, R2);
				omega << 0, -rate, 0;
				dR2 = cr3(omega)*R2;
				
				angAx<< .3,0,0;
				matrixExpOmegaCross(angAx, R3);
				
				RDesJoint[12] = R3*R2*R1;
				RDot = R3*dR2*R1;
				
				crossExtract(RDot*RDesJoint[12].transpose(),rateDes);
				
				rateDesJoint[12] = rateDes ;
				accDesJoint[12].setZero(3) ;
				
				 
				 //cout << "t2 " << angle << endl;
				 // cout << "r2 " << rate << " v " << (angle-prevAngle2)/simThread->cdt << endl;
				 prevAngle2 = angle;
				
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
						//TaskWeight.segment(taskRow+1,1).setConstant(90);
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
				aCom.head(3) = kpFootRot[i] * eOmega;
				
				if (footServos[i] == COM_SERVO) {
					aCom.head(3) += aDesFoot[i].head(3) + kdFootRot[i] * (vDesFoot[i].head(3) - vFoot[i].head(3));
					
					aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - (pFoot[i]-pCom));
					aCom.tail(3) += aDesFoot[i].tail(3) + kdFoot[i] * (vDesFoot[i].tail(3) - (vFoot[i].tail(3) - vCom));
				}
				else {
					
					aCom.head(3) += kdFootRot[i] * (vDesFoot[i].head(3) - vFoot[i].head(3));
					
					aCom.tail(3) = kpFoot[i] * (pDesFoot[i] - pFoot[i]);
					aCom.tail(3) += kdFoot[i] * (vDesFoot[i].tail(3) - vFoot[i].tail(3));
					aCom += aDesFoot[i];
				}

				
				
				if (kdFoot[i] == 0) {
					aCom = - 20 * vFoot[i];
					//aCom.setZero();
				}
				else {
					//cout << "Foot " << i << " e= " << eOmega.transpose() << endl;
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
	
	/*if(state == STANCE1 && simThread->sim_time > 6.04)
	{
		Vector3F fDes;
		fDes = hDotDes.tail(3);
		fDes(2)+=9.8*totalMass;
		
		cout << " fDes " << fDes.transpose() << endl;
		cout << " fOpt " << fs.segment(3,3).transpose() + fs.segment(9,3).transpose() << endl;
		cout << " hDes " << hDotDes.transpose() << endl;
		cout << " hOpt " << hDotOpt.transpose() << endl;
	}*/
	
	{
		
		/*static Vector3F pComPrev;
		
		Matrix3F i_R_pi_mat;
		
		cout << "m= " << totalMass << endl;
		cout << "VCom1 " << vCom.transpose() << endl;
		VectorXF p(26);
		p = artic->H * qd; 
	
		CartesianTensor i_R_pi; CartesianVector pLink;
		artic->m_link_list[0]->link->getPose(i_R_pi,pLink);
		copyRtoMat(i_R_pi, i_R_pi_mat);
		Vector3F v2 = i_R_pi_mat.transpose() * p.segment(3,3);
		cout << "VCom2 " << v2.transpose()/totalMass << endl;
		Float c = simThread->cdt;
		
		cout << "VCom3 " << (pCom- pComPrev).transpose()/c << endl;
		
		pComPrev = pCom;*/
	}
	
	
	if (pushRequest) {
		pushTime = simThread->sim_time;
		pushActive = true;
		pushRequest = false;
		SpatialVector push;
		push[0] =0; push[1]=0; push[2]=0; push[3]=0; push[4] = 1000; push[5] = 0;
		if (pushDirection) {
			push[4]*=-1;
		}
		
		((dmRigidBody *) (artic->m_link_list[0]->link))->setExternalForce(push);
		
		
		
	}
	if (pushActive && (simThread->sim_time-pushTime)>.01) {
		pushActive = false;
		SpatialVector push;
		push[0] =0; push[1]=0; push[2]=0; push[3]=0; push[4] = 0; push[5] = 0;
		((dmRigidBody *) (artic->m_link_list[0]->link))->setExternalForce(push);
	}
	
	
	if (frame->logDataCheckBox->IsChecked()) {
		logData();
	}
	/*if (stateTime == simThread->cdt) {
		
		cout << "State = " << stateNames[state] << endl;
		cout << "Weights = " << endl << TaskWeight << endl;
		cout << "Schedule = " << endl << OptimizationSchedule << endl;
	}*/
	
	
	stateTime += simThread->cdt;
}