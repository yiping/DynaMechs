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
: HumanoidDataLogger(robot, NUM_JUMP_STATES), ComTrajectory(3)
{
	stateFunctions.resize(numStates);
	
	stateNames[FLOATING] = "Floating";
	stateFunctions[FLOATING] = &RunningStateMachine::Floating;
	
	stateNames[DROP] = "Drop";
	stateFunctions[DROP] = &RunningStateMachine::Drop;
	
	stateNames[BALANCE_MIDDLE] = "Balance Middle";
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
	stateFunctions[RAISE_FOOT] = &RunningStateMachine::RaiseFoot;
	
	
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
	kpCM = 30*0;
	kdCM = 2*sqrt(kpCM)*0;
	kdAM = 25*0;
	pComDes << 2.00, 2.0, .48*1.7;
	//pComDes << 2.00, 2.0, .72;
	vComDes.setZero();
	aComDes.setZero();
	kDotDes.setZero();
	kComDes.setZero();
	
	
	
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
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}
	transitionFlag = false;
}
void RunningStateMachine::BalanceMiddle() {
	if (transitionFlag) {
		cout << pCom << endl;
	}
	if (stateTime > 1) {
		state = BALANCE_LEFT;
		transitionFlag = true;
		return;
	}
	kpCM = 30;
	kdCM = 2*sqrt(kpCM);
	kdAM = 25;
	pComDes << 2.00, 2.0, .48*1.7;
	//pComDes << 2.00, 2.0, .72;
	vComDes.setZero();
	aComDes.setZero();
	kDotDes.setZero();
	kComDes.setZero();
	transitionFlag = false;
}

void RunningStateMachine::Squat()
{
	const Float SquatTime = .5;
	
	if(transitionFlag)
	{
		cout << "Squat" << endl;
		VectorXF pComInit = pCom;
		VectorXF vComInit = vCom;
		
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		
		pComEnd << 2.02, 2.0, .32*1.7;
		
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

void RunningStateMachine::Thrust()
{
	if (transitionFlag) {
		cout << "Thrust" << endl;
	}
	//cout << pCom(2) << endl;
	//cout << "Thrust" << endl;
	if (q(11) < .85) {
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

void RunningStateMachine::Flight()
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
		
		footSpline.init(pInit, vInit, pFinal, vZero, .3);
		//transitionSpline.init(0, 0, 1, 0, .05);
	}
	OptimizationSchedule.head(6).setConstant(-1);
	OptimizationSchedule.tail(12).setConstant(2);
	
	//taskOptimActive.head(6).setZero();
	//taskConstrActive.tail(12).setZero();
	//taskOptimActive.tail(12).setOnes();
	
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

void RunningStateMachine::Land()
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

void RunningStateMachine::RaiseFoot()
{
	static CubicSplineTrajectory footSpline(3);
	static VectorXF vDes3(3), aDes3(3), pAct(3), vAct(6);
	static bool raisedFlag = true;
	static Vector3F pCenter,pRel;
	
	Float raiseSplineTime = .3;
	if (transitionFlag) {
		VectorXF pFootEnd(3);
		
		VectorXF vFootEnd(3);
		vFootEnd.setZero();
		
		pFootEnd << pFoot[0](0),pFoot[0](1),pFoot[0](2)+.02*1.7; 
		footSpline.init(pFoot[0], vFoot[0].tail(3), pFootEnd, vFootEnd,raiseSplineTime);
		
		transitionFlag = false;
	}
	if (stateTime > raiseSplineTime) {
		state = KICK;
		transitionFlag = true;
	}
	
	
	OptimizationSchedule.segment(0,3).setConstant(4);
	AssignFootMaxLoad(0,0);
	
	kpFoot[0] = 50;
	kdFoot[0] = 150;
	VectorXF pDes3(3);
	footSpline.eval(stateTime, pDes3, vDes3, aDes3);
	
	vDesFoot[0].setZero();
	aDesFoot[0].setZero();
	
	pDesFoot[0] = pDes3;
	vDesFoot[0].tail(3) = vDes3;
	aDesFoot[0].tail(3) = aDes3;
	
	
}

void RunningStateMachine::Kick()
{
	//static BezierCurve rthetaCurve;
	//static MatrixXF A(2,9);
	
	static CubicSplineTrajectory rthetaSpline(2);
	
	static bool raisedFlag = true;
	
	static Vector3F pCenter,pRel;
	
	static Float r0, theta0;
	
	static Matrix3F RFoot0;
	static Vector3F pFootEnd;
	
	static int subState = 0;
	
	Matrix3F IBarC0_init = IBarC0;
	
	Float kickTime = 1.1*1.5;
	Float swingBackTime =.5*1.5;
	Float swingForwardTime = .25*1.5;
	Float slowTime = .5*1.5;
	Float returnTime = .6*1.5;
	static Float splineTime = 0;
	kpFoot[0] = 50*5;
	kdFoot[0] = 2*sqrt(kpFoot[0]);
	
	if (transitionFlag) {
		pFootEnd = pFoot[0];
		//cout << "pFoot init = " << pFoot[0].transpose() << endl;
		
		pCenter(0) = artic->m_link_list[2]->link_val2.p_ICS[0];
		pCenter(1) = artic->m_link_list[2]->link_val2.p_ICS[1];
		pCenter(2) = artic->m_link_list[2]->link_val2.p_ICS[2];
		
		pRel = pFoot[0] - pCenter;
		//cout << "Rel Pos " << pRel.transpose() << endl;
		
		r0 = sqrt(pRel(0)*pRel(0) + pRel(2)*pRel(2));
		theta0 = atan2(pRel(0), -pRel(2));
		//cout << "r =  " << r0 << " theta = " << theta0 << endl;
		
		RFoot0 = RFoot[0];
		
		VectorXF g0(2),gd0(2),gf(2),gdf(2);
		g0 << r0,theta0;
		gd0 << 0,0;
		gf  << r0,-.93;
		gdf << 0,0;
		
		rthetaSpline.init(g0, gd0, gf, gdf, swingBackTime);
		//WayPointVector Ps;
		//Ps.resize(9);
		//A.row(1) <<	0.000000,0.000000,1.295157,-6.481478,1.844407,4.495988,-1.512786,0.300000,0.300000;
		//A.row(0) << r0,r0,r0,r0,r0,r0,r0,r0,r0;
		//for (int i=0; i<9; i++) {
		//	Ps[i].resize(2);
		//	Ps[i] = A.col(i);
		//}
		//rthetaCurve.init(Ps, kickTime);
		transitionFlag = false;
	}
	if (subState == 0 && splineTime >swingBackTime) {
		VectorXF g0(2),gd0(2),gf(2),gdf(2);
		
		gf  << r0,0;
		gdf << 0,7;
		
		rthetaSpline.reInit(swingBackTime,gf, gdf, swingForwardTime);
		subState = 1;
		splineTime = 0;
	}
	if (subState == 1 && splineTime >swingForwardTime) {
		
		VectorXF g0(2),gd0(2),gf(2),gdf(2);
		
		gf  << r0+.055,.95;
		gdf << 0,0;
		
		rthetaSpline.reInit(swingForwardTime,gf, gdf, slowTime);
		subState = 2;
		splineTime=0;
	}
	if (subState == 2 && splineTime >slowTime*.8) {
		VectorXF g0(2),gd0(2),gf(2),gdf(2);
		
		gf  << r0,0;
		gdf << 0,0;
		
		rthetaSpline.reInit(slowTime*.8,gf, gdf, returnTime);
		subState = 3;
		splineTime=0;
	}
	
	
	
	
	
	AssignFootMaxLoad(0,0);
	
	
	VectorXF gDes(2),gdDes(2),gddDes(2);
	
	//rthetaCurve.eval(stateTime, gDes, gdDes, gddDes);
	rthetaSpline.eval(splineTime, gDes, gdDes, gddDes);
	
	Float r = gDes(0);
	Float rd = gdDes(0);
	Float rdd = gddDes(0);
	
	Float t = theta0+gDes(1);
	Float td = gdDes(1);
	Float tdd = gddDes(1);
	
	Float ct = cos(t);
	Float st = sin(t);
	
	vDesFoot[0].setZero();
	aDesFoot[0].setZero();
	
	Vector3F pDes,vDes,aDes;
	
	
	pDes = pCenter;
	pDes(0) += r*st;
	pDes(1) = pFootEnd(1);
	pDes(2) -= r*ct;
	
	vDes.setZero(); aDes.setZero();
	
	vDes(0) += rd*st + r*ct*td;
	aDes(0) += rdd*st + 2 *rd*ct*td - r*st*pow(td,2) + r*ct*tdd;
	
	vDes(2) -= rd*ct - r*st*td;
	aDes(2) -= rdd*ct -2*rd*st*td-r*ct*pow(td,2) - r*st*tdd;
	
	
	Float ballState[7],ballStateDot[7];
	ball->getState(ballState, ballStateDot);
	Float ballRad = .07;
	
	Vector3F ballPos;
	ballPos << ballState[4], ballState[5],ballState[6] ;
	
	Matrix3F ball_R_ics;
	CartesianTensor ball_R_icsM;
	CartesianVector pBall;
	ball->getLink(0)->getPose(ball_R_icsM,pBall);
	copyRtoMat(ball_R_icsM, ball_R_ics);
	
	Vector6F force;
	force.setZero();
	
	
	Vector3F contactPoint;
	
	Vector6F sumForce;
	sumForce.setZero();
	
	Vector6F footForce;
	footForce.setZero();
	
	for (Float contacty = -.046 ; contacty <=.046; contacty+=.046/4)
	{
		contactPoint << 0,contacty,0.119;
		contactPoint = RFoot[0] * contactPoint;
		contactPoint += pFoot[0];
		
		Vector3F relPos = ballPos - contactPoint;
		
		
		if (relPos.norm() < ballRad) {
			Float penetration = ballRad - relPos.norm();
			relPos.normalize();
			
			//cout << "Collision Force " << relPos*penetration * 2500/4. <<  endl;
			
			force.tail(3)=relPos*penetration * 2500/4;
			sumForce+=force;
			
		}
	}
	if (sumForce.norm() > 0) {
		//sumForce(1)=0;
		//sumForce(2) = 0;
		
		footForce.head(3) = cr3(sumForce.tail(3)) * (ballPos-pFoot[0]);
		footForce.tail(3) = -sumForce.tail(3);
		footForce.head(3) = RFoot[0].transpose()*footForce.head(3);
		footForce.tail(3) = RFoot[0].transpose()*footForce.tail(3);
		
		
		//cout << "Force = " << sumForce.transpose() << endl;
		sumForce.tail(3) = ball_R_ics * sumForce.tail(3);
	}
	
	
	((dmRigidBody *) artic->getLink(SupportIndices[0]))->setExternalForce(footForce.data());
	((dmRigidBody *) ball->getLink(0))->setExternalForce(sumForce.data());	
	
	
	kpCM = 10;
	kdCM = 2*sqrt(kpCM);
	kdAM = 15;
	
	kComDes.setZero();
	kDotDes.setZero();
	
	if (subState <= 2) {
		OptimizationSchedule.segment(0,3).setConstant(4);
		TaskWeight.segment(0,3).setConstant(200/10.);
		//TaskWeight.segment(0,3).setConstant(50);
		kComDes(1) = -IBarC0_init(1,1) * td*.8;
		kDotDes(1) = -IBarC0_init(1,1) * tdd*.8;		
	}
	else {
		OptimizationSchedule.segment(0,3).setConstant(4);
	}
	
	//kComDes.setZero();
	//kDotDes.setZero();
	
	Vector3F angAx;
	angAx << 0,-t,0;
	matrixExpOmegaCross(angAx,RDesFoot[0]);
	
	RDesFoot[0] = RDesFoot[0]*RFoot0;
	pDesFoot[0] = pDes;
	vDesFoot[0](1) = -td;
	vDesFoot[0].tail(3) = vDes;
	aDesFoot[0](1) = -tdd;
	aDesFoot[0].tail(3) = aDes;
	//cout << "t = " << stateTime << " (r,t) = " << r << ", " << t << " pDes = " << pDes.transpose() << endl;
	splineTime+=simThread->cdt;
}
void RunningStateMachine::BalanceLeft()
{
	static Float MaxFootLoad;
	if (transitionFlag) {
		VectorXF pComEnd(3);
		VectorXF vComEnd = VectorXF::Zero(3);
		VectorXF pComInit = pCom;
		VectorXF vComInit = centMom.tail(3)/totalMass;
		
		pComEnd  << pCom(0)+.06, pCom(1)+.06*1.7, pCom(2)+.08*1.7;
		
		ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,.8*1.5);
		transitionFlag = false;
		
		MaxFootLoad = grfInfo.fCoPs[0](2);
	}
	if (stateTime > .81*1.5) {
		state = RAISE_FOOT;
		transitionFlag =true;
		return;
	}
	
	OptimizationSchedule.segment(0,3).setConstant(4);
	Float footLoad = MaxFootLoad * (.8-stateTime)/.8;
	if (footLoad<0) {
		footLoad = 0;
	}
	
	AssignFootMaxLoad(0,footLoad);
	
	
	ComTrajectory.eval(stateTime, pComDes, vComDes, aComDes);
}



void RunningStateMachine::StateControl(ControlInfo & ci)
{
	dmTimespec controlStart, controlEnd;
	dmGetSysTime(&controlStart);
	
	this->HumanoidController::ControlInit();
	
	OptimizationSchedule.setZero(6+NJ+6+12);
	OptimizationSchedule.segment(0,6).setConstant(3);
	
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
				
				kpHip = 120/3.;
				wHip  = 1;
				
				kpShould = 220;
				wShould  = 10.0;
				
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
					/////////////////////////////////////////////////////////
				}
				
				
				
			}
			
			
			
			
			
			//Float Kp = 120, Kd = 2*sqrt(Kp);
			
			Vector3F relPos;
			relPos = (pCom - pFoot[1]);
			cout << relPos.norm() << endl;
			
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
						TaskWeight.segment(taskRow,3).setConstant(100);
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
		
		
		//cout << "Control Complete " << simThread->sim_time << endl;
		//exit(-1);
		
	}
	else {
		tau.setZero(26);
		qdd.setZero(26);
		qddA.setZero(26);
		fs.setZero(12);
	}
	
	
	if (frame->logDataCheckBox->IsChecked()) {
		logData();
	}
	
	
	stateTime += simThread->cdt;
}