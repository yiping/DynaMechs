/*
 *  humanoidControl.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/5/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"

#include "humanoidControl.h"
#include "dm.h"
#include "dmTime.h"
#include "TaskSpaceController.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
#include "CubicSplineTrajectory.h"
#include "HumanoidDataLogger.h"
#include "DataLogger.h"
//#define CONTROL_DEBUG



Matrix3F RSup;

TaskSpaceController * tsc;
Matrix6XF CentMomMat;
Vector3F pCom, vCom;
Vector6F cmBias,centMom, hDotDes, hDotOpt;
VectorXF q, qdDm, qd,tau, qdd, fs, lambda;
VectorXF pComDes(3), vComDes(3);

Float totalMass;

void ComputeGrfInfo(GRFInfo & grf);
void initializeDataLogging();

CubicSplineTrajectory ComTrajectory;

void initControl() {
	q.resize(NJ+7);
	qdDm.resize(NJ+7);
	qd.resize(NJ+6);
	
	tsc = new TaskSpaceController(G_robot);
	tsc->SupportIndices.resize(NS);
	
	tsc->SupportIndices[0] = 5;
	tsc->SupportIndices[1] = 9;
	
	RSup << 0,0,1,0,1,0,-1,0,0;
	
	XformVector Xforms(NS);
	for (int k=0; k<NS; k++) {
		Xforms[k].resize(6,6);
		
		dmRigidBody * link = (dmRigidBody*) G_robot->getLink(tsc->SupportIndices[k]);
		dmContactModel * dmContactLattice = (dmContactModel *) link->getForce(0); 
		
		Vector3F pRel;
		dmContactLattice->getContactPoint(0,pRel.data());
		pRel(1) = 0; pRel(2) = 0;
		Xforms[k].block(0,0,3,3) = RSup;
		Xforms[k].block(0,3,3,3) = Matrix3F::Zero();
		Xforms[k].block(3,0,3,3) = -RSup*cr3(pRel);
		Xforms[k].block(3,3,3,3) = RSup;
	}
	tsc->SupportXforms = Xforms;
	
		
	ComTrajectory.setSize(3);
	dataLogger->initializeDataLogging();
}

void HumanoidControl(ControlInfo & ci) {
	dmTimespec tv1, tv2, tv3, tv4;
	Float discountFactor = 1.;
	static bool splineFlag =true, splineFlag2 = true;
	int taskRow = 0;
	
	//Update State Variables for Control
	{
		dmGetSysTime(&tv1);
		G_robot->getState(q.data(),qdDm.data());
		
		// Test code for velocity initialization
		{
			//qdDm.segment(0,3) = Vector3F::Constant(2);
			//qdDm(3) = -1;
			//qdDm(4) = 2;
			//qdDm(5) = -.8;
			//qdDm.segment(7,NJ) = VectorXF::Ones(NJ);
			//G_robot->setState(q.data(),qdDm.data());
		}
		
		qd.segment(6,NJ) = qdDm.segment(7,NJ);
		//Dynamechs expresses the velocity of the FB in the ICS coordinate
		//We get it in the FB coord here to be consistent
		//with our definition of H.
		G_robot->getLink(0)->rtxFromInboard(qdDm.data(),qd.data());
		G_robot->getLink(0)->rtxFromInboard(qdDm.data()+3,qd.data()+3);
		
		tsc->ObtainArticulationData();
	}
	
	// Initialize Task Quantities
	{
		tsc->TaskJacobian.resize(6+NJ+12,NJ+6);
		tsc->TaskJacobian.setZero();
		tsc->TaskBias.resize(6+NJ+12);
	}
	
	// Compute Centroidal Task Information
	{
		ComputeComInfo(CentMomMat,cmBias,pCom,totalMass);
		ComputeGrfInfo(grfInfo);
		
		centMom = CentMomMat*qd;
		vCom = centMom.tail(3)/totalMass;
		
		Float kp, kd;
		VectorXF aComDes(3);
		
		
		
		if (simThread->sim_time<2) {
			kp = 15;
			kd = 25;
			pComDes << 2.09, 2-.07, .40;
			vComDes.setZero();
			aComDes.setZero();
		}
		else if (simThread->sim_time < 4) {
			double splineTime = .8;
			if (splineFlag) {
				
				VectorXF pComEnd(3);
				VectorXF vComEnd = VectorXF::Zero(3);
				VectorXF pComInit = pCom;
				VectorXF vComInit = centMom.tail(3)/totalMass;
				
				pComEnd  << 2.14, 2-.07, .50;
				
				ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,splineTime);
				splineFlag = false;
			}
			
			ComTrajectory.eval(simThread->sim_time-2, pComDes, vComDes, aComDes);
			kp = 90/5;
			//kd = 25*6*4;
			kd = 2*sqrt(kp)/5;
		}
		else {
			double splineTime = 1.2;
			if (splineFlag2) {
				
				VectorXF pComEnd(3);
				VectorXF vComEnd = VectorXF::Zero(3);
				VectorXF pComInit = pCom;
				VectorXF vComInit = centMom.tail(3)/totalMass;
				
				pComEnd  << 2.00, 2-.07, .50;
				
				ComTrajectory.init(pComInit, vComInit, pComEnd, vComEnd,splineTime);
				splineFlag2 = false;
			}
			ComTrajectory.eval(simThread->sim_time-4, pComDes, vComDes, aComDes);
			kp = 90/5;
			//kd = 25*6*4;
			kd = 2*sqrt(kp)/5;
			
		}

		
		
		//Our task jacobain will be a desired pose + centroidal quantities
		tsc->TaskJacobian.block(0,0,6,NJ+6) = CentMomMat;
		
		// Compute Task Bias from RNEA Fw Kin
		tsc->TaskBias.segment(0,6) = -cmBias;
		
		Vector3F linMomDotDes = totalMass*aComDes + totalMass*kp*(pComDes - pCom) + kd*(vComDes*totalMass - centMom.tail(3));
		Vector3F angMomDotDes = -kd*centMom.head(3);
		hDotDes.head(3) = angMomDotDes;
		hDotDes.tail(3) = linMomDotDes;
		
		//cout << "eCom " << (pComDes - pCom).transpose() << endl;
		//cout << "veCom " << (vComDes - centMom.tail(3) / totalMass) << endl;
		//cout << "adCom " << aComDes.transpose() << endl;
		//linMomDotDes << 0,0,10;
		//angMomDotDes << 0,0,0;
		
		// Dampen angular and PD CoM
		tsc->TaskBias.segment(0,3) += angMomDotDes;
		tsc->TaskBias.segment(3,3) += linMomDotDes;
		
		tsc->TaskJacobian.block(3,0,3,NJ+6)*=100;
		tsc->TaskBias.segment(3,3)*=100;
		
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
		tsc->TaskJacobian.block(taskRow,0,NJ,6) = MatrixXF::Zero(NJ,6);
		tsc->TaskJacobian.block(taskRow,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ)*discountFactor;
		
		// Compute task bias based on PD on joint angles
		VectorXF qDes = VectorXF::Zero(NJ); 					  
		qDes << 0,0,0,0,0,0,0,0,0,0,0,0, 0,2.5,-1.57, 0, 0, -2.4, -1.57, 0;
		Float Kp = 20, Kd = 12;
		tsc->TaskBias.segment(taskRow,NJ) = (Kp * (qDes - q.segment(7,NJ)) - Kd * qd.segment(6,NJ))*discountFactor;
		
		//cout << "q = " << q.segment(7,NJ) << endl;
		//exit(-1);
		
		// Handle Orientation Error for Shoulder Joints
		Matrix3F RDes,RAct;
		Vector3F eOmega;
		
		RDes.setIdentity();
		CartesianTensor RShoulder;
		CartesianVector pShoulder;
		
		const int linkNums[] = {10,12};
		const int jointNums[] = {12,16};
		for (int i=0; i<2; i++) {
			const int linkNum = linkNums[i];
			const int jointNum = jointNums[i];
			
			G_robot->getLink(linkNum)->getPose(RShoulder,pShoulder);
			copyRtoMat(RShoulder, RAct);
			matrixLogRot(RDes*RAct.transpose(),eOmega);
			eOmega *=-1;
			
			tsc->TaskBias.segment(taskRow+jointNum,3) = (Kp * eOmega - Kd * qd.segment(6+jointNum,3))*discountFactor;
			
			//cout << "Joint " << jointNum << endl;
			
			//cout << "RAct " << endl << RAct << endl;
			//cout << "e w " << endl<< eOmega << endl;
			
			//cout << "Bias seg " << tsc->TaskBias.segment(taskRow+jointNum,3) << endl;
		}
		
		
		
		
		//exit(-1);
		// Scale Rows Accordingly....
		
		// Legs
		tsc->TaskJacobian.block(taskRow,0,12,NJ+6) *= 0;
		tsc->TaskBias.segment(taskRow,12)        *= 0;
		
		// Right Arm
		tsc->TaskJacobian.block(taskRow+12,0,4,NJ+6) *= 10;
		tsc->TaskBias.segment(taskRow+12,4)		   *= 10;
		
		
		// Left Arm
		tsc->TaskJacobian.block(taskRow+16,0,4,NJ+6) *= 10;
		tsc->TaskBias.segment(taskRow+16,4)		   *= 10;
		
		taskRow += NJ;
	}
	
	//Compute Foot Task Information
	{
		vector<MatrixXF > footJacs(NS);
		vector<Vector6F > footBias(NS);
		
		static bool footSplineInit = false;
		CubicSplineTrajectory footSpline;
		footSpline.setSize(3);
		
		MatrixX6F X;
		X.resize(6,6);  X.block(0,3,3,3).setZero(); X.block(3,0,3,3).setZero();
		Matrix3F R;
		Matrix3F Rdes;
		Matrix3F eR;
		
		Rdes <<  0, 1,  0, 
				 0, 0, -1,
				-1, 0,  0;
		Vector3F eOmega, pDes, pAct;
		Vector6F aCom, vAct, vDes, aDes;
		vDes.setZero();
		aDes.setZero();
		//pDes << 2,2,1.25;
		pDes << 2,2,.01;
		
		
		
		for (int i=0; i<NS; i++) {
			int kp = 50*0, kd = 150*0;
			
			int jointIndex = tsc->SupportIndices[i];
			LinkInfoStruct * link = G_robot->m_link_list[jointIndex];
			copyRtoMat(link->link_val2.R_ICS, R);
			COPY_P_TO_VEC(link->link_val2.p_ICS, pAct);
			
			// Compute the orientation Error
			eR = Rdes * R.transpose();
			matrixLogRot(eR,eOmega);
			
			// Compute the link velocity in the ICS
			vAct.head(3) = R * link->link_val2.v.head(3);
			vAct.tail(3) = R * link->link_val2.v.tail(3);
			
			VectorXF pFootEnd(3),vDes3(3), aDes3(3);
			pFootEnd << 2.18,2,.1;
			
			if (simThread->sim_time > 7) {
				if (i==1) {
					if (!footSplineInit) {
						
						VectorXF vFootEnd(3);
						vFootEnd.setZero();
						
						footSpline.init(pAct, vAct.tail(3), pFootEnd, vFootEnd,1);
					}
					kp = 50;
					kd = 150;
					VectorXF pDes3(3);
					footSpline.eval(simThread->sim_time-7, pDes3, vDes3, aDes3);
					pDes = pDes3;
					vDes.tail(3) = vDes3;
					aDes.tail(3) = aDes3;
				}
			}
			if (simThread->sim_time>8 && i==1) {
				kp = 50;
				kd = 150;
				Float om = 10;
				Float ampz = .05, ampy = .05;
				Float som = sin(om*(simThread->sim_time-8));
				Float com = cos(om*(simThread->sim_time-8));
				
				pDes << 0 , ampy*som, -ampz*com+ampz;
				pDes+=pFootEnd;
				
				vDes3 << 0,ampy*com,ampz*som;
				vDes3 *=om;
				
				aDes3 << 0,-ampy*som,ampz*com;
				aDes3 *= om*om;
				
				vDes.tail(3) = vDes3;
				aDes.tail(3) = aDes3;
			}
			
			
			// Use a PD to create a desired acceleration
			aCom.head(3) = kp * eOmega;
			aCom.tail(3) = kp * (pDes - pAct);
			aCom += aDes + kd * (vDes - vAct);
			
			// Compute Task Information
			X.block(0,0,3,3) = R; 
			X.block(3,3,3,3) = R;
			G_robot->computeJacobian(jointIndex,X,footJacs[i]);
			computeAccBiasFromFwKin(link->link_val2,footBias[i]);
			
			// Load Task Information
			tsc->TaskJacobian.block(taskRow,0,6,NJ+6) = footJacs[i];
			tsc->TaskBias.segment(taskRow,6)          = aCom - footBias[i];
			
			// Option to Scale Linear Position Control
			tsc->TaskJacobian.block(taskRow+3,0,3,NJ+6) *=100; 
			tsc->TaskBias.segment(taskRow+3,3) *= 100;
			
			
			// Option to Scale whole task
			tsc->TaskJacobian.block(taskRow,0,6,NJ+6)*=10;
			tsc->TaskBias.segment(taskRow,6)*=10;
			
			
			//cout << "Task Jac " << jointIndex << endl << footJacs[i] << endl;
			//cout << "bias = " << footBias[i].transpose() << endl;
			//cout << "aCom =  " << aCom.transpose() << endl;
			//cout << "eOm " << eOmega.transpose() << endl;
			//cout << "eR " << eR << endl;
			taskRow+=6;
			pDes(0)+=.18;
		}
	}
	
	//Update Graphics Variables
	{
		ComPos[0] = pCom(0);
		ComPos[1] = pCom(1);
		ComPos[2] = pCom(2);
		
		ComDes[0] = pComDes(0);
		ComDes[1] = pComDes(1);
		ComDes[2] = pComDes(2);
		
	}
	
	
	// Perform Optimization
	{
		dmGetSysTime(&tv2);
		tsc->InitializeProblem();
		if (simThread->sim_time > 6.) {
			if (simThread->sim_time < 7) {
				static bool footInit =false;
				static double maxLoad;
				if(!footInit)
				{
					maxLoad = grfInfo.fCoPs[1](2);
					footInit = true;
				}
				//maxLoad = 0;
				tsc->AssignFootMaxLoad(1,maxLoad*(7-simThread->sim_time));
			}
			else {
				tsc->AssignFootMaxLoad(1,0);
			}
			
		}
		dmGetSysTime(&tv3);
		tsc->Optimize();
		tau = tsc->xx.segment(tauStart,NJ);
		qdd = tsc->xx.segment(qddStart,NJ+6);
		fs = tsc->xx.segment(fStart,6*NS);
		lambda = tsc->xx.segment(lambdaStart,NS*NP*NF);
		
		// Extract Results
		hDotOpt = CentMomMat*qdd + cmBias;
		
		// Form Joint Input and simulate
		VectorXF jointInput = VectorXF::Zero(NJ+7);
		jointInput.segment(7,NJ) = tau;
		G_robot->setJointInput(jointInput.data());
		dmGetSysTime(&tv4);
	}
	
	//Populate Control Info Struct
	{
		ci.calcTime = timeDiff(tv1,tv2);
		ci.setupTime = timeDiff(tv2,tv3);
		ci.optimTime = timeDiff(tv3,tv4);
		ci.totalTime = timeDiff(tv1,tv4);
		ci.iter      = tsc->iter;
	}
	
	
	#ifdef CONTROL_DEBUG
	// Debug Code
	{
		if (sim_time > 6) {
			
		cout << setprecision(5);
			
			MSKboundkeye key;
			double bl,bu;
			
			MSK_getbound(tsc->task, MSK_ACC_VAR, 57, &key, &bl, &bu);
			cout << "Lower " << bl << endl;
			cout << "Upper " << bu << endl;
			switch (key) {
				case MSK_BK_FR:
					cout << " Free " << endl;
					break;
				case MSK_BK_LO:
					cout << " Lower Bound " << endl;
					break;
				case MSK_BK_UP:
					cout << "Upper Bound " << endl;
					break;
				case MSK_BK_FX:
					cout << "Fixed" << endl;
					break;
				default:
					break;
			}
		cout << "x(57) = " << tsc->xx(57) << endl;
		cout << "tau = " << tau.transpose() << endl;
		cout << "qdd = " << qdd.transpose() << endl;
		cout << "fs = "  << fs.transpose()  << endl;
		//cout << "lambda = "  << lambda.transpose()  << endl;
		
		
		VectorXF a = tsc->TaskJacobian * qdd;
		//cout << "a" << endl;
		
		VectorXF e = tsc->TaskJacobian * qdd - tsc->TaskBias;
		cout << "e = " << e.transpose() << endl;
		
		MatrixXF H = G_robot->H;
		VectorXF CandG = G_robot->CandG;
		
		MatrixXF S = MatrixXF::Zero(NJ,NJ+6);
		S.block(0,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ);
		
		VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
		for (int i=0; i<NS; i++) {
			generalizedContactForce += tsc->SupportJacobians[i].transpose()*fs.segment(6*i,6);
		}
		
		VectorXF qdd2 = H.inverse()*(S.transpose() * tau + generalizedContactForce- CandG);
		//cout << "qdd2 = " << qdd2.transpose() << endl << endl << endl;
		
		//cout << "CandG " << CandG.transpose() << endl; 
		
		//cout << "Gen Contact Force " << generalizedContactForce.transpose() << endl;
		
		cout << "hdot " << (CentMomMat*qdd + cmBias).transpose() << endl;
		
		cout << "cmBias " << cmBias.transpose() << endl;
		
		cout << "qd " << qd.transpose() << endl;
		//VectorXF qdd3 = H.inverse()*(S.transpose() * tau - CandG);
		//FullPivHouseholderQR<MatrixXF> fact(H);
		
		cout <<"fNet    \t" << (fs.segment(3,3) + fs.segment(9,3)).transpose() << endl;
		Vector3F g;
		g << 0,0,-9.81;
		cout <<"hdot - mg\t" << (CentMomMat*qdd + cmBias).segment(3,3).transpose() -  totalMass * g.transpose()<< endl;
		//exit(-1);
		
		
		
		
		// Old Debug Code
		{
			VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
			
			Matrix6F X;
			MatrixXF Jac;
			X.setIdentity();
			
			for (int i=0; i<NS; i++) {
				int linkIndex = tsc->SupportIndices[i];
				G_robot->computeJacobian(linkIndex,X,Jac);
				dmRigidBody * link = (dmRigidBody *) G_robot->getLink(linkIndex);
				
				for (int j=0; j< link->getNumForces(); j++) {
					dmForce * f = link->getForce(j);
					Vector6F fContact;
					f->computeForce(G_robot->m_link_list[linkIndex]->link_val2,fContact.data());
					generalizedContactForce += Jac.transpose()*fContact;
				}
			}
			
			cout << "J' f = " << generalizedContactForce.transpose() << endl;
			
			VectorXF qdd3   = H.inverse()*(S.transpose() * tau - CandG + generalizedContactForce);
			cout << "qdd3 = " << qdd3.transpose() << endl;
			
			VectorXF state = VectorXF::Zero(2*(NJ+7));
			state.segment(0,NJ+7) = q;
			state.segment(NJ+7,NJ+7) = qdDm;
			
			VectorXF stateDot = VectorXF::Zero(2*(NJ+7));
			
			
			//Process qdds
			G_robot->dynamics(state.data(),stateDot.data());
			//
			VectorXF qdds = VectorXF::Zero(NJ+6);
			qdds.segment(0,6) = stateDot.segment(NJ+7,6);
			
			//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl;
			
			qdds.segment(3,3) -= cr3(qdDm.segment(0,3))*qdDm.segment(3,3);
			qdds.segment(6,NJ) = stateDot.segment(NJ+7*2,NJ);
			
			Matrix3F ics_R_fb;
			copyRtoMat(G_robot->m_link_list[0]->link_val2.R_ICS,ics_R_fb);
			
			qdds.head(3) = ics_R_fb.transpose() * qdds.head(3);
			qdds.segment(3,3) = ics_R_fb.transpose() * qdds.segment(3,3);
			
			
			cout << "qdds = " << qdds.transpose()  << endl;
			
			//cout << "CandG " << endl << CandG << endl;
			
			//cout << setprecision(6);
			//cout << "I_0^C = " << endl << G_robot->H.block(0,0,6,6) << endl;
			//exit(-1);
		}
		}
	}
	#endif
	
	if (!frame->logDataCheckBox->IsChecked()) {
		dataLogger->logData();
	}
	
	
	//exit(-1);
}

void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m) {
	
	LinkInfoStruct * fb = G_robot->m_link_list[0];
	dmLink * fbLink = fb->link;
	
	CartesianTensor fb_R_ics;
	
	//pCom initally contains the position of the torso in the i.c.s
	fbLink->getPose(fb_R_ics,pCom.data());
	
	
	Matrix3F ics_R_fb;
	ics_R_fb << fb_R_ics[0][0] , fb_R_ics[1][0] , fb_R_ics[2][0],
		        fb_R_ics[0][1] , fb_R_ics[1][1] , fb_R_ics[2][1],
		        fb_R_ics[0][2] , fb_R_ics[1][2] , fb_R_ics[2][2];
	
	Vector3F pFBRelCoM;
	m = fb->I_C.m;
	pFBRelCoM(0) = -fb->I_C.h[0]/m;
	pFBRelCoM(1) = -fb->I_C.h[1]/m;
	pFBRelCoM(2) = -fb->I_C.h[2]/m;
	
	//Matrix6F IC0;
	//CrbToMat(fb->I_C, IC0);
	//cout << "FB IC0 in CoM Func " << endl << IC0 << endl;
	
	// pCom = pBase + pComRelBase
	pCom -= ics_R_fb*pFBRelCoM;
	
	int N = G_robot->H.cols();
	Cmm.resize(6,N);
	
	
	Matrix6F XT = Matrix6F::Zero();
	XT.block(0,0,3,3) = ics_R_fb;
	XT.block(0,3,3,3) = ics_R_fb*cr3(pFBRelCoM);
	XT.block(3,3,3,3) = ics_R_fb;
	
	
	Cmm = XT * G_robot->H.topRows(6);
	
	
	bias = XT * (fb->link_val2.f-G_robot->H.block(0,0,6,6)*fb->link_val2.a);
}

void ComputeGrfInfo(GRFInfo & grf) {
	
	Vector6F fZMP = Vector6F::Zero();
	Vector6F icsForce;
	SpatialVector localForce;
	Float * nICS = icsForce.data();
	Float * fICS =nICS+3;
	Float * nLoc =localForce;
	Float * fLoc =nLoc + 3;
	CartesianVector tmp;
	
	grf.localContacts = NS;
	grf.pCoPs.resize(NS);
	grf.fCoPs.resize(NS);
	grf.nCoPs.resize(NS);

	
	for (int k1 = 0; k1 < NS; k1++) {
		dmRigidBody * body = (dmRigidBody *) G_robot->getLink(tsc->SupportIndices[k1]);
		LinkInfoStruct * listruct = G_robot->m_link_list[tsc->SupportIndices[k1]];
		
		for (int k2 = 0; k2 < body->getNumForces(); k2++) {
			body->getForce(k2)->computeForce(listruct->link_val2,localForce);
			
			//Float * p = localForce;
			//cout << "Local Force " << tsc->SupportIndices[k1];
			//cout << " = " << localForce[0] << " , " << localForce[1] << " , " << localForce[2] << " , ";
			//cout << localForce[3] << " , " << localForce[4] << " , " << localForce[5] << endl;
			
			// Apply Spatial Force Transform Efficiently
			// Rotate Quantities
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,nLoc,nICS);
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,fLoc,fICS);
			
			// Add the r cross f
			CROSS_PRODUCT(listruct->link_val2.p_ICS,fICS,tmp);
			ADD_CARTESIAN_VECTOR(nICS,tmp);
			
			//cout << "ICS Force " << icsForce.transpose() << endl;
			
			fZMP+=icsForce;
			
			transformToZMP(icsForce,grf.pCoPs[k1]);
			grf.fCoPs[k1] = icsForce.tail(3);
			grf.nCoPs[k1] = icsForce(2);
		}
	}
	transformToZMP(fZMP,grf.pZMP);
	grf.fZMP = fZMP.tail(3);
	grf.nZMP = fZMP(2);
}