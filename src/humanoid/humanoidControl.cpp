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
VectorXF q, qdDm, qd;
TaskSpaceController * tsc;
Matrix3F RSup;
#include "DataLogger.h"

void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m);
void matrixExpOmegaCross(const Vector3F & omega,Matrix3F & R);
void matrixLogRot(const Matrix3F & R, Vector3F & omega);
void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector6F & a);

void copyRtoMat(const CartesianTensor R, Matrix3F & Rmat);
double timeDiff(const dmTimespec & t1, const dmTimespec & t2);
void transformToZMP(Vector6F & fZMP, Vector3F & pZMP) ;
void ComputeGrfInfo(GRFInfo & grf);
void initializeDataLogging();

CubicSplineTrajectory ComTrajectory;
DataLogger dataLog;

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
	initializeDataLogging();
}

void HumanoidControl(ControlInfo & ci) {
	dmTimespec tv1, tv2, tv3, tv4;
	Matrix6XF CentMomMat;
	Vector3F pCom;
	Vector6F cmBias,centMom;
	Float totalMass;
	Float discountFactor = 1.;
	VectorXF pComDes(3);
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
		Float kp, kd;
		VectorXF vComDes(3);
		VectorXF aComDes(3);
		
		
		
		if (sim_time<2) {
			kp = 15;
			kd = 25;
			pComDes << 2.09, 2-.07, .40;
			vComDes.setZero();
			aComDes.setZero();
		}
		else if (sim_time < 4) {
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
			
			ComTrajectory.eval(sim_time-2, pComDes, vComDes, aComDes);
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
			ComTrajectory.eval(sim_time-4, pComDes, vComDes, aComDes);
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
		
		
		MatrixX6F X;
		X.resize(6,6);  X.block(0,3,3,3).setZero(); X.block(3,0,3,3).setZero();
		Matrix3F R;
		Matrix3F Rdes;
		Matrix3F eR;
		
		Rdes <<  0, 1,  0, 
				 0, 0, -1,
				-1, 0,  0;
		Vector3F eOmega, pDes, pAct;
		Vector6F aCom, vAct;
		
		//pDes << 2,2,1.25;
		pDes << 2,2,.01;
		
		int kp = 50*0, kd = 150*0;
		
		for (int i=0; i<NS; i++) {
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
			
			// Use a PD to create a desired acceleration
			aCom.head(3) = kp * eOmega;
			aCom.tail(3) = kp * (pDes - pAct);
			aCom -= kd * vAct;
			
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
		dmGetSysTime(&tv3);
		tsc->Optimize();
		VectorXF tau = tsc->xx.segment(tauStart,NJ);
		VectorXF qdd = tsc->xx.segment(qddStart,NJ+6);
		VectorXF fs = tsc->xx.segment(fStart,6*NS);
		VectorXF lambda = tsc->xx.segment(lambdaStart,NS*NP*NF);
		
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
		cout << setprecision(5);
		
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
	#endif
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

void matrixExpOmegaCross(const Vector3F & omega,Matrix3F & R) {
	Float theta = omega.norm();
	Matrix3F omegaHat = cr3(omega/theta);
	
	//R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
	R.setIdentity();
	R+=omegaHat*(sin(theta)*Matrix3F::Identity() + omegaHat*(1-cos(theta)));
}

void matrixLogRot(const Matrix3F & R, Vector3F & omega) {
	// theta = acos( (Trace(R) - 1)/2 )
	Float theta;
	Float tmp = (R(0,0) + R(1,1) + R(2,2) - 1) / 2;
	if (tmp >=1.) {
		theta = 0;
	}
	else if (tmp <=-1.) {
		theta = M_PI;
	}
	else {
		theta = acos(tmp);
	}
	
	//Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
	//crossExtract(omegaHat,omega);
	omega << R(2,1)-R(1,2) , R(0,2)-R(2,0),R(1,0)-R(0,1);
	if (theta > 10e-5) {
		omega*=theta / (2*sin(theta));
	}
	else {
		omega/=2;
	}
}


void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector6F & a) {
	Vector6F tmp;
	tmp = infoStruct.a + infoStruct.ag;
	ClassicAcceleration(tmp,infoStruct.v, a);
	a.swap(tmp);
	Float * pa = a.data();
	Float * ptmp = tmp.data();
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa);
	ptmp+=3;
	pa+=3;
	APPLY_CARTESIAN_TENSOR(infoStruct.R_ICS,ptmp,pa);
}

void copyRtoMat(const CartesianTensor R, Matrix3F & Rmat) {
	Rmat << R[0][0] , R[0][1], R[0][2] , 
			R[1][0] , R[1][1], R[1][2] ,
			R[2][0] , R[2][1], R[2][2] ;
}


double timeDiff(const dmTimespec & t1, const dmTimespec & t2) {
	return ((double) t2.tv_sec - t1.tv_sec) + (1.0e-9*((double) t2.tv_nsec - t1.tv_nsec));
}

void transformToZMP(Vector6F & fZMP, Vector3F & pZMP) {
	 const Float nx = fZMP(0), ny = fZMP(1), fz = fZMP(5);
	 
	 pZMP(0) = - ny/fz;
	 pZMP(1) =   nx/fz;
	 pZMP(2) = 0;
	 
	 fZMP(0) = 0;
	 fZMP(1) = 0;
	 fZMP(2)+= fZMP(4)*pZMP(0) - fZMP(3)*pZMP(1);
}

void initializeDataLogging() {
	
	// Angles
	dataLog.setItemName(BASE_QUAT0,		"Base Quaternion0");
	dataLog.setItemName(BASE_QUAT1,		"Base Quaternion1");
	dataLog.setItemName(BASE_QUAT2,		"Base Quaternion2");
	dataLog.setItemName(BASE_QUAT3,		"Base Quaternion3");
	dataLog.setItemName(BASE_P_X,		"Base Position X");
	dataLog.setItemName(BASE_P_Y,		"Base Position Y");
	dataLog.setItemName(BASE_P_Z,		"Base Position Z");
	dataLog.setItemName(RHIP_PHI,		"R Hip Phi");
	dataLog.setItemName(RHIP_PSI,		"R Hip Psi");
	dataLog.setItemName(RHIP_GAMMA,		"R Hip Gamma");
	dataLog.setItemName(RKNEE,			"R Knee Angle");
	dataLog.setItemName(RANK1,			"R Ank 1 Angle");
	dataLog.setItemName(RANK2,			"R Ank 2 Angle");
	dataLog.setItemName(LHIP_PHI,		"L Hip Phi");
	dataLog.setItemName(LHIP_PSI,		"L Hip Psi");
	dataLog.setItemName(LHIP_GAMMA,		"L Hip Gamma");
	dataLog.setItemName(LKNEE,			"L Knee Angle");
	dataLog.setItemName(LANK1,			"L Ank 1 Angle");
	dataLog.setItemName(LANK2,			"L Ank 2 Angle");
	dataLog.setItemName(RSHOULD_PHI,	"R Shoulder Phi");
	dataLog.setItemName(RSHOULD_PSI,	"R Shoulder Psi");
	dataLog.setItemName(RSHOULD_GAMMA,	"R Shoulder Gamma");
	dataLog.setItemName(RELBOW,			"R Elbow Angle");
	dataLog.setItemName(LSHOULD_PHI,	"L Shoulder Phi");
	dataLog.setItemName(LSHOULD_PSI,	"L Shoulder Psi");
	dataLog.setItemName(LSHOULD_GAMMA,	"L Shoulder Gamma");
	dataLog.setItemName(LELBOW,			"L Elbow Angle");
	
	int angleItems[] = {BASE_QUAT0,BASE_QUAT1,BASE_QUAT2,BASE_QUAT3,
					BASE_P_X,BASE_P_Y,BASE_P_Z,RHIP_PHI,RHIP_PSI, RHIP_GAMMA,
					RKNEE,RANK1,RANK2,LHIP_PHI,LHIP_PSI,LHIP_GAMMA,LKNEE,LANK1,LANK2,
					RSHOULD_PHI,RSHOULD_PSI,RSHOULD_GAMMA,RELBOW,LSHOULD_PHI,
					LSHOULD_PSI,LSHOULD_GAMMA,LELBOW};
	IntVector angleGroup(angleItems,angleItems+sizeof(angleItems)/sizeof(int));
	dataLog.declareGroup(JOINT_ANGLES,angleGroup);
	
	dataLog.setItemName(BASE_OMEGA_X,	"Base Omega X");
	dataLog.setItemName(BASE_OMEGA_Y,	"Base Omega Y");
	dataLog.setItemName(BASE_OMEGA_Z,	"Base Omega Z");
	dataLog.setItemName(BASE_V_X,		"Base Vel. X");
	dataLog.setItemName(BASE_V_Y,		"Base Vel. Y");
	dataLog.setItemName(BASE_V_Z,		"Base Vel. Z");
	dataLog.setItemName(RHIP_OMEGA_X,	"R Hip Omega X");
	dataLog.setItemName(RHIP_OMEGA_Y,   "R Hip Omega Y");
	dataLog.setItemName(RHIP_OMEGA_Z,	"R Hip Omega Z");
	dataLog.setItemName(RKNEE_RATE,		"R Knee Rate");
	dataLog.setItemName(RANK1_RATE,		"R Ank 1 Rate");
	dataLog.setItemName(RANK2_RATE,		"R Ank 2 Rate");
	dataLog.setItemName(LHIP_OMEGA_X,	"L Hip Omega X");
	dataLog.setItemName(LHIP_OMEGA_Y,   "L Hip Omega Y");
	dataLog.setItemName(LHIP_OMEGA_Z,	"L Hip Omega Z");
	dataLog.setItemName(LKNEE_RATE,		"L Knee Rate");
	dataLog.setItemName(LANK1_RATE,		"L Ank 1 Rate");
	dataLog.setItemName(LANK2_RATE,		"L Ank 2 Rate");
	dataLog.setItemName(RSHOULD_OMEGA_X,"R Shoulder Omega X");
	dataLog.setItemName(RSHOULD_OMEGA_Y,"R Shoulder Omega Y");
	dataLog.setItemName(RSHOULD_OMEGA_Z,"R Shoulder Omega Z");
	dataLog.setItemName(RELBOW_RATE,	"R Elbow Rate");
	dataLog.setItemName(LSHOULD_OMEGA_X,"L Shoulder Omega X");
	dataLog.setItemName(LSHOULD_OMEGA_Y,"L Shoulder Omega Y");
	dataLog.setItemName(LSHOULD_OMEGA_Z,"L Shoulder Omega Z");
	dataLog.setItemName(LELBOW_RATE,	"L Elbow Rate");
	
	int rateItems[] = {BASE_OMEGA_X,BASE_OMEGA_Y,BASE_OMEGA_Z,BASE_V_X,BASE_V_Y,BASE_V_Z,
						RHIP_OMEGA_X,RHIP_OMEGA_Y,RHIP_OMEGA_Z,RKNEE_RATE,RANK1_RATE,RANK2_RATE,
						LHIP_OMEGA_X,LHIP_OMEGA_Y,LHIP_OMEGA_Z,LKNEE_RATE,LANK1_RATE,LANK2_RATE,
						RSHOULD_OMEGA_X,RSHOULD_OMEGA_Y,RSHOULD_OMEGA_Z,RELBOW_RATE,
						LSHOULD_OMEGA_X,LSHOULD_OMEGA_Y,LSHOULD_OMEGA_Z,LELBOW_RATE};
	
	IntVector rateGroup(angleItems,rateItems+sizeof(rateItems)/sizeof(int));
	dataLog.declareGroup(JOINT_RATES,rateGroup);
	
	dataLog.setItemName(RHIP_TAU_X,		"R Hip Tau X");
	dataLog.setItemName(RHIP_TAU_Y,		"R Hip Tau Y");
	dataLog.setItemName(RHIP_TAU_Z,		"R Hip Tau Z");
	dataLog.setItemName(RKNEE_TAU,		"R Knee Tau");
	dataLog.setItemName(RANK1_TAU,		"R Ank 1 Tau");
	dataLog.setItemName(RANK2_TAU,		"R Ank 2 Tau");
	dataLog.setItemName(LHIP_TAU_X,		"L Hip Tau X");
	dataLog.setItemName(LHIP_TAU_Y,		"L Hip Tau Y");
	dataLog.setItemName(LHIP_TAU_Z,		"L Hip Tau Z");
	dataLog.setItemName(LKNEE_TAU,		"L Knee Tau");
	dataLog.setItemName(LANK1_TAU,		"L Ank 1 Tau");
	dataLog.setItemName(LANK2_TAU,		"L Ank 2 Tau");
	dataLog.setItemName(RSHOULD_TAU_X,	"R Shoulder Tau X");
	dataLog.setItemName(RSHOULD_TAU_Y,	"R Shoulder Tau Y");
	dataLog.setItemName(RSHOULD_TAU_Z,	"R Shoulder Tau Z");
	dataLog.setItemName(RELBOW_TAU,		"R Knee Tau");
	dataLog.setItemName(RSHOULD_TAU_X,	"L Shoulder Tau X");
	dataLog.setItemName(RSHOULD_TAU_Y,	"L Shoulder Tau Y");
	dataLog.setItemName(RSHOULD_TAU_Z,	"L Shoulder Tau Z");
	dataLog.setItemName(RELBOW_TAU,		"L Knee Tau");
	

	dataLog.setItemName(COM_P_X,		"CoM Pos X");
	dataLog.setItemName(COM_P_Y,		"CoM Pos Y");
	dataLog.setItemName(COM_P_Z,		"CoM Pos Z");
	dataLog.setItemName(COM_V_X,		"CoM Vel X");
	dataLog.setItemName(COM_V_Y,		"CoM Vel Y");
	dataLog.setItemName(COM_V_Z,		"CoM Vel Z");
	
	dataLog.setItemName(CM_K_X,			"Cent Ang Mom X");
	dataLog.setItemName(CM_K_Y,			"Cent Ang Mom Y");
	dataLog.setItemName(CM_K_Z,			"Cent Ang Mom Z");
	dataLog.setItemName(CM_L_X,			"Cent Lin Mom X");
	dataLog.setItemName(CM_L_Y,			"Cent Lin Mom Y");
	dataLog.setItemName(CM_L_Z,			"Cent Lin Mom Z");
	
	dataLog.setItemName(LCOP_F_X,		"L CoP Force X");
	dataLog.setItemName(LCOP_F_Y,		"L CoP Force Y");
	dataLog.setItemName(LCOP_F_Z,		"L CoP Force Z");
	dataLog.setItemName(LCOP_P_X,		"L CoP Pos X");
	dataLog.setItemName(LCOP_P_Y,		"L CoP Pos Y");
	dataLog.setItemName(LCOP_N_Z,		"L CoP Mom Z");
	
	dataLog.setItemName(RCOP_F_X,		"R CoP Force X");
	dataLog.setItemName(RCOP_F_Y,		"R CoP Force Y");
	dataLog.setItemName(RCOP_F_Z,		"R CoP Force Z");
	dataLog.setItemName(RCOP_P_X,		"R CoP Pos X");
	dataLog.setItemName(RCOP_P_Y,		"R CoP Pos Y");
	dataLog.setItemName(RCOP_N_Z,		"R CoP Mom Z");
	
	dataLog.setItemName(ZMP_F_X,		"ZMP Force X");
	dataLog.setItemName(ZMP_F_Y,		"ZMP Force Y");
	dataLog.setItemName(ZMP_F_Z,		"ZMP Force Z");
	dataLog.setItemName(ZMP_P_X,		"ZMP Pos X");
	dataLog.setItemName(ZMP_P_Y,		"ZMP Pos Y");
	dataLog.setItemName(ZMP_N_Z,		"ZMP Mom Z");
	
	
	
	/*DataLogger();
	void newRecord();
	
	void assignItem(int code, Float value);
	void assignGroup(int groupCode, const VectorXF & value);
	
	void writeRecords();
	void setFile(const string & fName);
	
	void setMaxGroups(int);
	void setMaxItems(int);
	void declareGroup(int groupCode, IntVector & itemCodes);
	void setItemName(int itemCode, string &);*/
	
}

void logData() {
	dataLog.newRecord();
	

}