/*
 *  humanoidControl.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/5/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "humanoidControl.h"
#include "dm.h"
#include "TaskSpaceController.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
VectorXF q, qdDm, qd;
TaskSpaceController * tsc;
Matrix3F RSup;


extern dmArticulation *G_robot;

void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m);

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
}

void HumanoidControl() {
	
	G_robot->getState(q.data(),qdDm.data());
	
	qdDm.segment(7,NJ) = VectorXF::Ones(NJ);
	G_robot->setState(q.data(),qdDm.data());
	
	
	qd.segment(6,NJ) = qdDm.segment(7,NJ);
	//Dynamechs expresses the velocity of the FB in a spatial basis
	//fixed to the earth. We get it at the FB here to be consistent
	//with our definition of H.
	G_robot->getLink(0)->stxFromInboard(qdDm.data(),qd.data());
	
	tsc->ObtainArticulationData();
	
	for (int i=0; i<14; i++) {
		
		//cout << "p{" << i << "} = \t " << G_robot->m_link_list[i]->link_val2.p_ICS[0] << ", " <<
		//									G_robot->m_link_list[i]->link_val2.p_ICS[1] << ", " <<
		//									G_robot->m_link_list[i]->link_val2.p_ICS[2] << endl;
		
		dmABForKinStruct fk;
		G_robot->forwardKinematics(i,fk);
		
		cout << "p{" << i << "} = \t " << fk.p_ICS[0] << ", " << fk.p_ICS[1] << ", " << fk.p_ICS[2] << endl;
	}
	
	return;
 	
	
	Matrix6XF CentMomMat;
	Vector3F pCom;
	Vector6F cmBias,centMom;
	Float totalMass;
	
	ComputeComInfo(CentMomMat,cmBias,pCom,totalMass);
	centMom = CentMomMat*qd;
	
	Vector3F pComDes;
	pComDes << 2.09, 2, .55;
	
	//cout << "pCoM " << pCom << endl;
	//cout << "Cent Mom " << centMom.transpose() << endl;
	cout << "Total Mass " << totalMass << endl;
	
	//Our task jacobain will be a desired pose + centroidal quantities
	Float jointDiscount = 50;
	tsc->TaskJacobian.resize(6+NJ,NJ+6);
	//tsc->TaskJacobian.resize(6,NJ+6);
	tsc->TaskJacobian.block(0,0,6,NJ+6) = CentMomMat;
	tsc->TaskJacobian.block(6,0,NJ,6) = MatrixXF::Zero(NJ,6);
	tsc->TaskJacobian.block(6,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ)/jointDiscount;
	
	tsc->TaskBias.resize(6+NJ);
	//tsc->TaskBias.resize(6);
	// Compute Task Bias from RNEA Fw Kin
	tsc->TaskBias.segment(0,6) = -cmBias;
	
	Vector3F linMomDotDes = totalMass*15*(pCom - pComDes) - 25*centMom.tail(3);
	Vector3F angMomDotDes = -25*centMom.head(3);
	
	//angMomDotDes.setZero();
	//linMomDotDes.setZero();
	//linMomDotDes(2) += 20;
	
	cout << "ldotdes " << linMomDotDes << endl;
	cout << "kdotdes " << angMomDotDes << endl;
	// Dampen angular and PD CoM
	tsc->TaskBias.segment(0,3) += angMomDotDes;
	tsc->TaskBias.segment(3,3) += linMomDotDes;
	
	// Compute task bias based on PD on joint angles
	VectorXF qDes = VectorXF::Zero(NJ); 					  
	qDes << 0,0,0,0,0,0,0,0,0,0,0,0, 0,2.5,-1.57, 0, 0, -2.4, -1.57, 0;
	Float Kp = 5, Kd = 8;
	tsc->TaskBias.segment(6,NJ) = (Kp * (q.segment(6,NJ) - qDes) - Kd * qd.segment(6,NJ))/jointDiscount;
	

	
	
	tsc->InitializeProblem();
	tsc->Optimize();
	VectorXF tau = tsc->xx.segment(tauStart,NJ);
	VectorXF qdd = tsc->xx.segment(qddStart,NJ+6);
	VectorXF fs = tsc->xx.segment(fStart,6*NS);
	VectorXF lambda = tsc->xx.segment(lambdaStart,NS*NP*NF);
	
	cout << setprecision(5);
	
	cout << "tau = " << tau.transpose() << endl;
	cout << "qdd = " << qdd.transpose() << endl;
	cout << "fs = "  << fs.transpose()  << endl;
	cout << "lambda = "  << lambda.transpose()  << endl;
	
	
	VectorXF a = tsc->TaskJacobian * qdd;
	cout << "a" << endl;
	
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
	cout << "qdd2 = " << qdd2.transpose() << endl << endl << endl;
	
	cout << "CandG " << CandG.transpose() << endl; 
	
	cout << "Gen Contact Force " << generalizedContactForce.transpose() << endl;
	
	cout << "hdot " << (CentMomMat*qdd + cmBias).transpose() << endl;
	
	cout << "cmBias " << cmBias.transpose() << endl;
	
	cout << "qd " << qd.transpose() << endl;
	//VectorXF qdd3 = H.inverse()*(S.transpose() * tau - CandG);
	//FullPivHouseholderQR<MatrixXF> fact(H);
	
	cout <<"fNet    \t" << (fs.segment(3,3) + fs.segment(9,3)).transpose() << endl;
	Vector3F g;
	g << 0,0,-9.81;
	cout <<"hdot - mg\t" << (CentMomMat*qdd + cmBias).segment(3,3).transpose() -  totalMass * g.transpose()<< endl;
	
	
	// Form Joint Input and simulate
	VectorXF jointInput = VectorXF::Zero(NJ+7);
	jointInput.segment(7,NJ) = tau;
	
	
	//VectorXF qdd3   = H.fullPivHouseholderQr().solve(S.transpose() * tau - CandG);
	//cout << "qdd3 = " << qdd3.transpose() << endl;
	
	//VectorXF state = VectorXF::Zero(2*(NJ+7));
	//state.segment(0,NJ+7) = q;
	//state.segment(NJ+7,NJ+7) = qd;
	
	//VectorXF stateDot = VectorXF::Zero(2*(NJ+7));
	
	//jointInput.segment(7,NJ) = VectorXF::Ones(NJ);
	//G_robot->setJointInput(jointInput.data());
	
	
	// Process qdds
	//G_robot->dynamics(state.data(),stateDot.data());
	//
	//
	//VectorXF qdds = VectorXF::Zero(NJ+6);
	//qdds.segment(0,6) = stateDot.segment(NJ+7,6);
	
	//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl;
	
	//qdds.segment(3,3) -= cr3(qd.segment(0,3))*qd.segment(3,3);
	//qdds.segment(6,NJ) = stateDot.segment(NJ+7*2,NJ);
	//qdds(5)+=9.81;
	
	//cout << "qdds = " << qdds.transpose()  << endl;
	
	//cout << "CandG " << endl << CandG << endl;
	
	//cout << setprecision(6);
	//cout << "I_0^C = " << endl << G_robot->H.block(0,0,6,6) << endl;
	exit(-1);
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
	
	Matrix6F IC0;
	CrbToMat(fb->I_C, IC0);
	cout << "FB IC0 in CoM Func " << endl << IC0 << endl;
	
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

void ComputeGrfInfo(Vector3F& pZMP, Vector6F & fZMP) {
	
	fZMP.setZero();
	Vector6F icsForce;
	SpatialVector localForce;
	Float * nICS = icsForce.data();
	Float * fICS =nICS+3;
	Float * nLoc =localForce;
	Float * fLoc =nLoc + 3;
	CartesianVector tmp, tmp2;
	
	for (int k1 = 0; k1 < NS; k1++) {
		dmRigidBody * body = (dmRigidBody *) G_robot->getLink(tsc->SupportIndices[k1]);
		LinkInfoStruct * listruct = G_robot->m_link_list[tsc->SupportIndices[k1]];
		
		for (int k2 = 0; k2 < body->getNumForces(); k2++) {
			body->getForce(k2)->computeForce(listruct->link_val2,localForce);
			
			// Apply Spatial Force Transform Efficiently
			// Rotate Quantities
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,nLoc,nICS);
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,fLoc,fICS);
			
			// Add the r cross f
			CROSS_PRODUCT(listruct->link_val2.p_ICS,fICS,tmp);
			ADD_CARTESIAN_VECTOR(nICS,tmp);
			
			fZMP+=icsForce;
		}
	}
	
	Float nx = fZMP(0), ny = fZMP(1), fz = fZMP(5);
	
	pZMP(0) = - ny/fz;
	pZMP(1) =   nx/fz;
	pZMP(2) = 0;
	
	fZMP(0) = 0;
	fZMP(1) = 0;
	fZMP(2)+= fZMP(4)*pZMP(0) - fZMP(3)*pZMP(1);
	
	
}