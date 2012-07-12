/*
 *  humanoidControl.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/5/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"

#include "HumanoidController.h"
#include "dm.h"
#include "dmTime.h"
#include "TaskSpaceController.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
#include "CubicSplineTrajectory.h"
#include "HumanoidDataLogger.h"
#include "DataLogger.h"
//#define CONTROL_DEBUG


#define STATE_SIZE (NJ+7+4)
#define RATE_SIZE  (NJ+6)
/*Matrix3F RSup;

TaskSpaceController * tsc;
Matrix6XF CentMomMat;
Vector3F pCom, vCom;
Vector6F cmBias,centMom, hDotDes, hDotOpt;
VectorXF q, qdDm, qd,tau, qdd, fs, lambda;
VectorXF pComDes(3), vComDes(3);

Float totalMass;



CubicSplineTrajectory ComTrajectory;*/

HumanoidController::HumanoidController(dmArticulation * robot) : TaskSpaceController(robot) {
	q.resize(STATE_SIZE);
	qdDm.resize(STATE_SIZE);
	qd.resize(RATE_SIZE);
	
	SupportIndices.resize(NS);
	SupportIndices[0] = 5;
	SupportIndices[1] = 9;
	
	contactState.resize(NS);
	slidingState.resize(NS);
	
	
	Matrix3F RSup;
	RSup << 0,0,1,0,1,0,-1,0,0;
	
	XformVector Xforms(NS);
	for (int k=0; k<NS; k++) {
		Xforms[k].resize(6,6);
		
		dmRigidBody * link = (dmRigidBody*) G_robot->getLink(SupportIndices[k]);
		dmContactModel * dmContactLattice = (dmContactModel *) link->getForce(0); 
		
		Vector3F pRel;
		dmContactLattice->getContactPoint(0,pRel.data());
		pRel(1) = 0; pRel(2) = 0;
		Xforms[k].block(0,0,3,3) = RSup;
		Xforms[k].block(0,3,3,3) = Matrix3F::Zero();
		Xforms[k].block(3,0,3,3) = -RSup*cr3(pRel);
		Xforms[k].block(3,3,3,3) = RSup;
	}
	SupportXforms = Xforms;
	
	pFoot.resize(NS);
	pDesFoot.resize(NS);
	
	vFoot.resize(NS);
	vDesFoot.resize(NS);
	
	RFoot.resize(NS);
	RDesFoot.resize(NS);
	
	aFoot.resize(NS);
	aDesFoot.resize(NS);
	
	for (int i=0; i<NS; i++) {
		pFoot[i].resize(3);
		pDesFoot[i].resize(3);
		
		vFoot[i].resize(6);
		vDesFoot[i].resize(6);
		
		aFoot[i].resize(6);
		aDesFoot[i].resize(6);
	}
	//ComTrajectory.setSize(3);
}

void HumanoidController::ControlInit()
{
	//Update State Variables for Control
	{
		artic->getState(q.data(),qdDm.data());
		int N = G_robot->getTrueNumDOFs();
		
		extractQd(qdDm, qd);
		
		// Test code for velocity initialization
		{
			//qdDm.segment(0,3) = Vector3F::Constant(2);
			//qdDm(3) = -1;
			//qdDm(4) = 2;
			//qdDm(5) = -.8;
			//qdDm.segment(7,NJ) = VectorXF::Ones(NJ);
			//G_robot->setState(q.data(),qdDm.data());
		}
		
		
		//Dynamechs expresses the velocity of the FB in the ICS coordinate
		//We get it in the FB coord here to be consistent
		//with our definition of H.
		G_robot->getLink(0)->rtxFromInboard(qdDm.data(),qd.data());
		G_robot->getLink(0)->rtxFromInboard(qdDm.data()+3,qd.data()+3);
		
		ObtainArticulationData();
		UpdateVariableBounds();
		UpdateInitialConstraintBounds();
		
		for (int i =0; i<NS; i++) {
			dmRigidBody * link = (dmRigidBody * ) artic->getLink(SupportIndices[i]);
			dmContactModel * cont = (dmContactModel *) link->getForce(0);
			contactState[i] = 0;
			slidingState[i] = 0;
			for (int j=0; j<NP; j++) {
				if (cont->getContactState(j)) {
					contactState[i]+=(int) pow(2.0,j);
				}
				if (cont->getSlidingState(j)) {
					slidingState[i]+=(int) pow(2.0,j);
				}
			}
		}
	}
	
	// Initialize Task Quantities
	{
		TaskJacobian.resize(6+NJ+12,NJ+6);
		TaskJacobian.setZero();
		TaskBias.resize(6+NJ+12);
	}
	
	// Init Centroidal State
	{
		ComputeComInfo(CentMomMat,cmBias,pCom,totalMass);
		ComputeGrfInfo(grfInfo);
		
		centMom = CentMomMat*qd;
		vCom = centMom.tail(3)/totalMass;
	}
	
	// Init Foot State
	{
		for (int i=0; i<NS; i++) {
			InertialKinematicInfo(SupportIndices[i], RFoot[i], pFoot[i], vFoot[i]);
		}
	}
}

void HumanoidController::HumanoidControl(ControlInfo & ci) {
	int taskRow = 0;
	Float discountFactor = 1;
	dmTimespec tv1, tv2, tv3, tv4;
	
	
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
		UpdateObjective();
		UpdateHPTConstraintBounds();
		UpdateConstraintMatrix();
		
		dmGetSysTime(&tv3);
		Optimize();
		tau = xx.segment(tauStart,NJ);
		qdd = xx.segment(qddStart,NJ+6);
		fs = xx.segment(fStart,6*NS);
		lambda = xx.segment(lambdaStart,NS*NP*NF);
		
		// Extract Results
		hDotOpt = CentMomMat*qdd + cmBias;
		
		// Form Joint Input and simulate
		VectorXF jointInput = VectorXF::Zero(STATE_SIZE);
		
		// Extact Desired ZMP info
		zmpWrenchOpt.setZero();
		Vector6F icsForce, localForce;
		Float * nICS = icsForce.data(), * nLoc = localForce.data();
		Float * fICS =	nICS+3, * fLoc = nLoc+3;
		
		for (int k1 = 0; k1 < NS; k1++) {
			LinkInfoStruct * listruct = artic->m_link_list[SupportIndices[k1]];
			CartesianVector tmp;
			
			localForce = SupportXforms[k1].transpose()*fs.segment(6*k1,6);			
				
			// Apply Spatial Force Transform Efficiently
			// Rotate Quantities
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,nLoc,nICS);
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,fLoc,fICS);
				
			// Add the r cross f
			CROSS_PRODUCT(listruct->link_val2.p_ICS,fICS,tmp);
			ADD_CARTESIAN_VECTOR(nICS,tmp);
			
			zmpWrenchOpt+=icsForce;
		}
		transformToZMP(zmpWrenchOpt,zmpPosOpt);
		
		
		int k = 7;
		// Skip over floating base (i=1 initially)
		for (int i=1; i<artic->m_link_list.size(); i++) {
			LinkInfoStruct * linki = artic->m_link_list[i];
			if (linki->dof) {
				//cout << "Link " << i << " dof = " << linki->dof << endl;
				//cout << "qd = " << qdDm.segment(k,linki->dof).transpose() << endl;
				jointInput.segment(k,linki->dof) = tau.segment(linki->index_ext-6,linki->dof);
				k+=linki->link->getNumDOFs();
			}
		}
		
		//cout << "Tau = " << tau.transpose() << endl;
		//cout << "Joint input = " << jointInput.transpose() << endl;
		//exit(-1);
		//jointInput.segment(7,NJ) = tau;
		G_robot->setJointInput(jointInput.data());
		ComputeActualQdd(qddA);
		dmGetSysTime(&tv4);
	}
	
	//Populate Control Info Struct
	{
		ci.calcTime = timeDiff(tv1,tv2);
		ci.setupTime = timeDiff(tv2,tv3);
		ci.optimTime = timeDiff(tv3,tv4);
		ci.totalTime = timeDiff(tv1,tv4);
		ci.iter      = iter;
	}
	
	
	#ifdef CONTROL_DEBUG
	// Debug Code
	{
		cout << "q " << q.transpose() << endl;
		cout << "qd " << qdDm.transpose() << endl;
		cout << "qd2 " << qd.transpose() << endl;
		cout << "Task Bias " << TaskBias << endl;
		
		//cout << "H = " << endl << G_robot->H << endl;
		cout << "CandG = " << endl << G_robot->CandG.transpose() << endl;
		
		if (simThread->sim_time > 0) {
			
			cout << setprecision(5);
			
			MSKboundkeye key;
			double bl,bu;
			for (int i=0; i<numCon; i++) {
				MSK_getbound(task, MSK_ACC_VAR, i, &key, &bl, &bu);
				cout << "i = " << i;
				
				switch (key) {
					case MSK_BK_FR:
						cout << " Free " << endl;
						break;
					case MSK_BK_LO:
						cout << " Lower Bound " << endl;
						break;
					case MSK_BK_UP:
						cout << " Upper Bound " << endl;
						break;
					case MSK_BK_FX:
						cout << " Fixed " << endl;
						break;
					case MSK_BK_RA:
						cout << " Ranged " << endl;
						break;
					default:
						cout << " Not sure(" << key <<  ")" << endl;
						break;
				}
				cout << bl << " to " << bu << endl;
			}
			
			cout << "x(57) = " << xx(57) << endl;
			cout << "tau = " << tau.transpose() << endl;
			cout << "qdd = " << qdd.transpose() << endl;
			cout << "fs = "  << fs.transpose()  << endl;
			//cout << "lambda = "  << lambda.transpose()  << endl;
			
			
			VectorXF a = TaskJacobian * qdd;
			//cout << "a" << endl;
			
			VectorXF e = TaskJacobian * qdd - TaskBias;
			cout << "e = " << e.transpose() << endl;
			
			MatrixXF H = G_robot->H;
			VectorXF CandG = G_robot->CandG;
			
			MatrixXF S = MatrixXF::Zero(NJ,NJ+6);
			S.block(0,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ);
			
			VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
			for (int i=0; i<NS; i++) {
				generalizedContactForce += SupportJacobians[i].transpose()*fs.segment(6*i,6);
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
			exit(-1);
		}
		
		
		
		
		// Old Debug Code
		{
			VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
			
			Matrix6F X;
			MatrixXF Jac;
			X.setIdentity();
			
			for (int i=0; i<NS; i++) {
				int linkIndex = SupportIndices[i];
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
	
	if (frame->logDataCheckBox->IsChecked()) {
		humanoid->logData();
	}
	
	//exit(-1);
}
		
void HumanoidController::extractQd(const VectorXF & dotDm, VectorXF & dot)
{
	dot.resize(RATE_SIZE);
	int k = 0;
	for (int i=0; i<artic->m_link_list.size(); i++) {
		LinkInfoStruct * linki = artic->m_link_list[i];
		if (linki->dof) {
			//cout << "Link " << i << " dof = " << linki->dof << endl;
			//cout << "qd = " << qdDm.segment(k,linki->dof).transpose() << endl;
			dot.segment(linki->index_ext,linki->dof) = dotDm.segment(k,linki->dof);
			k+=linki->link->getNumDOFs();
		}
	}
}

void HumanoidController::ComputeActualQdd(VectorXF & qddA)
{
	VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
	
	Matrix6F X;
	MatrixXF Jac;
	X.setIdentity();
	
	for (int i=0; i<NS; i++) {
		int linkIndex = SupportIndices[i];
		artic->computeJacobian(linkIndex,X,Jac);
		dmRigidBody * link = (dmRigidBody *) artic->getLink(linkIndex);
		
		for (int j=0; j< link->getNumForces(); j++) {
			dmForce * f = link->getForce(j);
			Vector6F fContact;
			f->computeForce(artic->m_link_list[linkIndex]->link_val2,fContact.data());
			generalizedContactForce += Jac.transpose()*fContact;
		}
	}
	
	//cout << "J' f = " << generalizedContactForce.transpose() << endl;
	
	//MatrixXF S = MatrixXF::Zero(NJ,NJ+6);
	//S.block(0,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ);
	//VectorXF qdd3   = artic->H.inverse()*(S.transpose() * tau - artic->CandG + generalizedContactForce);
	//cout << "qdd3 = " << qdd3.transpose() << endl;
	
	VectorXF state = VectorXF::Zero(2*STATE_SIZE);
	state.segment(0,STATE_SIZE) = q;
	state.segment(STATE_SIZE,STATE_SIZE) = qdDm;
	
	VectorXF stateDot = VectorXF::Zero(2*STATE_SIZE);
	
	//Process qdds
	G_robot->dynamics(state.data(),stateDot.data());
	extractQd(stateDot.segment(STATE_SIZE,STATE_SIZE), qddA);
	
	//qdds.segment(0,6) = stateDot.segment(STATE_SIZE,STATE_SIZE);
	//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl;
	
	qddA.segment(3,3) -= cr3(qdDm.segment(0,3))*qdDm.segment(3,3);
	Matrix3F ics_R_fb;
	copyRtoMat(G_robot->m_link_list[0]->link_val2.R_ICS,ics_R_fb);
	qddA.head(3) = ics_R_fb.transpose() * qddA.head(3);
	qddA.segment(3,3) = ics_R_fb.transpose() * qddA.segment(3,3);
	
	//cout << "qdd3 = " << qdd3.transpose() << endl;
	//cout << "qdds = " << qddA.transpose()  << endl;
	//exit(-1);
	
	//cout << "CandG " << endl << CandG << endl;
	
	//cout << setprecision(6);
	//cout << "I_0^C = " << endl << G_robot->H.block(0,0,6,6) << endl;
	//exit(-1);	
	
}

void HumanoidController::ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m) {
	
	LinkInfoStruct * fb = artic->m_link_list[0];
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
	
	
	Cmm = XT * artic->H.topRows(6);
	
	IC0 = XT*artic->H.block(0,0,6,6)*XT.transpose();
	IBarC0 = IC0.block(0,0,3,3);
	
	bias = XT * (fb->link_val2.f-artic->H.block(0,0,6,6)*fb->link_val2.a);
}

void HumanoidController::ComputeGrfInfo(GRFInfo & grf) {
	
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
		dmRigidBody * body = (dmRigidBody *) artic->getLink(SupportIndices[k1]);
		LinkInfoStruct * listruct = artic->m_link_list[SupportIndices[k1]];
		
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

void HumanoidController::InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS)
{
	LinkInfoStruct * link = artic->m_link_list[index];
	copyRtoMat(link->link_val2.R_ICS, RtoICS);
	
	COPY_P_TO_VEC(link->link_val2.p_ICS, pICS);
	
	// Compute the link velocity in the ICS
	spatVelICS.head(3) = RtoICS * link->link_val2.v.head(3);
	spatVelICS.tail(3) = RtoICS * link->link_val2.v.tail(3);
}