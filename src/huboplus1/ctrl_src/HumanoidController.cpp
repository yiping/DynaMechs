// HumanoidController.cpp
// Nov 27, 2012
// YL

#include "globalVariables.h"
#include "globalFunctions.h"
#include "control_defs.h"

#include "dm.h"
#include "dmTime.h"

#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"

#include "HumanoidController.h"
#include "TaskSpaceController.h"

#include "DataLogger.h"


//#define CONTROL_DEBUG


// Pat's humanoid
//#define STATE_SIZE (NJ+7+4)
//#define RATE_SIZE  (NJ+6)

// Hubo+
#define STATE_SIZE (NJ+7)
#define RATE_SIZE  (NJ+6)


HumanoidController::HumanoidController(dmArticulation * robot) : TaskSpaceControllerA(robot) 
{


	// Create Linearized Friction Cone Basis
	// in support frame (z-up)
	FrictionBasis = MatrixXF::Zero(6,NF);
	for (int j=0; j<NF; j++) 
	{
		double angle = (j * 2 * M_PI) / NF;
		FrictionBasis(3,j) = MU*cos(angle);
		FrictionBasis(4,j) = MU*sin(angle);
		FrictionBasis(5,j) = 1;
	}

	// Initialize Support Jacobians
	SupportJacobians.resize(NS);
	for (int i=0; i<NS; i++) 
	{
		SupportJacobians[i] = MatrixXF::Zero(6,1); 
	}
	


	// Indicate support bodies
	SupportIndices.resize(NS);	// number of support
	SupportIndices[0] = 11;		// support body 0
	SupportIndices[1] = 20;		// support body 1
	
	contactState.resize(NS);
	slidingState.resize(NS);
	
	// Construct transform from ankle frame to support frame
	Matrix3F RSup;		// Sup - support
	// RSup << 0,0,1,0,1,0,-1,0,0;
	RSup << 0,0,1, 0,-1,0, 1,0,0;
	
	XformVector Xforms(NS);
	for (int k=0; k<NS; k++) 
	{
		Xforms[k].resize(6,6);
		
		dmRigidBody * link = (dmRigidBody*) artic->getLink(SupportIndices[k]);
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
	
	// Construct force transform from each contact point to support origin
	PointForceXforms.resize(NS);
	for (int i=0; i<NS; i++) 
	{
		PointForceXforms[i].resize(NJ);
		
		//cout << "Getting forces for link " << SupportIndices[i] << endl;
		dmRigidBody * linki = (dmRigidBody*) artic->m_link_list[SupportIndices[i]]->link;
		dmContactModel * dmContactLattice = (dmContactModel *) linki->getForce(0); 
		
		//cout << "Assigning Temporary Matricies" << endl;
		Matrix3F RSup = SupportXforms[i].block(0,0,3,3);
		Matrix3F tmpMat = SupportXforms[i].block(3,0,3,3)*RSup.transpose();
		Vector3F piRelSup;
		crossExtract(tmpMat,piRelSup);
		
		//cout << "pirelsup " << endl << piRelSup << endl;
		
		for (int j=0; j<NP; j++) 
		{
			Vector3F pRel, tmp;
				
			//Tmp is now the contact point location relative to the body coordinate
			dmContactLattice->getContactPoint(j,tmp.data());
			
			// Point of contact (relative to support origin) in support coordinates
			pRel = RSup*tmp + piRelSup;
			
			PointForceXforms[i][j].setIdentity(6,6);
			PointForceXforms[i][j].block(0,3,3,3) = cr3(pRel);
		}
	}


	// Initialize variables
	q.resize(STATE_SIZE);
	qdDm.resize(STATE_SIZE);	// joint rate DM
	qd.resize(RATE_SIZE);		// joint rate

	grfInfo.localContacts = 0;
	
	pFoot.resize(NS);
	vFoot.resize(NS);
	aFoot.resize(NS);
	RFoot.resize(NS);	// 3x3


	pDesFoot.resize(NS);
	vDesFoot.resize(NS);
	RDesFoot.resize(NS);
	aDesFoot.resize(NS);
	
	kpFoot.resize(NS);
	kdFoot.resize(NS);
	
	for (int i=0; i<NS; i++) 
	{
		pFoot[i].resize(3);
		vFoot[i].resize(6);
		aFoot[i].resize(6);

		pDesFoot[i].resize(3);
		vDesFoot[i].resize(6);
		aDesFoot[i].resize(6);
		
		aDesFoot[i].setZero();
		vDesFoot[i].setZero();
		pDesFoot[i].setZero();
		
		RDesFoot[i].setZero();	// 3x3
		
		kpFoot[i] = 0;
		kdFoot[i] = 0;
	}

	
	aComDes.resize(3);
	aComDes.setZero();
	
	vComDes.resize(3);
	vComDes.setZero();
	
	pComDes.resize(3);
	pComDes.setZero();
	
	kComDes.resize(3);	// angular momentum around com
	kComDes.setZero();


	int numLinks = artic->getNumLinks();
	
	RDesJoint.resize(numLinks);
	posDesJoint.resize(numLinks);
	rateDesJoint.resize(numLinks);
	accDesJoint.resize(numLinks);
	kpJoint.resize(numLinks);
	kdJoint.resize(numLinks);

	artic->getTrueNumDOFs();	// fill bodyi->index_ext, bodyi->dof	
	for (int i=0; i<numLinks; i++) 
	{
		LinkInfoStruct * bodyi = artic->m_link_list[i];
		int dof = bodyi->dof;
		if (dof != 0) 
		{
			if (dof == 6) 
			{
				posDesJoint[i].setZero(3);	// linear position of FB
			}
			else if (dof ==3)
			{
				// Do nothing, since only uses RDes
			}
			else if (dof == 1)
			{
				posDesJoint[i].setZero(1);
			}
			rateDesJoint[i].setZero(dof);
			accDesJoint[i].setZero(dof);
		}
	}
	
}

//! This function is called at the beginning of each control step
//! It collects(computes) necessary dynamic information of the articulated system
void HumanoidController::ControlInit()
{
	//Update State Variables for Control
	{
		artic->getState(q.data(),qdDm.data());
		int N = artic->getTrueNumDOFs();
		
		extractQd(qdDm, qd);
		
		// Test code for velocity initialization
		//{
			//qdDm.segment(0,3) = Vector3F::Constant(2);
			//qdDm(3) = -1;
			//qdDm(4) = 2;
			//qdDm(5) = -.8;
			//qdDm.segment(7,NJ) = VectorXF::Ones(NJ);
			//artic->setState(q.data(),qdDm.data());
		//}
		
		
		//Dynamechs expresses the velocity of the FB in the ICS coordinate
		//We get it in the FB coord here to be consistent
		//with our definition of H.
		artic->getLink(0)->rtxFromInboard(qdDm.data(),qd.data());
		artic->getLink(0)->rtxFromInboard(qdDm.data()+3,qd.data()+3);
		
		ObtainArticulationData();	// compute Support Jacobians, H, CandG


		
		// get contact and sliding state for each contact point	
		for (int i =0; i<NS; i++) 
		{
			dmRigidBody * link = (dmRigidBody * ) artic->getLink(SupportIndices[i]);
			dmContactModel * cont = (dmContactModel *) link->getForce(0);
			contactState[i] = 0;
			slidingState[i] = 0;
			for (int j=0; j<NP; j++) 
			{
				if (cont->getContactState(j)) 
				{
					contactState[i]+=(int) pow(2.0,j);
				}
				if (cont->getSlidingState(j)) 
				{
					slidingState[i]+=(int) pow(2.0,j);
				}
			}
		}
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
		for (int i=0; i<NS; i++) 
		{
			InertialKinematicInfo(SupportIndices[i], RFoot[i], pFoot[i], vFoot[i]);
		}
	}

	// Initialize Task Quantities
	{
		TaskJacobian.resize(6+(NJ+6)+12,NJ+6);
		TaskJacobian.setZero();
		TaskBias.resize(6+(NJ+6)+12);
	}
	
	UpdateVariableBounds();
	UpdateInitialConstraintBounds();
	

}



void HumanoidController::HumanoidControl() 
{

	
	// Perform Optimization
	{

		UpdateConstraintMatrix();
		
		int maxPriorityLevels = OptimizationSchedule.maxCoeff();
		const int numTasks = OptimizationSchedule.size();
		
		if (maxPriorityLevels > 0) 
		{
			for (int level=1; level<=maxPriorityLevels; level++) 
			{
				taskConstrActive.setZero(numTasks);
				taskOptimActive.setZero(numTasks);
				bool runOpt = false;
				for (int i=0; i<numTasks;i ++) 
				{
					if (OptimizationSchedule(i) == level) 
					{
						taskOptimActive(i) = 1;
						runOpt = true;
					}
					else if ((OptimizationSchedule(i) < level) && (OptimizationSchedule(i) > -1)) 
					{
						taskConstrActive(i) = 1;
					}
				}
				
				if (runOpt) 
				{
					UpdateObjective();
					UpdateHPTConstraintBounds();
					
					//cout << "Optimizing level " << level << endl;
					Optimize();
					for (int i=0; i<numTasks;i ++) 
					{
						if (OptimizationSchedule(i) == level) 
						{
							TaskBias(i) += TaskError(i);
							//cout << "Optimization Level " << level << " task error " << i << " = " << TaskError(i) << endl; 
						}
					}
				}
			}
		}

		
		
		

		
		/// Compute optimal quantities
		// desired change of centroidal momentum
		hDotOpt = CentMomMat*qdd + cmBias;
		
		// Desired ZMP info
		zmpWrenchOpt.setZero();
		Vector6F icsForce, localForce;
		Float * nICS = icsForce.data();		// ICS
		Float * fICS =	nICS+3; 
		Float * nLoc = localForce.data();	// Local
		Float * fLoc = nLoc+3;
		
		for (int k1 = 0; k1 < NS; k1++) 
		{
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
		
		

		// Form Joint Input and simulate
		VectorXF jointInput = VectorXF::Zero(STATE_SIZE);
		
		int k = 7;
		// Skip over floating base (i=1 initially)
		for (int i=1; i<artic->m_link_list.size(); i++) 
		{
			LinkInfoStruct * linki = artic->m_link_list[i];
			if (linki->dof) 
			{
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
		artic->setJointInput(jointInput.data());

		/// verification
		ComputeActualQdd(qddA);
		
	}
	

	
	
	#ifdef CONTROL_DEBUG
	// Debug Code
	{
		
	}
	#endif
	

}
		

//! Qd received from DM includes dummy entries, this function removes it.
//! Author: PMW 
//! Make sure you call this function only after dmArticulation::getTrueNumDOFs()
//! dmArticulation::computeCandG() internally first calls dmArticulation::getTrueNumDOFs(), so that will do.
void HumanoidController::extractQd(const VectorXF & dotDm, VectorXF & dot)
{
	dot.resize(RATE_SIZE);
	int k = 0;
	for (int i=0; i<artic->m_link_list.size(); i++) 
	{
		LinkInfoStruct * linki = artic->m_link_list[i];
		if (linki->dof) 
		{
			//cout << "Link " << i << " dof = " << linki->dof << endl;
			//cout << "qd = " << qdDm.segment(k,linki->dof).transpose() << endl;
			dot.segment(linki->index_ext,linki->dof) = dotDm.segment(k,linki->dof);
			k += linki->link->getNumDOFs();
		}
	}
}



//! Compute the actual qdd (ground reaction forces come from contact model)
//! Author: PMW
void HumanoidController::ComputeActualQdd(VectorXF & qddA)
{
	/*
	VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
	
	Matrix6F X;
	MatrixXF Jac;
	X.setIdentity();
	
	for (int i=0; i<NS; i++) 
	{
		int linkIndex = SupportIndices[i];
		artic->computeJacobian(linkIndex,X,Jac);
		dmRigidBody * link = (dmRigidBody *) artic->getLink(linkIndex);
		
		for (int j=0; j< link->getNumForces(); j++) 
		{
			dmForce * f = link->getForce(j);
			Vector6F fContact;
			f->computeForce(artic->m_link_list[linkIndex]->link_val2,fContact.data());
			generalizedContactForce += Jac.transpose()*fContact;
		}
	}
	
	cout << "J' f = " << generalizedContactForce.transpose() << endl;
	
	MatrixXF S = MatrixXF::Zero(NJ,NJ+6);
	S.block(0,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ);
	VectorXF qdd3   = artic->H.inverse()*(S.transpose() * tau - artic->CandG + generalizedContactForce);
	cout << "qdd3 = " << qdd3.transpose() << endl;
	*/


	VectorXF state = VectorXF::Zero(2*STATE_SIZE);
	state.segment(0,STATE_SIZE) = q;
	state.segment(STATE_SIZE,STATE_SIZE) = qdDm;
	
	VectorXF stateDot = VectorXF::Zero(2*STATE_SIZE);
	
	//Process qdds
	artic->dynamics(state.data(),stateDot.data());
	extractQd(stateDot.segment(STATE_SIZE,STATE_SIZE), qddA);
	
	//qdds.segment(0,6) = stateDot.segment(STATE_SIZE,STATE_SIZE);
	//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl;
	
	qddA.segment(3,3) -= cr3(qdDm.segment(0,3))*qdDm.segment(3,3);
	Matrix3F ics_R_fb;
	copyRtoMat(artic->m_link_list[0]->link_val2.R_ICS,ics_R_fb);
	qddA.head(3) = ics_R_fb.transpose() * qddA.head(3);
	qddA.segment(3,3) = ics_R_fb.transpose() * qddA.segment(3,3);
	
	//cout << "qdd3 = " << qdd3.transpose() << endl;
	//cout << "qdds = " << qddA.transpose()  << endl;
	//exit(-1);
	
	//cout << "CandG " << endl << CandG << endl;
	
	//cout << setprecision(6);
	//cout << "I_0^C = " << endl << artic->H.block(0,0,6,6) << endl;
	//exit(-1);	
	
}


//! Only call this function after you run ObtainArticulationData()
//! Author: PMW
void HumanoidController::ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m) 
{
	
	LinkInfoStruct * fb = artic->m_link_list[0];	// floating base
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
	pFBRelCoM(0) = -fb->I_C.h[0]/m;		// fb frame and com frame have the same orientation
	pFBRelCoM(1) = -fb->I_C.h[1]/m;
	pFBRelCoM(2) = -fb->I_C.h[2]/m;
	
	//Matrix6F IC0;
	//CrbToMat(fb->I_C, IC0);
	//cout << "FB IC0 in CoM Func " << endl << IC0 << endl;
	
	// pCom = pBase + pComRelBase
	pCom -= ics_R_fb*pFBRelCoM;
	
	int N = artic->H.cols();
	Cmm.resize(6,N);
	
	
	Matrix6F XT = Matrix6F::Zero();
	XT.block(0,0,3,3) = ics_R_fb;
	XT.block(0,3,3,3) = ics_R_fb*cr3(pFBRelCoM);
	XT.block(3,3,3,3) = ics_R_fb;	// XT is force transformation from FB to COM 
	
	
	Cmm = XT * artic->H.topRows(6);	//expressed in ICS coordinate
	
	IC0 = XT*artic->H.block(0,0,6,6)*XT.transpose();
	IBarC0 = IC0.block(0,0,3,3);
	
	// These quantities (f,a) for the floating base are the result of
	// inverse dynamics with qdd=0. As a result, fb->link_val2.f 
	// contains the first 6 rows of CandG!
	
	bias = XT * (fb->link_val2.f-artic->H.block(0,0,6,6)*fb->link_val2.a); // bias = \dot{A}_G \dot{q}
}





//! Populate information about ground reaction forces
//! Author: PMW
void HumanoidController::ComputeGrfInfo(GRFInfo & grf) 
{
	
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
	grf.footWrenches.resize(NS);
	grf.footJacs.resize(NS);
	
	for (int k1 = 0; k1 < NS; k1++) 
	{
		dmRigidBody * body = (dmRigidBody *) artic->getLink(SupportIndices[k1]);
		LinkInfoStruct * listruct = artic->m_link_list[SupportIndices[k1]];
		
		artic->computeJacobian(SupportIndices[k1], Matrix6F::Identity(), grf.footJacs[k1]);	// get foot jacobian
		
		for (int k2 = 0; k2 < body->getNumForces(); k2++) 
		{
			body->getForce(k2)->computeForce(listruct->link_val2,localForce);	// get contact force in local coordinate
			grf.footWrenches[k1] << localForce[0],localForce[1],localForce[2],
									localForce[3],localForce[4],localForce[5];
			
			
			//Float * p = localForce;
			//cout << "Local Force " << tsc->SupportIndices[k1];
			//cout << " = " << localForce[0] << " , " << localForce[1] << " , " << localForce[2] << " , ";
			//cout << localForce[3] << " , " << localForce[4] << " , " << localForce[5] << endl;
			
			// Apply Spatial Force Transform Efficiently
			// Rotate Quantities
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,nLoc,nICS);	// nICS = R_ICS * nLoc
			APPLY_CARTESIAN_TENSOR(listruct->link_val2.R_ICS,fLoc,fICS);	// fICS = R_ICS * fLoc
			
			// Add the r cross f
			CROSS_PRODUCT(listruct->link_val2.p_ICS,fICS,tmp);
			ADD_CARTESIAN_VECTOR(nICS,tmp);								// nICS = pICS X fICS + nICS
			
			//cout << "ICS Force " << icsForce.transpose() << endl;
			
			fZMP+=icsForce;	// intermediate values
			
			{
				Matrix3F RtoICS;
				copyRtoMat(listruct->link_val2.R_ICS, RtoICS);
				
				grfInfo.fCoPs[k1] = RtoICS * grf.footWrenches[k1].tail(3);
				
				// from ankle to support origin
				Vector6F supportFrameWrench = SupportXforms[k1].inverse().transpose()*grf.footWrenches[k1];	
				Vector3F supportFrameCoP;
				// NOTE: SupportXforms[i] is the 6D transform from ankle frame to support frame
				
				
				transformToZMP(supportFrameWrench,supportFrameCoP);	// get foot COP in support frame
																	// note "supportFrameWrench" is modified, it becomes "foot ZMP wrench"
				grfInfo.nCoPs[k1] = supportFrameWrench(2);
				
				Vector3F pRel;
				((dmContactModel *) body->getForce(k2))->getContactPoint(0,pRel.data());
				pRel(1) = 0; pRel(2) = 0;	// only preserve x component /// !!!
				
				Vector3F footFrameCoP = pRel + SupportXforms[k1].block(0,0,3,3).transpose()*supportFrameCoP;// express the foot COP in foot frame
				
				grfInfo.pCoPs[k1] = RtoICS*footFrameCoP;
				
				grfInfo.pCoPs[k1](0) += listruct->link_val2.p_ICS[0];
				grfInfo.pCoPs[k1](1) += listruct->link_val2.p_ICS[1];
				grfInfo.pCoPs[k1](2) += listruct->link_val2.p_ICS[2];// foot COP in ICS
			
			}
			
			
		}
	}
	transformToZMP(fZMP,grf.pZMP);
	grf.fZMP = fZMP.tail(3);
	grf.nZMP = fZMP(2);
}

//! Acquire designated link kinematic information (retrieve ICS_R_link, ICS_p_link, and compute the link velocity in the ICS)\n
//! Author: PMW 
void HumanoidController::InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS)
{
	LinkInfoStruct * link = artic->m_link_list[index];
	copyRtoMat(link->link_val2.R_ICS, RtoICS);
	
	COPY_P_TO_VEC(link->link_val2.p_ICS, pICS);
	
	// Compute the link velocity in the ICS
	spatVelICS.head(3) = RtoICS * link->link_val2.v.head(3);
	spatVelICS.tail(3) = RtoICS * link->link_val2.v.tail(3);
}
