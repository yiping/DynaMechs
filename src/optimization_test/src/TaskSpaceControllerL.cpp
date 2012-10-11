
//  TaskSpaceControllerL.cpp
//  YL
//  8/14/12


#include "TaskSpaceControllerL.h"
#include "dmRigidBody.hpp"
#include "dmContactModel.hpp"
#include "math_funcs.h"
#include "globalFunctions.h"



#define TAU_START     0
#define TAU_END       (NJ-1)
#define QDD_START     NJ
#define QDD_END       (2*NJ + 5)
#define LAMBDA_START  (2*NJ + 6)
#define LAMBDA_END    (2*NJ+5 + NS*NP*NF)



//#define DEBUG_TSCL


TaskSpaceControllerL::TaskSpaceControllerL(dmArticulation * robot) 
{

#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL constructor --"<<endl;
	cout<<"NVAR = "<<NVAR<<endl;
#endif 
	artic = robot;

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
		SupportJacobians[i] = MatrixXF::Zero(6,1); // necessary?
	}

	//
	ST = MatrixXF::Zero(NJ+6, NJ);
	ST.block(6, 0, NJ, NJ) = MatrixXF::Identity(NJ, NJ);


	// Indicate support bodies
	SupportIndices.resize(NS);	// number of support
	SupportIndices[0] = 5;		// support body 0
	SupportIndices[1] = 9;		// support body 1

	contactState.resize(NS);
	slidingState.resize(NS);

	// Construct transform from ankle frame to support frame
	Matrix3F RSup;	// Sup - support
	RSup << 0,0,1,0,1,0,-1,0,0;
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
		Vector3F pfRelSup;
		crossExtract(tmpMat,pfRelSup);
		//cout << "pfrelsup " << endl << pfRelSup << endl;
		
		for (int j=0; j<NP; j++) 
		{
			Vector3F pRel, tmp;
				
			// tmp is now the contact point location relative to the ankle coordinate
			dmContactLattice->getContactPoint(j,tmp.data());
			
			// Point of contact (relative to support origin) in support coordinates
			pRel = RSup*tmp + pfRelSup;
			
			PointForceXforms[i][j].setIdentity(6,6);
			PointForceXforms[i][j].block(0,3,3,3) = cr3(pRel);
		}
	}


	// Initialize variables
	q.resize(STATE_SIZE);
	qdDm.resize(STATE_SIZE);	// joint rate DM
	qd.resize(RATE_SIZE);		// joint rate

	pFoot.resize(NS);
	vFoot.resize(NS);
	aFoot.resize(NS);
	RFoot.resize(NS);	//3x3

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
	kpJoint.resize(numLinks);
	kdJoint.resize(numLinks);

	RDesJoint.resize(numLinks);
	posDesJoint.resize(numLinks);
	rateDesJoint.resize(numLinks);
	accDesJoint.resize(numLinks);

	// cout<< " numLinks = "<<numLinks<<endl;
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



	D0 = MatrixXF::Zero(NJ + NS*NP*NF, NVAR);	
	D0.topLeftCorner(NJ,NJ) = MatrixXF::Identity(NJ,NJ);	
	D0.bottomRightCorner(NS*NP*NF, NS*NP*NF) = MatrixXF::Identity(NS*NP*NF, NS*NP*NF);	

	//CBar = MatrixXF::Identity(NVAR, NVAR);
	//dBar = VectorXF::Zero(NVAR);

}

TaskSpaceControllerL::~TaskSpaceControllerL() 
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL destructor --"<<endl;
#endif 


}


void TaskSpaceControllerL::ObtainArticulationData()
{
	artic->computeH();
	artic->computeCandG();	// runs ID assuming qdd = 0
		
	for (int i=0; i<NS; i++) 
	{
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}


//! This function is called at the beginning of each control step
//! It collects(computes) necessary dynamic information of the articulated system
//! for the following prioritized controller
void TaskSpaceControllerL::ControlInit()
{

	artic->getState(q.data(),qdDm.data());	// For Quaternion link, qdDM has an extra dummy entry
	int N = artic->getTrueNumDOFs();	// for Pat's humanoid: N = 26
	extractQd(qdDm, qd);	// remove any dummy entry

	// DynaMechs expresses the velocity of the FB in the ICS coordinate
	// We get it in the FB coord here to be consistent
	// with our definition of H.
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
			if (cont->getContactState(j)) // encode contact state
			{
				contactState[i]+=(int) pow(2.0,j);
			}
			if (cont->getSlidingState(j)) 
			{
				slidingState[i]+=(int) pow(2.0,j);
			}
		}
	}

	// Init Centroidal State
	ComputeComInfo(CentMomMat,cmBias,pCom,totalMass);
	ComputeGrfInfo(grfInfo);
	
	centMom = CentMomMat*qd;
	vCom = centMom.tail(3)/totalMass;

	// Init Foot State
	for (int i=0; i<NS; i++) 
	{
		InertialKinematicInfo(SupportIndices[i], RFoot[i], pFoot[i], vFoot[i]);
	}


}

void TaskSpaceControllerL::RobotControl() 
{
	
	/// Perform Optimization

	SetInitialConstraintBounds();
	SetInitialVariableBounds();
		
	Reset();
	for(int i = 0; i<TaskSchedule.size();i++)
	{
#ifdef DEBUG_TSCL
		cout<<"\n ------- Task "<<i<<" -------"<<endl;
#endif

		UpdateObjective(TaskJacobians[TaskSchedule[i]], TaskBiases[TaskSchedule[i]]);	
		UpdateConstraintMatrix();
		UpdateConstraintBounds();
		UpdateVariableBounds();
		//solver.InspectQPproblem();
		OptimizeSingleTier();

		//cout<<" === "<<dBar.transpose()<<endl;
	}

	tauOpt = dBar.segment(TAU_START,NJ);
	qddOpt = dBar.segment(QDD_START,NJ+6);
	lambdaOpt = dBar.segment(LAMBDA_START,NS*NP*NF);


	/// Compute optimal quantities
	// desired change of centroidal momentum
	hDotOpt = CentMomMat*qddOpt + cmBias;

	// desired ZMP info
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
		
		// localForce = SupportXforms[k1].transpose()*fs.segment(6*k1,6);	
		Vector6F fSup = Vector6F::Zero();
		for (int j = 0; j< NP; j++)
		{
			fSup += PointForceXforms[k1][j] * FrictionBasis * lambdaOpt.segment(NP*NF*k1 + NF*j, NF) ; 
		}	
		localForce = SupportXforms[k1].transpose() * fSup;	// contact force in foot(ankle) coordinate
			
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
			jointInput.segment(k,linki->dof) = tauOpt.segment(linki->index_ext-6,linki->dof);
			k += linki->link->getNumDOFs();	// fake dof
		}
	}
		
	//cout << "Tau = " << tau.transpose() << endl;
	//cout << "Joint input = " << jointInput.transpose() << endl;
	//exit(-1);
	//jointInput.segment(7,NJ) = tau;
	artic->setJointInput(jointInput.data());


	/// verification

	ComputeActualQdd(qddA);

#ifdef DEBUG_TSCL
/*	cout<<"totalMass = "<<totalMass<<endl;
	cout<<"pCom        = "<<pCom.transpose()<<endl;
	cout<<"zmpWrench   = "<<zmpWrenchOpt.transpose()<<endl;
	cout<<"zmpPos      = "<<zmpPosOpt.transpose()<<endl;
	//cout<<"PComDes     = "<<pComDes.transpose()<<endl;
	//cout<<"aDesFoot[0] = "<<aDesFoot[0].transpose()<<endl;
	//cout<<"aDesFoot[1] = "<<aDesFoot[1].transpose()<<endl;
	cout<<"This one should be zero: "<< (DynCon*dBar - artic->CandG ).transpose()<<endl;
	cout<<"hDotOpt:"<<endl<<hDotOpt<<endl;
	cout<<"tauOpt:"<<endl<<tauOpt.transpose()<<endl;
	cout<<"qddOpt:"<<endl<<qddOpt.transpose()<<endl;
	cout<<"qddA: "<<endl<<qddA.transpose()<<endl;
	cout<<"lambdaOpt:"<<endl<<lambdaOpt.transpose()<<endl;*/
#endif 

}



void TaskSpaceControllerL::Reset()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::Reset() --"<<endl;
#endif 

	// form dynamic constraint matrix
	JTXTV = MatrixXF::Zero(NJ+6, NS*NP*NF);
	for (int i = 0; i < NS; i++) 
	{
	
		MatrixXF XTVi(6, NP*NF);
		//Loop through contact points for support body i
		for (int j = 0; j < NP; j++) 
		{
			XTVi.block(0,NF*j,6,NF) = PointForceXforms[i][j]*FrictionBasis;
		}

		JTXTV.block(0, NP*NF*i, NJ+6, NP*NF) = SupportJacobians[i].transpose() *XTVi;
	}

	DynCon = MatrixXF::Zero(NJ+6, 2*NJ + 6 + NS*NP*NF);
	DynCon.block(0, 0, NJ+6, NJ) = ST;
	DynCon.block(0, NJ, NJ+6, NJ+6) = -artic->H; 
	DynCon.block(0, 2*NJ+6, NJ+6, NS*NP*NF) = JTXTV;


	// compute C0
	MatrixXF C0;
	getNullSpace(DynCon, C0);


	// compute d0
	// assume all taus are zeros and all lambdas are .1s
	VectorXF RHS = artic->CandG - JTXTV*VectorXF::Ones(NS*NP*NF);
	VectorXF qddvec;
	solveInverse(-artic->H, RHS, qddvec);

	VectorXF d0 = VectorXF::Zero(NJ+NJ+6+NS*NP*NF);
	d0.segment(0, NJ) = VectorXF::Zero(NJ);
	d0.segment(NJ, NJ+6) = qddvec;
	d0.segment(NJ+NJ+6, NS*NP*NF) = VectorXF::Ones(NS*NP*NF);

	// Dynamic Constraint imposed 
	dBar = d0;
	CBar = C0; 

#ifdef DEBUG_TSCL
/*	IOFormat OctaveFmt(FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
	cout<<"CBar is " <<CBar.rows()<<" x "<<CBar.cols()<<endl;
	cout<<"d0 = (full precision)"<< endl<<(dBar.transpose()).format(OctaveFmt)<<endl;
	cout<<"-H = (full precision)"<<endl<<(-artic->H).format(OctaveFmt)<<endl;
	cout<<"DynCon = (full precision)"<<endl<<DynCon.format(OctaveFmt)<<endl;
	cout<<"CandG = (full precision)"<<endl<<(artic->CandG.transpose()).format(OctaveFmt)<<endl;
	cout<<"CBar = (full precision)"<<endl<<CBar.format(OctaveFmt)<<endl;*/
#endif
}

void TaskSpaceControllerL::SetInitialConstraintBounds()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::SetInitialConstraintBounds() --"<<endl;
#endif 
	c_bk = VectorXbk::Zero(NJ+NS*NP*NF);
	c_bl = VectorXF::Zero(NJ+NS*NP*NF);
	c_bu = VectorXF::Zero(NJ+NS*NP*NF);
	
	int i;
	// 'tau' bounds - (ranged)
	// 'qdd' bounds - (free) 
	// 'lambda' bounds - (lower bound)

	for (i=0; i<NJ; i++) 
	{
		c_bk[i] = MSK_BK_RA;
		c_bl[i] = -600;
		c_bu[i] = +600;
	}

	for (i=NJ; i<NJ+NS*NP*NF; i++) 
	{
		c_bk[i] = MSK_BK_LO;
		c_bl[i] = 0;
		c_bu[i] = +MSK_INFINITY;			
	}	

}


void TaskSpaceControllerL::SetInitialVariableBounds()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::SetInitialVariableBounds() --"<<endl;
#endif 


}

void TaskSpaceControllerL::UpdateObjective(const MatrixXF &TaskJacobian, const VectorXF &TaskBias) 
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::UpdateObjective(...) --"<<endl;
#endif 
	MatrixXd JtBar = TaskJacobian * CBar;
	getNullSpace(JtBar, C);

	if ( isFullRank(JtBar))
	{
		//cout<<"\nJtBar is full rank! \n"<<endl;
		JtBarFullColRank = true;
	}

	MatrixXd JtBarT = JtBar.transpose();
	MatrixXd Q = JtBarT * JtBar; 

	// modify Q
	MatrixXF Qm = MatrixXF::Zero(NVAR, NVAR);
	Qm.block(NJ, NJ, NJ+6, NJ+6) += MatrixXF::Identity(NJ+6,NJ+6) * 0.001;
	Qm.block(NJ+NJ+6, NJ+NJ+6, NS*NP*NF, NS*NP*NF) += MatrixXF::Identity(NS*NP*NF,NS*NP*NF) * 0.01;
	Q += CBar.transpose()*Qm*CBar;

	VectorXd btBar = TaskBias - TaskJacobian * dBar; 
	VectorXd c = -JtBarT*btBar; 

	double cfix = 0.5*btBar.dot(btBar);

#ifdef DEBUG_TSCL
/*	cout<<endl<<"Update objective: "<<endl;
	IOFormat DisplayFmt(FullPrecision, 0, "  ","\n", "    ", " ", "", "\n");
	cout<<"JtBarColFullRank (bool): "<<JtBarFullColRank<<endl;
	cout<<"CBar =  "<<endl<<CBar.format(DisplayFmt)<<endl;
	cout<<"JtBar = "<<endl<<JtBar.format(DisplayFmt)<<endl;
	cout<<"btBar = "<<endl<<btBar.transpose().format(DisplayFmt)<<endl;
	cout<<"Q = "<<endl<<Q.format(DisplayFmt)<<endl;
	cout<<"Null space of JtBar (C): "<<endl<<C.format(DisplayFmt)<<endl;
	cout<<"Current JtBar has rank "<<showRank(JtBar)<<endl<<endl;*/
#endif 

	solver.UpdateObjective(Q, c, cfix);

}


void TaskSpaceControllerL::UpdateConstraintMatrix()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::UpdateConstraintMatrix() --"<<endl;
#endif 
	solver.UpdateConstraintMatrix(D0 * CBar);
}


void TaskSpaceControllerL::UpdateConstraintBounds()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::UpdateConstraintBounds() --"<<endl;
#endif 
	solver.UpdateConstraintBounds(c_bk, c_bl - D0*dBar, c_bu - D0*dBar    );
}





//! The index in this function refers to the index of support body
void TaskSpaceControllerL::AssignFootMaxLoad(int index, double maxLoad) 
{
	for (int i = 0; i< NS*NP*NF; i++)
	{
		solver.ModifySingleConstraintBound(NJ + i, MSK_BK_UP, 0, maxLoad);
	}
}


void TaskSpaceControllerL::UpdateVariableBounds()
{
#ifdef DEBUG_TSCL
	cout<<" -- TaskSpaceControllerL::UpdateVariableBounds() --"<<endl;
#endif 
	int a = CBar.cols();	//!!!
	v_bk = VectorXbk::Constant(a, MSK_BK_FR);
	v_bl = VectorXF::Constant(a, 0);
	v_bu = VectorXF::Constant(a, 0);

	solver.UpdateVariableBounds(v_bk, v_bl, v_bu    );
}







//! Only call this function after you run ObtainArticulationData()
//! Author: PMW
void TaskSpaceControllerL::ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m) 
{
	
	LinkInfoStruct * fb = artic->m_link_list[0];	// floating base
	dmLink * fbLink = fb->link;
	
	CartesianTensor fb_R_ics;
	
	// pCom initally contains the position of the torso in the i.c.s
	fbLink->getPose(fb_R_ics,pCom.data());
	
	
	Matrix3F ics_R_fb;
	ics_R_fb << fb_R_ics[0][0] , fb_R_ics[1][0] , fb_R_ics[2][0],
		        fb_R_ics[0][1] , fb_R_ics[1][1] , fb_R_ics[2][1],
		        fb_R_ics[0][2] , fb_R_ics[1][2] , fb_R_ics[2][2];
	
	Vector3F pFBRelCoM;
	m = fb->I_C.m;
	pFBRelCoM(0) = -fb->I_C.h[0]/m;		// fb frame and com frame have the same orientation
	pFBRelCoM(1) = -fb->I_C.h[1]/m;
	pFBRelCoM(2) = -fb->I_C.h[2]/m;		//YL?
	
	//Matrix6F IC0;
	//CrbToMat(fb->I_C, IC0);
	
	// pCom = pBase + pComRelBase
	pCom -= ics_R_fb*pFBRelCoM;
	
	int N = artic->H.cols();
	Cmm.resize(6,N);
	
	
	Matrix6F XT = Matrix6F::Zero();
	XT.block(0,0,3,3) = ics_R_fb;
	XT.block(0,3,3,3) = ics_R_fb*cr3(pFBRelCoM);
	XT.block(3,3,3,3) = ics_R_fb;	// XT is force transformation from FB to COM expressed in ICS coordinate
	
	
	Cmm = XT * artic->H.topRows(6);
	
	IC0 = XT*artic->H.block(0,0,6,6)*XT.transpose();
	IBarC0 = IC0.block(0,0,3,3);

	// These quantities (f,a) for the floating base are the result of
	// inverse dynamics with qdd=0. As a result, fb->link_val2.f 
	// contains the first 6 rows of CandG vector!

	bias = XT * (fb->link_val2.f - artic->H.block(0,0,6,6)*fb->link_val2.a);	// bias = \dot{A}_G \dot{q}
}


//! Populate information about ground reaction forces
//! Author: PMW
void TaskSpaceControllerL::ComputeGrfInfo(GRFInfo & grf)
{
	
	Vector6F fZMP = Vector6F::Zero();
	Vector6F icsForce;
	SpatialVector localForce;
	Float * nICS = icsForce.data();
	Float * fICS = nICS+3;
	Float * nLoc = localForce;
	Float * fLoc = nLoc + 3;
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
			ADD_CARTESIAN_VECTOR(nICS,tmp);							// nICS = pICS X fICS + nICS
			
			//cout << "ICS Force " << icsForce.transpose() << endl;
			
			fZMP+=icsForce;	// intermediate values
			
			{
				Matrix3F RtoICS;
				copyRtoMat(listruct->link_val2.R_ICS, RtoICS);
				
				grfInfo.fCoPs[k1] = RtoICS * grf.footWrenches[k1].tail(3);


				// from ankle to support origin
				Vector6F supportFrameWrench = SupportXforms[k1].inverse().transpose()*grf.footWrenches[k1];
				Vector3F supportFrameCoP;
				
				
				transformToZMP(supportFrameWrench,supportFrameCoP);	// get foot COP in support frame
																	// note "supportFrameWrench" is modified, it becomes "wrenchZMP"
				grfInfo.nCoPs[k1] = supportFrameWrench(2);

				
				Vector3F pRel;
				((dmContactModel *) body->getForce(k2))->getContactPoint(0,pRel.data());
				pRel(1) = 0; pRel(2) = 0;
				Vector3F footFrameCoP = pRel + SupportXforms[k1].block(0,0,3,3).transpose()*supportFrameCoP;	// express the foot COP in foot frame
				

				grfInfo.pCoPs[k1] = RtoICS*footFrameCoP;	
				grfInfo.pCoPs[k1](0) += listruct->link_val2.p_ICS[0];
				grfInfo.pCoPs[k1](1) += listruct->link_val2.p_ICS[1];
				grfInfo.pCoPs[k1](2) += listruct->link_val2.p_ICS[2];	// foot COP in ICS
			
			}
			
			
		}
	}
	transformToZMP(fZMP,grf.pZMP);
	grf.fZMP = fZMP.tail(3);
	grf.nZMP = fZMP(2);
}


//! Qd received from DM includes dummy entries, this function removes it.
//! Author: PMW 
//! Make sure you call this function only after dmArticulation::getTrueNumDOFs()
//! dmArticulation::computeCandG() internally first calls dmArticulation::getTrueNumDOFs(), so that will do.  
void TaskSpaceControllerL::extractQd(const VectorXF & dotDm, VectorXF & dot)
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
			dot.segment(linki->index_ext,linki->dof) = dotDm.segment(k,linki->dof); // linki->dof is the 'true' DOF of the link
			k+=linki->link->getNumDOFs();	// this is the 'DM' DOF, not the real DOF.
		}
	}
}


//! Compute the actual qdd (ground reaction forces come from contact model)
//! Author: PMW 
void TaskSpaceControllerL::ComputeActualQdd(VectorXF & qddA)
{

	/*
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
	//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl; 	// omega cross v
	
	qddA.segment(3,3) -= cr3(qdDm.segment(0,3))*qdDm.segment(3,3);	// convert to traditional acceleration
	Matrix3F ics_R_fb;
	copyRtoMat(artic->m_link_list[0]->link_val2.R_ICS,ics_R_fb);	
	qddA.head(3) = ics_R_fb.transpose() * qddA.head(3);				// expressed in ICS
	qddA.segment(3,3) = ics_R_fb.transpose() * qddA.segment(3,3);
	
	//cout << "qdd3 = " << qdd3.transpose() << endl;
	//cout << "qdds = " << qddA.transpose()  << endl;
	//exit(-1);
	
	//cout << "CandG " << endl << CandG << endl;
	
	//cout << setprecision(6);
	//cout << "I_0^C = " << endl << artic->H.block(0,0,6,6) << endl;
	//exit(-1);	
	
}



//! Acquire designated link kinematic information (retrieve ICS_R_link, ICS_p_link, and compute the link velocity in the ICS)
//! Author: PMW 
void TaskSpaceControllerL::InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS)
{
	LinkInfoStruct * link = artic->m_link_list[index];
	copyRtoMat(link->link_val2.R_ICS, RtoICS);
	
	COPY_P_TO_VEC(link->link_val2.p_ICS, pICS);
	
	// Compute the link velocity in the ICS
	spatVelICS.head(3) = RtoICS * link->link_val2.v.head(3);
	spatVelICS.tail(3) = RtoICS * link->link_val2.v.tail(3);
}


