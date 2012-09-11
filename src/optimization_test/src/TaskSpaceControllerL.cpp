
//  TaskSpaceControllerL.cpp
//  YL
//  8/14/12

#include "TaskSpaceControllerL.h"
#include "dmRigidBody.hpp"
#include "math_funcs.h"

#define NVAR (NJ+ NJ+6 + NS*NP*NF)

#define TAU_START     0
#define TAU_END       (NJ-1)
#define QDD_START     NJ
#define QDD_END       (2*NJ + 5)
#define LAMBDA_START  (2*NJ + 6)
#define LAMBDA_END    (2*NJ+5 + NS*NP*NF)



TaskSpaceControllerL::TaskSpaceControllerL(dmArticulation * robot) 
{
	artic = robot;

	solver = QPsolver();

	// Create Linearized Friction Cone Basis
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
	ST.block(6, 0, NJ, NJ) = MatrixXF:Identity(NJ, NJ);
}

TaskSpaceControllerL::ObtainArticulationData()
{
	artic->computeH();
	artic->computeCandG();
		
	for (int i=0; i<NS; i++) 
	{
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}


TaskSpaceControllerL::Reset()
{
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
	DynCon.block(0, NJ+6, NJ+6, NJ+6) = -artic->H; 
	DynCon.block(0, 2*NJ+6, NJ+6, NS*NP*NF) = JTXTV);


	// compute C0
	getNullSpace(DynCon, C0);


	// compute d0
	// assume all taus are zeros and all lambdas are 1s
	VectorXF RHS = artic->CandG - JTXTV*VectorXF::Ones(NS*NP*NF);
	VectorXF qddvec;
	solveInverse(artic->H, RHS, qddvec);

	VectorXF d0 = VectorXF::Zero(NJ+NJ+6+NS*NP*NF);
	d0.segment(0, NJ) = VectorXF::Zero(NJ);
	d0.segment(NJ, NJ+6) = qddvec;
	d0.segment(NJ+6, NS*NP*NF) = VectorXF::Ones(NS*NP*NF);

}


void TaskSpaceControllerL::UpdateObjective() 
{

	
	MatrixXd Jt = MatrixXd::Zero( TaskJacobian.rows(), NVAR);
	Jt.block(0, NJ, TaskJacobian.rows(), NJ+6) = TaskJacobian;

	for (int i=0; i<taskOptimActive.size(); i++) 
	{
		if (taskOptimActive(i)==0) 
		{
			Jt.row(i).setZero();
		}
		else 
		{
			Jt.row(i)*=TaskWeight(i)*TaskWeight(i);
		}
	}
	MatrixXd JtBar = Jt * CBar;
	MatrixXd JtBarT = JtBar.transpose();

	MatrixXd Q = JtBarT * JtBar; 
	Q.block(NJ, NJ, NJ+6, NJ+6) += MatrixXF::Identity(NJ+6,NJ+6) * 0.001;
	Q.block(NJ+NJ+6, NJ+NJ+6, NS*NP*NF, NS*NP*NF) += MatrixXF::Identity(NS*NP*NF,NS*NP*NF) * 0.01;

	VectorXd btBar = TaskBias - Jt * dBar; 

	VectorXd c = -JtBarT*btBar; //qddStart = NJ

	double cfix = 0.5*btBar.dot(btBar);

	solver.UpdateObjective(Q, c, cfix);


}


void TaskSpaceControllerL::UpdateConstraintMatrix()
{
	// for TSC-L, you basically do not need to do anything in this function.
}



void TaskSpaceControllerL::UpdateVariableBounds() 
{
	VectorXbk  bkx(NVAR);
	VectorXF   blx(NVAR);
	VectorXF   bux(NVAR);
	
	// Set up 'tau' bounds (ranged)
	for (i=TAU_START; i<=TAU_END; i++) 
	{
		bkx[i] = MSK_BK_RA;
		blx[i] = -500;
		bux[i] = +500;
	}
	
	// Set up 'qdd' bounds - (free) 
	for (i=QDD_START; i<=QDD_END; i++) 
	{
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	// Set up 'lambda' bounds - (lower bound)
	for (i=LAMBDA_START; i<=LAMBDA_END; i++) 
	{
		bkx[i] = MSK_BK_LO;
		blx[i] = 0;
		bux[i] = +MSK_INFINITY;			
	}	

	solver.UpdateVariableBounds(bkx, blx, bux);
}







