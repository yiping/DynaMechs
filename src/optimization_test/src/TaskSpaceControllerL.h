//!  TaskSpaceControllerL.h
//!  Inspired by PMW's TaskSpaceControllerA/B 
//!  YL
//!  8/14/12
//!  This file implements the task space controller described in 
//!  "Feature-Based Locomotion Controllers" paper 
//!  by Martin de Lasa's, 2010, ACM Transactions on Graphics 

#ifndef __TASK_SPACE_CONTROLLER_L_H__
#define __TASK_SPACE_CONTROLLER_L_H__

#define NVAR (NJ+ NJ+6 + NS*NP*NF)

#define STATE_SIZE (NJ+7+4)
#define RATE_SIZE  (NJ+6)

#include "control_globals.h"
#include "dmArticulation.hpp"
#include "PrioritizedController.h"
#include "QPsolver.h"


class TaskSpaceControllerL: public PrioritizedController
{
public:
	TaskSpaceControllerL(dmArticulation * robot);
	virtual ~TaskSpaceControllerL();

	// Dynamics
	void ObtainArticulationData();
	void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m);
	void ComputeGrfInfo(GRFInfo & grf);
	void extractQd(const VectorXF & dotDm, VectorXF & dot);
	void ComputeActualQdd(VectorXF & qddA);
	void InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS);
	void AssignFootMaxLoad(int index, double maxLoad);

	// reimplementation 
	virtual void Reset();
	virtual void SetInitialConstraintBounds();
	virtual void SetInitialVariableBounds();
	virtual void UpdateObjective(const MatrixXF &TaskJacobian, const VectorXF &TaskBias);
	virtual void UpdateConstraintMatrix();
	virtual void UpdateConstraintBounds();
	virtual void UpdateVariableBounds();


	void ControlInit();
	void RobotControl();

	

protected:
	dmArticulation * artic;	

	// support related
	MatrixXF FrictionBasis;
	IntVector SupportIndices;
	XformVector SupportXforms;
	vector<MatrixXF > SupportJacobians;
	vector<XformVector > PointForceXforms; // force transformation (from contact point to support frame origin)  


	// robot properties that are updated every control step
	Matrix6XF CentMomMat;	// centroidal momentum matrix expressed in ICS
	Matrix6F IC0;			// CRB inertia (6x6) expressed in ICS
	Matrix3F IBarC0;		// 

	Vector3F pCom, vCom;
	Vector6F centMom;	
	Vector6F cmBias;		//  \dot{A}_G \dot{q}
	Float totalMass;
	VectorXF q;
	VectorXF qdDm, qd;
	VectorXF qddA;
	

	vector<VectorXF> pFoot;
	vector<VectorXF> vFoot;
	vector<VectorXF> aFoot;
	vector<Matrix3F> RFoot;

	IntVector contactState;
	IntVector slidingState;

	GRFInfo grfInfo;

	// optimal quantitiies
	VectorXF tauOpt, qddOpt, lambdaOpt;
	Vector6F hDotOpt;
	Vector6F zmpWrenchOpt;
	Vector3F zmpPosOpt;


	// PD gains
	vector<Float> kpFoot;
	vector<Float> kdFoot;
	vector<Float> kpJoint;
	vector<Float> kdJoint;
	Float kpCM, kdCM;	// linear momentum
	Float kdAM;


	// desired setpoints 
	vector<VectorXF> aDesFoot;
	vector<VectorXF> vDesFoot;
	vector<Vector3F> pDesFoot;
	vector<Matrix3F> RDesFoot;

	vector<VectorXF> posDesJoint;
	vector<Matrix3F> RDesJoint;
	vector<VectorXF> rateDesJoint;
	vector<VectorXF> accDesJoint;

	Vector6F hDes;
	Vector6F hDotDes;
	Vector3F kDotDes;

	VectorXF pComDes;
	VectorXF vComDes;
	VectorXF aComDes;
	VectorXF kComDes;

private:

	MatrixXF ST;
	MatrixXF DynCon;

	
};

#endif
