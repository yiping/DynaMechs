// HumanoidController.h
// Nov 27, 2012
// YL

#ifndef __HUMANOID_CONTROLLER_H__
#define __HUMANOID_CONTROLLER_H__

#include "TaskSpaceControllerA.h"
#include "TaskSpaceControllerConic.h"

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

//class HumanoidController : public TaskSpaceControllerA 
class HumanoidController : public TaskSpaceControllerConic 
{
public:
	HumanoidController(dmArticulation * robot);
	void ControlInit();
	void HumanoidControl();
	void ComputeGrfInfo(GRFInfo & grf);
	void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m); 

	void extractQd(const VectorXF & dotDm, VectorXF & dot);
	void ComputeActualQdd(VectorXF & qddA);
	void InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS);

	Vector3F getComPos(){ return pCom; }
	Vector3F getComPosDes(){ return pComDes;}
	Vector3F getZmpPos(){ return grfInfo.pZMP;}
	Vector3F getZmpPosOpt(){return zmpPosOpt;}

protected:
	// robot properties that are updated every control step
	Matrix6XF CentMomMat;	// centroidal momentum matrix expressed in ICS
	Matrix6F IC0;			// CRB inertia (6x6) expressed in ICS
	Matrix3F IBarC0;
	
	Vector3F pCom, vCom;
	Vector6F centMom;
	Vector6F cmBias;		//  \dot{A}_G \dot{q}
	VectorXF q;
	VectorXF qdDm, qd;
	VectorXF qddA;
	Float totalMass;

	vector<VectorXF > pFoot;
	vector<VectorXF > vFoot;
	vector<VectorXF > aFoot;
	vector<Matrix3F > RFoot;

	IntVector contactState;
	IntVector slidingState;

	GRFInfo grfInfo;

	// optimal quantities


	Vector6F hDotOpt;
	Vector6F zmpWrenchOpt;
	Vector3F zmpPosOpt;


	// desired setpoints
	vector<VectorXF > aDesFoot;
	vector<Vector3F > pDesFoot;
	vector<Matrix3F > RDesFoot;
	vector<VectorXF > vDesFoot;
	
	vector<VectorXF> posDesJoint;
	vector<Matrix3F> RDesJoint;
	vector<VectorXF> rateDesJoint;
	vector<VectorXF> accDesJoint;

	Vector6F hDes;
	Vector6F hDotDes;
	Vector3F kDotDes;

	VectorXF aComDes, pComDes, vComDes;
	VectorXF kComDes;
	
	// PD gains
	vector<Float> kpFoot; 
	vector<Float> kdFoot;

	vector<Float> kpJoint;
	vector<Float> kdJoint;
	
	Float controlTime;
	
	
private:
	void computeActualQdd(VectorXF & qdd);
	
};







#endif
