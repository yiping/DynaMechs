/*
 *  humanoidControl.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/5/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HUMANOID_CONTROL_H__
#define __HUMANOID_CONTROL_H__

#include "TaskSpaceControllerA.h"
#include "TaskSpaceControllerB.h"
#include "TaskSpaceControllerC.h"
#include "CustomTaskSpaceController.h"
#include "Humanoid.h"

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

class HumanoidController : public Humanoid , public TaskSpaceControllerB {
public:
	HumanoidController(dmArticulation * robot);
	void HumanoidControl(ControlInfo &);
	void ComputeGrfInfo(GRFInfo & grf);
	void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m); 
	void ControlInit();
	void extractQd(const VectorXF & dotDm, VectorXF & dot);
	void ComputeActualQdd(VectorXF & qddA);
	
	Vector6F centMom, hDotDes, hDotOpt, hDes;
	Vector3F kDotDes;
	VectorXF pComDes, vComDes;
	Float totalMass;
	
	GRFInfo grfInfo;
	

	void InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS);
	vector<Float > kpFoot, kdFoot;
	vector<VectorXF > aDesFoot;
	vector<Vector3F > pDesFoot;
	vector<Matrix3F > RDesFoot;
	vector<VectorXF > vDesFoot;
	vector<VectorXF > pFoot;
	vector<VectorXF > vFoot;
	vector<VectorXF > aFoot;
	
	vector<Matrix3F > RFoot;
	
	
	vector<VectorXF> posDesJoint;
	vector<Matrix3F> RDesJoint;
	vector<VectorXF> rateDesJoint;
	vector<VectorXF> accDesJoint;
	vector<Float > kpJoint;
	vector<Float > kdJoint;
	
	IntVector contactState;
	IntVector slidingState;
	
	
	
	Vector6F zmpWrenchOpt;
	Vector3F zmpPosOpt;
	
	VectorXF aComDes, kComDes;
	
	Float controlTime;
	
private:
	void computeActualQdd(VectorXF & qdd);
	
};







#endif