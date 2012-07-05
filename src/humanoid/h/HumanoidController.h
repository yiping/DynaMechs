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

#include "TaskSpaceController.h"
#include "Humanoid.h"

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

class HumanoidController : public Humanoid , public TaskSpaceController {
public:
	HumanoidController(dmArticulation * robot);
	void HumanoidControl(ControlInfo &);
	void ComputeGrfInfo(GRFInfo & grf);
	void ControlInit();
	
	Vector6F centMom, hDotDes, hDotOpt;
	VectorXF fs, lambda;
	VectorXF pComDes, vComDes;
	Float totalMass;
protected:
	void InertialKinematicInfo(int index, Matrix3F & RtoICS, VectorXF & pICS, VectorXF & spatVelICS);
	vector<Float > kpFoot, kdFoot;
	vector<VectorXF > aDesFoot;
	vector<Vector3F > pDesFoot;
	vector<Matrix3F > RDesFoot;
	vector<VectorXF > vDesFoot;
	vector<VectorXF > pFoot;
	vector<VectorXF > vFoot;
	vector<Matrix3F > RFoot;
	
};







#endif