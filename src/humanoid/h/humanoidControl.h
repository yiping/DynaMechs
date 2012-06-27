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

#include "GlobalDefines.h"
#include "TaskSpaceController.h"

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

extern TaskSpaceController * tsc;
extern Matrix6XF CentMomMat;
extern Vector3F pCom, vCom;
extern Vector6F cmBias,centMom, hDotDes, hDotOpt;
extern VectorXF q, qdDm, qd,tau, qdd, fs, lambda;
extern VectorXF pComDes, vComDes;


void HumanoidControl(ControlInfo &);
void initControl();


#endif