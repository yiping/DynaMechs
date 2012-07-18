/*
 *  Humanoid.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/28/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HUMANOID_H__
#define __HUMANOID_H__
#include "GlobalTypes.h"
#include <dm.h>

class Humanoid : public virtual ArticulationSpecializer {

public:
	
protected:
	Matrix6XF CentMomMat;
	Matrix6F IC0;
	Matrix3F IBarC0;
	
	Vector3F pCom, vCom;
	Vector6F cmBias;
	VectorXF q, qdDm, qd, qddA;
	
private:
	
	
	
};


#endif