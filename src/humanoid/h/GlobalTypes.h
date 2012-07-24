/*
 *  GlobalTypes.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/13/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __GLOBAL_TYPES__
#define __GLOBAL_TYPES__

#include <dm.h>
#include <dmArticulation.hpp>
#include <vector>
//#include "BalanceDemoStateMachine.h"
//#include "HumanoidDataLogger.h"


typedef struct GRFInfoStruct {
	Vector3F pZMP;
	Vector3F fZMP;
	Float nZMP;
	
	int localContacts;
	vector<Vector3F > pCoPs;
	vector<Vector3F > fCoPs;
	vector<Float > nCoPs;
	
	vector<Vector6F> footWrenches;
	vector<MatrixXF> footJacs;
} GRFInfo;


typedef struct ControlInfoStruct   {
	int iter;
	double calcTime;
	double setupTime;
	double optimTime;
	double totalTime;
}  ControlInfo ;


typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;

typedef vector<string > StringVector;

class ArticulationSpecializer {
protected:
	dmArticulation * artic;	
};

//class FeaturedHumanoid : public HumanoidDataLogger, BalanceDemoStateMachine {};

#endif