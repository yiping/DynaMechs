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

typedef struct GRFInfoStruct {
	Vector3F pZMP;
	Vector3F fZMP;
	Float nZMP;
	
	int localContacts;
	vector<Vector3F > pCoPs;
	vector<Vector3F > fCoPs;
	vector<Float > nCoPs;
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


#endif