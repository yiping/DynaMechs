/*
 *  TaskSpaceController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef __TASK_SPACE_CONTROLLER_H__
#define __TASK_SPACE_CONTROLLER_H__

#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"
#include "GlobalTypes.h"
#define NJ  20
#define NF  4
#define NS  2
#define NP  4
#define MU .5

const int tauStart    = 0;
const int tauEnd      = NJ-1;
const int qddStart    = NJ;
const int qddEnd      = 2*NJ+5;
const int fStart      = 2*NJ+6;
const int fEnd        = 2*NJ+5 + 6*NS;
const int lambdaStart = 2*NJ+6 + 6*NS;
const int lambdaEnd   = 2*NJ+5 + 6*NS + NS*NP*NF;

const int dynConstrStart = 0;
const int dynConstrEnd   = NJ+5;
const int fConstrStart   = NJ+6;
const int fConstrEnd     = NJ+5+6*NS;
const int hptConstrStart = fConstrEnd +1;




class TaskSpaceController : public virtual ArticulationSpecializer
{
public:
	TaskSpaceController(dmArticulation * art);
	
	// This function initilizes the entire optimization problem.
	//void InitializeProblem();
	
	// This function 
	void ObtainArticulationData();
	
	void AssignFootMaxLoad(int index, double maxLoad);
	
	
	void UpdateObjective();
	void UpdateTauObjective();
	
	void UpdateVariableBounds();
	
	void UpdateConstraintMatrix();
	void UpdateConstraintBounds();
	
	void Optimize();
	
	~TaskSpaceController();
	MatrixXF ConstraintJacobian;
	MatrixXF TaskJacobian;
	
	VectorXF TaskBias;
	VectorXF ConstraintBias;
	VectorXF TaskWeight;
	
	XformVector SupportXforms;
	IntVector SupportIndices;
	
	VectorXF xx;
	vector<MatrixXF > SupportJacobians;
	
	int numCon;
	int iter;
	MSKtask_t     task;
	
private:
	
	MSKenv_t      env;
	
	MSKrescodee   r;
	
	
	MatrixXF FrictionBasis;
	
};

#endif