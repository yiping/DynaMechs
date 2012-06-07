/*
 *  TaskSpaceController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"

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


typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;

class TaskSpaceController
{
public:
	TaskSpaceController(dmArticulation * art);
	
	// This function initilizes the entire optimization problem.
	void InitializeProblem();
	
	// This function 
	void ObtainArticulationData();
	
	
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
	
	
	XformVector SupportXforms;
	IntVector SupportIndices;
	
	VectorXF xx;
	vector<MatrixXF > SupportJacobians;
	
private:
	
	dmArticulation * artic;
	
	MSKenv_t      env;
	MSKtask_t     task;
	MSKrescodee   r;
	int numCon;
	
	MatrixXF FrictionBasis;
	
};