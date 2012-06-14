/*
 *  TaskSpaceController2.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

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

#define NUMTASKS 26
const int tauStart    = 0;
const int tauEnd      = NJ-1;
const int fStart      = tauEnd+1;
const int fEnd        = fStart+ (3*NS*NP-1);
const int fzStart     = fEnd+1;
const int fzEnd       = fzStart + (NS*NP-1);
const int zStart      = fzEnd + 1;
const int eStart      = zStart + 1;

const int eConstrStart = 0;
const int eConstrEnd   = eConstrStart + NUMTASKS;

const int hptConstrStart = fConstrEnd +1;


typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;

class TaskSpaceController2
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
	VectorXF TaskWeight;
	
	XformVector SupportXforms;
	IntVector SupportIndices;
	
	VectorXF xx;
	vector<MatrixXF > SupportJacobians;
	
	int iter;
	
private:
	
	dmArticulation * artic;
	
	MSKenv_t      env;
	MSKtask_t     task;
	MSKrescodee   r;
	int numCon;
	
	MatrixXF FrictionBasis;
	
};