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

typedef vector<int> IntVector;
typedef vector<Matrix6F> XformVector;

class TaskSpaceController
{
public:
	TaskSpaceController(dmArticulation * art, IntVector & suppIndices, XformVector & suppXforms);
	
	// This function initilizes the entire optimization problem.
	void InitializeProblem();
	
	// This function 
	void ObtainArticulationData();
	
	
	void UpdateObjective();
	void UpdateVariableBounds();
	
	void UpdateConstraintMatrix();
	void UpdateConstraintBounds();
	
	void Optimize();
	
	~TaskSpaceController();
private:
	
	dmArticulation * artic;
	
	MSKenv_t      env;
	MSKtask_t     task;
	MSKrescodee   r;
	int numCon;
	
	MatrixXF TaskJacobian;
	MatrixXF ConstraintJacobian;
	VectorXF TaskBias;
	VectorXF ConstraintBias;
	
	MatrixXF FrictionBasis;
	
	vector<MatrixXF > SupportJacobians;
	XformVector SupportXforms;
	IntVector SupportIndices;
};