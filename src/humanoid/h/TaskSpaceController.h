/*
 *  TaskSpaceController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 7/11/12.
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
#ifndef __TASK_SPACE_CONTROLLER_H__
#define __TASK_SPACE_CONTROLLER_H__

#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"
#include "GlobalTypes.h"


class TaskSpaceController : public virtual ArticulationSpecializer
{
public:
	TaskSpaceController(dmArticulation * art) { artic= art;} ;
	
	// This function 
	virtual void ObtainArticulationData()=0;
	
	virtual void AssignFootMaxLoad(int index, double maxLoad) =0;
	virtual void UpdateObjective() =0;
	virtual void UpdateTauObjective()=0;
	virtual void UpdateVariableBounds()=0;	
	virtual void UpdateConstraintMatrix()=0;
	virtual void UpdateInitialConstraintBounds()=0;
	virtual void UpdateHPTConstraintBounds()=0;
	
	virtual void Optimize()=0;
	
	MatrixXF TaskJacobian;
	VectorXF TaskBias;
	VectorXF TaskWeight;
	VectorXF TaskError;
	
	VectorXF taskConstrActive;
	VectorXF taskOptimActive;
	
	VectorXF OptimizationSchedule;
	
	
	XformVector SupportXforms;
	vector<XformVector > PointForceXforms;
	
	IntVector SupportIndices;
	
	VectorXF xx;
	vector<MatrixXF > SupportJacobians;
	
	int numCon;
	int iter;
	MSKtask_t     task;
	VectorXF tau, qdd, fs, lambda;
	
	
protected:
	
	MSKenv_t      env;
	
	MSKrescodee   r;
	
	
	MatrixXF FrictionBasis;
	
};

#endif