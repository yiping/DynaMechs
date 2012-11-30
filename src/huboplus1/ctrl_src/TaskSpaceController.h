// TaskSpaceController.h
// Nov 27, 2012
// YL


#ifndef __TASK_SPACE_CONTROLLER_H__
#define __TASK_SPACE_CONTROLLER_H__

#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"
#include "control_defs.h"


class TaskSpaceController 
{
public:
	TaskSpaceController(dmArticulation * art) { artic= art;} ;
	
	virtual void ObtainArticulationData()=0;
	
	virtual void AssignFootMaxLoad(int index, double maxLoad) =0;
	virtual void UpdateObjective() =0;
	virtual void UpdateVariableBounds()=0;	
	virtual void UpdateConstraintMatrix()=0;
	virtual void UpdateInitialConstraintBounds()=0;
	virtual void UpdateHPTConstraintBounds()=0;
	
	virtual void Optimize()=0;

protected:	
	MatrixXF TaskJacobian;
	VectorXF TaskBias;
	VectorXF TaskWeight;
	VectorXF TaskError;
	
	VectorXF taskConstrActive;
	VectorXF taskOptimActive;
	
	VectorXF OptimizationSchedule;
	
	
	XformVector SupportXforms;
	vector<XformVector > PointForceXforms; // force transformation (from contact point to support frame origin) 
	
	IntVector SupportIndices;
	vector<MatrixXF > SupportJacobians;
	
	VectorXF xx;	// optimal solution

	
	int numCon;
	int numVar;

	int iter;
	MSKtask_t     task;
	VectorXF tau, qdd, fs, lambda;
	
	
protected:
	dmArticulation * artic;
	MSKenv_t      env;
	
	MSKrescodee   r;
	
	
	MatrixXF FrictionBasis;
	
};

#endif
