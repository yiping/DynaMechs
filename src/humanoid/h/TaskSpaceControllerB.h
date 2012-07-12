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

#include "TaskSpaceController.h"
#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"

typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;
class TaskSpaceControllerB : public TaskSpaceController
{
public:
	TaskSpaceControllerB(dmArticulation * art);
	
	// This function 
	void ObtainArticulationData();
	
	void AssignFootMaxLoad(int index, double maxLoad);
	
	
	void UpdateObjective();
	void UpdateTauObjective();
	
	void UpdateVariableBounds();
	
	void UpdateConstraintMatrix();
	void UpdateInitialConstraintBounds();
	void UpdateHPTConstraintBounds();
	
	void Optimize();
	
	~TaskSpaceControllerB();
	
	static const int tauStart, tauEnd, fStart,fEnd,zStart,eStart,eEnd,eConstrStart,eConstrEnd;
};