/*
 *  TaskSpaceController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef __TASK_SPACE_CONTROLLERA_H__
#define __TASK_SPACE_CONTROLLERA_H__

#include "TaskSpaceController.h"


class TaskSpaceControllerA : public TaskSpaceController
{
public:
	TaskSpaceControllerA(dmArticulation * art);
	
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
	
	~TaskSpaceControllerA();

	static const int tauStart, tauEnd, qddStart,qddEnd,fStart,fEnd,lambdaStart,lambdaEnd,dynConstrStart,dynConstrEnd,fConstrStart,fConstrEnd,hptConstrStart;
};

#endif