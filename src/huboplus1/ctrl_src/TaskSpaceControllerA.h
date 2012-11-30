// TaskSpaceControllerA.h
// Nov 27, 2012
// YL

#ifndef __TASK_SPACE_CONTROLLERA_H__
#define __TASK_SPACE_CONTROLLERA_H__

#include "TaskSpaceController.h"


class TaskSpaceControllerA : public TaskSpaceController
{
public:
	TaskSpaceControllerA(dmArticulation * art);
	~TaskSpaceControllerA();	
	void ObtainArticulationData();
	
	void AssignFootMaxLoad(int index, double maxLoad);
	
	
	void UpdateObjective();
	
	void UpdateVariableBounds();
	
	void UpdateConstraintMatrix();
	void UpdateInitialConstraintBounds();
	void UpdateHPTConstraintBounds();
	
	void Optimize();
	


	static const int tauStart, tauEnd, qddStart,qddEnd,fStart,fEnd,lambdaStart,lambdaEnd,dynConstrStart,dynConstrEnd,fConstrStart,fConstrEnd,hptConstrStart;
};

#endif
