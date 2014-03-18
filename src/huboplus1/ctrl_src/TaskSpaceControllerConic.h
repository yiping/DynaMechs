/*
 * TaskSpaceControllerConic.h
 *
 *  Created on: Mar 9, 2014
 *      Author: yiping
 *      task-Space controller with
 *      conic optimization
 *      Reference:
 *      P. M. Wensing: Generation of Dynamic Humanoid Behaviors Through Task-Space
 *                     Control with Conic Optimization
 *
 */

#ifndef TASKSPACECONTROLLERCONIC_H_
#define TASKSPACECONTROLLERCONIC_H_

#include "mosek.h"
#include "dmArticulation.hpp"
#include "TaskSpaceController.h"


class TaskSpaceControllerConic : public TaskSpaceController
{
public:
	TaskSpaceControllerConic(dmArticulation * artic);
	~TaskSpaceControllerConic();

	void ObtainArticulationData();
	void AssignFootMaxLoad(int index, double maxLoad);
	void UpdateObjective();


	void UpdateVariableBounds();

	void UpdateConstraintMatrix();
	void UpdateInitialConstraintBounds();
	void UpdateHPTConstraintBounds(); // Higher Priority Tasks Constraints

	void Optimize();



	static const int tauStart, tauEnd, fStart,fEnd, zStart,eStart,eEnd,eConstrStart,eConstrEnd,fNormConstrStart,fNormConstrEnd;

	MatrixXF LambdaInvTau,LambdaInvF;
	Float minfz;

private:
	LDLT<MatrixXF> Hdecomp;  // LDL decomposition of H. H is positive definite, using LDLT is faster and relatively accurate
	VectorXF eBiasCandG; // b
};
#endif /* TASKSPACECONTROLLERCONIC_H_ */
