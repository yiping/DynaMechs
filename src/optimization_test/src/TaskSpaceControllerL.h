//  TaskSpaceControllerL.h
//  Inspired by PMW's TaskSpaceControllerA/B 
//  YL
//  8/14/12
//  This file implements the task space controller described in 
//  "Feature-Based Locomotion Controllers" paper 
//  by Martin de Lasa's, 2010, ACM Transactions on Graphics 

#ifndef __TASK_SPACE_CONTROLLER_L_H__
#define __TASK_SPACE_CONTROLLER_L_H__

#include "control_globals.h"
#include "dmArticulation.hpp"
#include "QPsolver.h"


class TaskSpaceControllerL
{
public:
	TaskSpaceControllerL(dmArticulation * robot);
	~TaskSpaceControllerL();

	void ObtainArticulationData();
	void Reset();
	void UpdateObjective();
	void UpdateVariableBounds();
	void UpdateConstraintMatrix();

	void UpdateHPTConstraintBounds();
	void Optimize();


	// deprecated



	

	


	static const int tauStart, tauEnd, qddStart,qddEnd,fStart,fEnd,lambdaStart,lambdaEnd,dynConstrStart,dynConstrEnd,fConstrStart,fConstrEnd,hptConstrStart;

protected:
	dmArticulation * robot;	


	MatrixXF FrictionBasis;
	XformVector SupportXforms;
	IntVector SupportIndices;
	vector<MatrixXF > SupportJacobians;
	vector<XformVector > PointForceXforms; // force transformation (from contact point to support body coordinate)  

	MatrixXF TaskJacobian;
	VectorXF TaskBias;

	MatrixXF C0;
	VectorXF d0;

	QPsolver solver;

private:
	MatrixXF JTXTV;
	MatrixXF ST
	MatrixXF DynCon;

	MatrixXF Cbar;
	VectorXF dbar;
	
};

#endif
