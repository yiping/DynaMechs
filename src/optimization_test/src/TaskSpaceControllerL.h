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
	void UpdateInitialConstraintBounds();
	void UpdateHPTConstraintBounds();
	void Optimize();


	// deprecated



	

	



protected:
	dmArticulation * robot;	


	MatrixXF FrictionBasis;
	XformVector SupportXforms;
	IntVector SupportIndices;
	vector<MatrixXF > SupportJacobians;
	vector<XformVector > PointForceXforms; // force transformation (from contact point to support body coordinate)  

	MatrixXF TaskJacobian;
	VectorXF TaskBias;



	QPsolver solver;

private:
	MatrixXF JTXTV;
	MatrixXF ST
	MatrixXF DynCon;

	MatrixXF C0;
	VectorXF d0;

	MatrixXF Cbar;
	VectorXF dbar;

	VectorXbk  bk;
	VectorXF   bl;
	VectorXF   bu;

	MatrixXF C;
	
};

#endif
