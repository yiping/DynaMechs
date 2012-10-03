// PrioritizedController.h
 

#ifndef __PRIORITIZED_CONTROLLER_H__
#define __PRIORITIZED_CONTROLLER_H__

#include "control_globals.h"
#include "QPsolver.h"
#include "math_funcs.h"


class PrioritizedController
{
public:
	PrioritizedController();
	virtual ~PrioritizedController();

	virtual void Reset()=0;
	virtual void UpdateObjective(const MatrixXF &TaskJacobian, const VectorXF &TaskBias);
	virtual void UpdateVariableBounds();
	virtual void SetInitialConstraintBounds()=0;
	virtual void SetInitialVariableBounds()=0;
	virtual void UpdateConstraintMatrix();
	virtual void UpdateConstraintBounds();

	void OptimizeSingleTier();

	VectorXF dBar;
	MatXFVector TaskJacobians;
	VecXFVector TaskBiases;
	VectorXi TaskSchedule;
	QPsolver solver;

protected:

	bool JtBarFullColRank;
	bool isFeasible;



	MatrixXF JTXTV;

	MatrixXF CBar;
	

	VectorXbk  c_bk;
	VectorXF   c_bl;
	VectorXF   c_bu;

	VectorXbk  v_bk;
	VectorXF   v_bl;
	VectorXF   v_bu;

	MatrixXF C;

	MatrixXF D0; 


	
};

#endif
