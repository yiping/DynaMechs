/*
 *  CustomTaskSpaceController.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __CUSTOM_TASK_SPACE_CONTROLLER_H__
#define __CUSTOM_TASK_SPACE_CONTROLLER_H__


#include "TaskSpaceController.h"
#include "mosek.h"
#include <Eigen/Core>
#include "dmArticulation.hpp"

typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;
class CustomTaskSpaceController : public TaskSpaceController
{
public:
	CustomTaskSpaceController(dmArticulation * art);
	
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
	
	
	Float norm(const VectorXF & a,const VectorXF & b);
	void logBarrier(const VectorXF & x, Float &b, VectorXF & bs);
	void evalBarrier(const VectorXF & x, bool computeHess, VectorXF & bs, VectorXF& grad, MatrixXF& H, MatrixXF & iH);
	void evalResidual(const VectorXF & x, const VectorXF & v, const MatrixXF & Aeq, const VectorXF & beq, const Float t, bool computeHess, VectorXF & rd, VectorXF & rp, VectorXF & bs, VectorXF& grad, MatrixXF & H, MatrixXF & iH);
	
	
	
	
	MatrixXF LambdaInvTau,LambdaInvF;
	MatrixXF Lttau,Ltf, Lctau,Lcf;
	VectorXF bt, bc;
	VectorXF tauLb,tauUb;
	VectorXF fnetUb;	
	VectorXF barriers, barriersHat;
	vector<VectorXF> grads, gradsHat;
	vector<Eigen::DiagonalMatrix<Float, Dynamic, Dynamic> > Hs, HsHat, iHs;
	VectorXF grad, gradHat;
	VectorXF rdInit;
	
	int numConstraintTasks, numOptimTasks, fStart, eStart, zStart;
	
private:
	int fLen, vLen, xLen, barLen;
	
	VectorXF eBiasCandG;
	LDLT<MatrixXF> Hdecomp;
};

#endif