/*
 *  CoordinatedCublicSpline.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 2/20/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __COORDINATED_CUBIC_SPLINE_H__
#define __COORDINATED_CUBIC_SPLINE_H__

#include "dm.h"

class CoordinatedCubicSpline {
	public:
	CoordinatedCubicSpline();
	void computeCoefficients(const VectorXF & times, const MatrixXF & pts, const VectorXF & v0, const VectorXF & vf);
	void eval(const Float t, VectorXF& pos, VectorXF&vel, VectorXF&acc);
	
	private:
	vector<MatrixXF> coefficients;
	VectorXF ts;
};




#endif