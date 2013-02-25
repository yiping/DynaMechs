/*
 *  CoordinatedCublicSpline.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 2/20/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */

#include "CoordinatedCubicSpline.h"

#include <Eigen/LU>


CoordinatedCubicSpline::CoordinatedCubicSpline()
{
}

void CoordinatedCubicSpline::computeCoefficients(const VectorXF & times, const MatrixXF & pts, const VectorXF & v0, const VectorXF & vf)
{
	ts = times;
	const int numSplines = times.size()-1;
	const int numPts     = numSplines+1;
	const int dim        = pts.rows();
	
	
	MatrixXF A(4*numSplines,4*numSplines);
	A.setZero();
	MatrixXF b(4*numSplines,dim);
	b.setZero();
	
	const int initRow = 4*numSplines-2;
	const int finalRow = 4*numSplines-1;
	
	int row = 0;
	for (int i=0; i<numSplines; i++) {
		const Float t0 = times(i);
		const Float t02 = t0*t0;
		const Float t03 = t0*t02;
		
		const Float tf = times(i+1);
		const Float tf2 = tf*tf;
		const Float tf3 = tf2*tf;
	
		const int varStart = 4*i;
		
		// Position Constraints at the strat and end of each spline
		A(row  ,varStart) = t03;	A(row  ,varStart+1) = t02;	A(row  ,varStart+2) = t0;	A(row  ,varStart+3) = 1;
		A(row+1,varStart) = tf3;	A(row+1,varStart+1) = tf2;	A(row+1,varStart+2) = tf;	A(row+1,varStart+3) = 1;
		
		int matchRow = row+2*numSplines-2; 
		if (i!=0) {
			// Velocity/acc match of the beginning of this spline to the end of the previous
			A(matchRow  ,varStart) = -3*t02;	A(matchRow  ,varStart+1) = -2*t0;	A(matchRow  ,varStart+2) = -1; 
			A(matchRow+1,varStart) = -6*t0;		A(matchRow+1,varStart+1) = -2;
		}
		else {
			// Initial Velocity constraint
			A(initRow  ,varStart) = 3*t02;	A(initRow  ,varStart+1) = 2*t0;	A(initRow  ,varStart+2) = 1;
		}

		matchRow+=2;
		
		if (i!=(numSplines-1)) {
			// Velocity/acc match of the end of this spline to the beginning of the next
			A(matchRow  ,varStart) = 3*tf2;		A(matchRow  ,varStart+1) = 2*tf;	A(matchRow  ,varStart+2) = 1; 
			A(matchRow+1,varStart) = 6*tf;		A(matchRow+1,varStart+1) = 2;
		}
		else {
			// Final Velocity Constraint
			A(finalRow  ,varStart) = 3*tf2;	A(finalRow  ,varStart+1) = 2*tf;	A(finalRow  ,varStart+2) = 1;
		}
		
		row+=2;
	}
	//cout << "A" << endl << A << endl;
	
	
	FullPivLU<MatrixXF> luDecomp(A);
	
	coefficients.resize(numSplines);
	for (int i=0; i<numSplines; i++) {
		coefficients[i].setZero(dim,4);
	}
	
	for (int j=0; j<dim ;j++)
	{
		row = 0;
		for (int i=0; i<numSplines; i++) {
			b(row++,j)=pts(j,i);
			b(row++,j)=pts(j,i+1);
		}
		b(initRow,j) = v0(j);
		b(finalRow,j) = vf(j);
	}
	
	//cout << "b" << endl << b << endl;
	
	
	MatrixXF x = luDecomp.solve(b);
	for (int i=0 ; i<numSplines ; i++) {
		for (int j=0; j<dim; j++) {
			// Extract coefficients from solved system
			coefficients[i].row(j) = x.block(4*i,j,4,1).transpose(); 
		}
	}
}
void CoordinatedCubicSpline::eval(const Float t, VectorXF& pos, VectorXF&vel, VectorXF&acc) {
	int coefficientIndex = 0;
	const int numSplines  = ts.size()-1;
	const int dim = coefficients[0].rows();
	
	const Float t2 = t*t;
	const Float t3 = t2*t;
	
	for (int i=0; i<numSplines; i++) {
		if(t>ts(i)) {
			coefficientIndex = i;
		}
	}
	
	MatrixXF results(dim,3);
	MatrixXF timeMat(4,3);
	timeMat <<   t3, 3*t2, 6*t,
	             t2, 2*t, 2,
				 t , 1,    0,
	             1 , 0, 0;
	
	results = coefficients[coefficientIndex]*timeMat;
	
	pos = results.col(0);
	vel = results.col(1);
	acc = results.col(2);
}