/*
 *****************************************************************************
 *     File: functions_B.cpp
 *   Author: Yiping Liu
 *  Created: 18 Apr 2012
 *  Summary: Math functions (built on Eigen, usually take a long time to compile)  
 *            
 *****************************************************************************/


#include <dm.h>


// A is 6 by 6 square matrix
Vector6F solveInverse(const Matrix6F & A, const Vector6F & b)
{
	Vector6F x;
	x = A.partialPivLu().solve( b );
	return x;
}

VectorXF solveJacobianPseudoInverse(const Matrix6XF & J, const Vector6F & rhs)
{
	VectorXF x;
	x = J.jacobiSvd(ComputeThinU | ComputeThinV).solve(rhs);
	return x;
}

