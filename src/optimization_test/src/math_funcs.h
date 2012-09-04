//  math_funcs.h
//  Aug 23, 2012
//  YL

#ifndef __MATH_FUNCS_H__
#define __MATH_FUNCS_H__

#include <dm.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>



using namespace std;
using namespace Eigen;

void getNullSpace(const MatrixXF &A, MatrixXF &C);
void showDefiniteness(const MatrixXF &A );


#endif
