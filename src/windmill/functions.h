#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H
#include "global_typedef.h"

Vector6F solveInverse(const Matrix6F & , const Vector6F & );
VectorXF solveJacobianPseudoInverse(const Matrix6XF & , const Vector6F & );
void simDataOutput(const DataRecVector & );

#endif
