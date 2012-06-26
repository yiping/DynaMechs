/*
 *  GlobalFunctions.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __GLOBAL_FUNCTIONS__
#define __GLOBAL_FUNCTIONS__
#include <dm.h>
#include <dmTime.h>

void ComputeComInfo(Matrix6XF & Cmm, Vector6F & bias, Vector3F & pCom, Float & m);
void matrixExpOmegaCross(const Vector3F & omega,Matrix3F & R);
void matrixLogRot(const Matrix3F & R, Vector3F & omega);
void computeAccBiasFromFwKin(dmRNEAStruct & infoStruct,Vector6F & a);

void copyRtoMat(const CartesianTensor R, Matrix3F & Rmat);

void transformToZMP(Vector6F & fZMP, Vector3F & pZMP) ;
double timeDiff(const dmTimespec & t1, const dmTimespec & t2);



#endif