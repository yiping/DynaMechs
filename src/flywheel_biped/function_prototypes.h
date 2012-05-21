/*
 *****************************************************************************
 *     File: function_prototypes.cpp
 *   Author: Yiping Liu
 *  Created: 17 Apr 2012
 *  Summary: collection of function prototypes
 *****************************************************************************/
#include "global.h"   

// ----------------------------------------------------------------------

void myinit (void);
void display (void);
void myReshape(int , int );
void processKeyboard(unsigned char , int, int);
void processSpecialKeys(int , int, int);

// ----------------------------------------------------------------------

void SaveToDataRecord(DataRecord *Rec);
void simDataOutput(const DataRecVector & );
void readTorsoPoseSetpoints();

// ----------------------------------------------------------------------

Vector6F solveInverse(const Matrix6F & , const Vector6F & );
VectorXF solveJacobianPseudoInverse(const Matrix6XF & , const Vector6F & );

// ----------------------------------------------------------------------

void calculateActualZMP(void);
void adjustBipedLegConfig(Float , Float , Float , Float);
void applyTorsoDisturbance(Float , Float , Vector3F , Vector3F );
void computeBipedRightLegDesiredQdd();
void computeBipedLeftLegDesiredQdd();
Vector3F popTorsoPositionSetPoint();
Matrix3F popTorsoOrientationSetPoint();
Vector6F resolveTorsoAcceleration(void);


