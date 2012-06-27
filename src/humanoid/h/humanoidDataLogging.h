/*
 *  humanoidDataLogging.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HUMANOID_DATA_LOGGING_H__
#define __HUMANOID_DATA_LOGGING_H__

#include "GlobalDefines.h"
#include "DataLogger.h"


extern int COM_POSITION, COM_VELOCITY, COM_POSITION_DES, CENTROIDAL_MOMENTUM, QDD_OPT, HMAT, HDOT_DES, HDOT_OPT;


void logData();
void initializeDataLogging();
void saveData();

extern DataLogger dataLog;


#endif