/*
 *  GlobalDefines.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __GLOBAL_DEFINES__
#define __GLOBAL_DEFINES__

#include <dm.h>
#include <dmArticulation.hpp>
#include <vector>

#include "GlobalTypes.h"

//#define EIGEN_NO_DEBUG
//#define OPTIM_DEBUG
//#define CONTROL_DEBUG

extern dmArticulation *G_robot;
extern volatile Float sim_time;
extern volatile Float ComPos[3];
extern volatile Float ComDes[3];
extern GRFInfo grfInfo;



enum DataItems {
	QUAT0,
	QUAT1,
	QUAT2,
	QUAT3,
	P1,
	P2,
	P3,
	RHIP_PHI,
	RHIP_PSI,
	RHIP_GAMMA,
	RKNEE,
	RANK1,
	RANK2,
	LHIP_PHI,
	LHIP_PSI,
	LHIP_GAMMA,
	LKNEE,
	LANK1,
	LANK2,
	RSHOULD_PHI,
	RSHOULD_PSI,
	RSHOULD_GAMMA,
	RELBOW,
	LSHOULD_PHI,
	LSHOULD_PSI,
	LSHOULD_GAMMA,
	LELBOW,
	//
	BASE_OMEGA_X,
	BASE_OMEGA_Y,
	BASE_OMEGA_Z,
	/*QUAT3,
	P1,
	P2,
	P3,
	RHIP_PHI,
	RHIP_PSI,
	RHIP_GAMMA,
	RKNEE,
	RANK1,
	RANK2,
	LHIP_PHI,
	LHIP_PSI,
	LHIP_GAMMA,
	LKNEE,
	LANK1,
	LANK2,
	RSHOULD_PHI,
	RSHOULD_PSI,
	RSHOULD_GAMMA,
	RELBOW,
	LSHOULD_PHI,
	LSHOULD_PSI,
	LSHOULD_GAMMA,
	LELBOW,*/
	MAX_NUM_ITEMS
	
};

enum DataGroups {
	JOINT_ANGLES,
	JOINT_RATES,
	JOINT_TORQUES,
	COM_LOCATION,
	COM_VELOCITY,
	CENTROIDAL_MOMENTUM,
	MAX_NUM_GROUPS
};


#endif