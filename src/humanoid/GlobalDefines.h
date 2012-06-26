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

//This is a dirty workaround
#undef Success

#include <dm.h>
#include <dmArticulation.hpp>
#include <vector>
#include "wx/wx.h"

#include "GlobalTypes.h"
#include <dmIntegEuler.hpp>
#include <dmTime.h>
#include <wxDMGLMouse.hpp>
#include <wxDMGLPolarCamera_zup.hpp>
#include "BasicGLPane.h"

//#define EIGEN_NO_DEBUG
//#define OPTIM_DEBUG
//#define CONTROL_DEBUG

extern dmArticulation *G_robot;
extern volatile Float sim_time;
extern volatile Float ComPos[3];
extern volatile Float ComDes[3];
extern GRFInfo grfInfo;
extern wxCheckBox * showCoM, * showGRF, * showNetForceAtGround, * showNetForceAtCoM;
extern wxStaticText * realTimeRatioDisplay;
extern BasicGLPane * glPane;

extern wxDMGLMouse *mouse;
extern wxDMGLPolarCamera_zup *camera;

extern Float idt, cdt, last_control_time, last_render_time, real_time_ratio;
extern Float rtime;
extern dmTimespec last_draw_tv;
extern dmIntegEuler *G_integrator;

extern bool IsWireframe;
extern bool paused_flag;

#define GROUP_FLAG 0x0800



enum DataItems {
	TIME,
	// POSITIONS
	BASE_QUAT0,
	BASE_QUAT1,
	BASE_QUAT2,
	BASE_QUAT3,
	BASE_P_X,
	BASE_P_Y,
	BASE_P_Z,
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
	// RATES
	BASE_OMEGA_X,
	BASE_OMEGA_Y,
	BASE_OMEGA_Z,
	BASE_V_X,
	BASE_V_Y,
	BASE_V_Z,
	RHIP_OMEGA_X,
	RHIP_OMEGA_Y,
	RHIP_OMEGA_Z,
	RKNEE_RATE,
	RANK1_RATE,
	RANK2_RATE,
	LHIP_OMEGA_X,
	LHIP_OMEGA_Y,
	LHIP_OMEGA_Z,
	LKNEE_RATE,
	LANK1_RATE,
	LANK2_RATE,
	RSHOULD_OMEGA_X,
	RSHOULD_OMEGA_Y,
	RSHOULD_OMEGA_Z,
	RELBOW_RATE,
	LSHOULD_OMEGA_X,
	LSHOULD_OMEGA_Y,
	LSHOULD_OMEGA_Z,
	LELBOW_RATE,
	// TORQUES
	RHIP_TAU_X,
	RHIP_TAU_Y,
	RHIP_TAU_Z,
	RKNEE_TAU,
	RANK1_TAU,
	RANK2_TAU,
	LHIP_TAU_X,
	LHIP_TAU_Y,
	LHIP_TAU_Z,
	LKNEE_TAU,
	LANK1_TAU,
	LANK2_TAU,
	RSHOULD_TAU_X,
	RSHOULD_TAU_Y,
	RSHOULD_TAU_Z,
	RELBOW_TAU,
	LSHOULD_TAU_X,
	LSHOULD_TAU_Y,
	LSHOULD_TAU_Z,
	LELBOW_TAU,
	// Centroial quantities
/*	COM_P_X,
	COM_P_Y,
	COM_P_Z,
	COM_V_X,
	COM_V_Y,
	COM_V_Z,
	CM_K_X,
	CM_K_Y,
	CM_K_Z,
	CM_L_X,
	CM_L_Y,
	CM_L_Z,*/
	// GRF Quantities
	LCOP_F_X,
	LCOP_F_Y,
	LCOP_F_Z,
	LCOP_P_X,
	LCOP_P_Y,
	LCOP_N_Z,
	RCOP_F_X,
	RCOP_F_Y,
	RCOP_F_Z,
	RCOP_P_X,
	RCOP_P_Y,
	RCOP_N_Z,
	ZMP_F_X,
	ZMP_F_Y,
	ZMP_F_Z,
	ZMP_P_X,
	ZMP_P_Y,
	ZMP_N_Z,
	MAX_STATIC_ITEMS
};

enum DataGroups {
	JOINT_ANGLES,
	JOINT_RATES,
	JOINT_TORQUES,
	LEFT_FOOT_WRENCH,
	//LEFT_FOOT_COP_FORCE,
	//LEFT_FOOT_COP_LOC,
	RIGHT_FOOT_WRENCH,
	//RIGHT_FOOT_COP_FORCE,
	//RIGHT_FOOT_COP_POS,
	ZMP_WRENCH,
	//ZMP_FORCE,
	//ZMP_POS,
	MAX_STATIC_GROUPS
};


#include "GlobalFunctions.h"

#endif
