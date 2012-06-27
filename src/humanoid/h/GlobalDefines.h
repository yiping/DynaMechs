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
#include "SimulationThread.h"
#include "MainFrame.h"

//#define EIGEN_NO_DEBUG
//#define OPTIM_DEBUG
//#define CONTROL_DEBUG


extern volatile Float ComPos[3];
extern volatile Float ComDes[3];

extern GRFInfo grfInfo;
extern dmArticulation *G_robot;
extern MainFrame *frame;
extern SimulationThread * simThread;

extern string dataSaveDirectory;

extern wxMutex dataMutex;
#define GROUP_FLAG 0x0800






#include "GlobalFunctions.h"

#endif
