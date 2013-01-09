/*
 *  GlobalDefines.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "dmArticulation.hpp"
#include "GlobalTypes.h"
#include "wx/wx.h"
#include <dmIntegEuler.hpp>
#include <dmTime.h>
#include <wxDMGLPolarCamera_zup.hpp>
#include <wxDMGLMouse.hpp>
#include "BasicGLPane.h"
#include "SimulationThread.h"
#include "HumanoidMainFrame.h"
#include "HumanoidDataLogger.h"
#include "dmEnvironment.hpp"

dmArticulation *G_robot;
dmArticulation *ball;
dmEnvironment *environment;
volatile Float ComPos[3];
volatile Float ComDes[3];



SimulationThread * simThread;
HumanoidMainFrame *frame;
wxMutex dataMutex;

HumanoidDataLogger * humanoid;