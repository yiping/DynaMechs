
//  globalVariables.h
//  Nov 26, 2012
//  YL


#ifndef __GLOBAl_VARIABLES__
#define __GLOBAL_VARIABLES__

//This is a dirty workaround
#undef Success

#include <dm.h>
#include <dmArticulation.hpp>

#include "SimulationThread.h"
#include "MainFrame.h"

#include "HuboController.h"
#include "HuboBalanceController.h"
//#define EIGEN_NO_DEBUG





extern dmArticulation *G_robot;
extern MainFrame *frame;
extern SimulationThread * simThread;
extern wxMutex dataMutex;

extern HuboController * huboCtrl;


#endif
