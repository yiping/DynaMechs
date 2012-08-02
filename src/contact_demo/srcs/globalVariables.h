
//  globalVariables.h
//  July 7, 2012
//  YL


#ifndef __GLOBAl_VARIABLES__
#define __GLOBAL_VARIABLES__

//This is a dirty workaround
#undef Success

#include <dm.h>
#include <dmArticulation.hpp>
#include <dmContactSystem.hpp>

#include "SimulationThread.h"
#include "ContactMainFrame.h"
#include "ContactDemoDataLogger.h"

//#define EIGEN_NO_DEBUG
//#define OPTIM_DEBUG
//#define CONTROL_DEBUG




extern dmArticulation *G_robot;
extern dmContactSystem *G_contact;
extern MainFrame *frame;
extern SimulationThread * simThread;
extern ContactDemoDataLogger * logger;

//extern wxMutex dataMutex;
#define GROUP_FLAG 0x0800

//#define USE_DYNAMIC_CONTACT_MODEL


#endif
