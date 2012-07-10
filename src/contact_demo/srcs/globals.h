
//  globals.h
//  July 7, 2012
//  YL


#ifndef __GLOBAlS__
#define __GLOBALS__

//This is a dirty workaround
#undef Success

#include <dm.h>
#include <dmArticulation.hpp>
#include <dmContactSystem.hpp>
#include <vector>
#include "wx/wx.h"

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



extern dmArticulation *G_robot;
extern dmContactSystem *G_contact;
extern MainFrame *frame;
extern SimulationThread * simThread;


extern wxMutex dataMutex;
#define GROUP_FLAG 0x0800




#endif
