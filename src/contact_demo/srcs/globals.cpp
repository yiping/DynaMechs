
//  globals.cpp
//  July 7, 2012
//	YL

#include <dmArticulation.hpp>
#include <dmContactSystem.hpp>
#include "wx/wx.h"
#include <dmIntegEuler.hpp>
#include <dmTime.h>
#include <wxDMGLPolarCamera_zup.hpp>
#include <wxDMGLMouse.hpp>
#include "BasicGLPane.h"
#include "SimulationThread.h"
#include "MainFrame.h"

dmArticulation *G_robot;
dmContactSystem *G_contact;
SimulationThread * simThread;
MainFrame *frame;
wxMutex dataMutex;

