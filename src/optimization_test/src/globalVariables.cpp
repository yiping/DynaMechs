
//  globalVariables.cpp
//  Sep 3, 2012
//  Project: Optimization Test
//	YL


#include "globalVariables.h"


dmArticulation *G_robot;

SimulationThread * simThread;
MainFrame *frame;

wxMutex dataMutex;
//OTDataLogger *logger;

StateMachineControllerA * humanoidCtrl;
