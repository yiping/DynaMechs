/*
 *  SimulationThread.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"
#include "SimulationThread.h"
#include "HumanoidController.h"

#include <iostream>
using namespace std;
#include <stdio.h>
#include <dmTime.h>

SimulationThread::SimulationThread() : wxThread(wxTHREAD_JOINABLE) 
{
	unPauseCondition  = new wxCondition(mutex);
	G_integrator = new dmIntegEuler();
	paused_flag = true;
	sim_time = 0;
}

SimulationThread::~SimulationThread()
{
	delete unPauseCondition;
}

void *SimulationThread::Entry()
{
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);
	stopRequested = false;
	
	while (!stopRequested) {
		if (paused_flag) {
			cout << "Sleeping for pause" << endl;
			mutex.Lock();
			unPauseCondition->Wait();
			mutex.Unlock();
			cout << "Awake now" << endl;
		}
		// Check if it's time for control
		if ((sim_time - last_control_time) >= cdt) {
			
			ControlInfo ci;
			humanoid->StateControl(ci);
			//HumanoidControl(ci); 
			//cout << ci.totalTime << "\t" << ci.calcTime << "\t" << ci.setupTime << "\t" << ci.optimTime << "\t" << ci.iter << endl;
			
			last_control_time = sim_time;
		}
		// Simulate
		//lockRobot();
		Float dt = idt;
		G_integrator->simulate(dt);
		//unlockRobot();
		sim_time += idt;
		dmGetSysTime(&tv_now);
	}
	return NULL;
}
void SimulationThread::unPause()
{
	unPauseCondition->Broadcast();
}
void SimulationThread::requestStop()
{
	stopRequested = true;
	unPause();
	Wait();
}

void SimulationThread::lockRobot() {
	mutex.Lock();
}
void SimulationThread::unlockRobot() {
	mutex.Unlock();	
}

