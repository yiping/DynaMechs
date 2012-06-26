/*
 *  SimulationThread.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "SimulationThread.h"
#include "GlobalDefines.h"
#include "humanoidControl.h"

#include <iostream>
using namespace std;
#include <stdio.h>
#include <dmTime.h>

SimulationThread::SimulationThread() : wxThread(wxTHREAD_JOINABLE)
{

}

void *SimulationThread::Entry()
{
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);
	stopRequested = false;
	
	while (!stopRequested) {
		
		// Check if it's time for control
		if ((sim_time - last_control_time) >= cdt) {
			if (sim_time > .2) {
				ControlInfo ci;
				HumanoidControl(ci); 
				//cout << ci.totalTime << "\t" << ci.calcTime << "\t" << ci.setupTime << "\t" << ci.optimTime << "\t" << ci.iter << endl;
			}
			last_control_time = sim_time;
		}
		
		// Simulate
		//lockRobot();
		G_integrator->simulate(idt);
		//unlockRobot();
		sim_time += idt;
		dmGetSysTime(&tv_now);
	}
}

void SimulationThread::requestStop()
{
	stopRequested = true;
}

void SimulationThread::OnExit()
{

}

void SimulationThread::lockRobot() {
	mutex.Lock();
}
void SimulationThread::unlockRobot() {
	mutex.Unlock();	
}

