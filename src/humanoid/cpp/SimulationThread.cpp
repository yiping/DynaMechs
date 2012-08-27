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
	dmTimespec tv_now, last_control_tv;
	dmGetSysTime(&tv_now);
	dmGetSysTime(&last_control_tv);
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
			
			Float slowFactor = 2;
			if (frame->slowMotion->IsChecked()) {
				slowFactor = pow(10,frame->slowMoRatio->getValue());
			}
			
			
			dmGetSysTime(&tv_now);
			Float realTimeDiff = timeDiff(last_control_tv, tv_now);
			Float sleepTime = slowFactor*cdt - realTimeDiff;
			
			//if (sleepTime > 0) {
			//	wxMicroSleep((unsigned long) (sleepTime * 1e6));
			//}
			
			//cout << "Control" << endl;
			dmGetSysTime(&last_control_tv);
			last_control_time = sim_time;
		}
		// Simulate
		//lockRobot();
		Float dt = idt;
		
		//Float qNow[31],qdNow[31];
		//G_robot->getState(qNow, qdNow);
		//cout << setprecision(6);
		//cout << "Sim time " << sim_time << "\t\t" << qNow[25] << "," << qNow[30] 
		//			<< " / " << qdNow[25] << "," << qdNow[30] << endl;
		
		/*humanoid->ControlInit();
		humanoid->ComputeActualQdd(humanoid->qddA);
		humanoid->tau.setZero(26);
		humanoid->qdd.setZero(26);
		humanoid->fs.setZero(12);/*
		
		
		/*if (sim_time > .16) {
			humanoid->logData();
		}
		
		
		
		if (sim_time >= .1606) {
			paused_flag = true;
		}*/
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

