//  SimulationThread.cpp
//  July 7, 2012
//  YL


#include "globals.h"
#include "SimulationThread.h"

#include <iostream>
using namespace std;
#include <stdio.h>
#include <dmTime.h>
#include "dmRigidBody.hpp"

SimulationThread::SimulationThread() : wxThread(wxTHREAD_JOINABLE) 
{
	unPauseCondition  = new wxCondition(mutex);
	G_integrator = new dmIntegEuler();
	paused_flag = true;
	mutexProtectSharedData.Unlock();
	for (unsigned int i=0; i<6;i++)
	{
		box_ext_f[i] = 0;
	}
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
	
	while (!stopRequested) 
	{
		if (paused_flag) {
			mutex.Lock();
			unPauseCondition->Wait(); // hold until unPause is signaled (broadcasted)
			mutex.Unlock();
		}
	
		// Check if it's time for control
		if ((sim_time - last_control_time) >= cdt) 
		{
			wxMutexLocker lock(mutexProtectSharedData);
			
			dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->setExternalForce(box_ext_f);

			
			
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

	// waits for a joinable thread to terminate and returns the value
	Wait();
}

