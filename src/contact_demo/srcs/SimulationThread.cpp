//  SimulationThread.cpp
//  July 7, 2012
//  YL


#include "globalVariables.h"
#include "SimulationThread.h"

#include <iostream>
using namespace std;
#include <stdio.h>
#include <dmTime.h>
#include "dmRigidBody.hpp"

SimulationThread::SimulationThread() : wxThread(wxTHREAD_JOINABLE) 
{
	unPauseCondition  = new wxCondition(mutex);
	refreshCondition  = new wxCondition(re_mutex);
	re_mutex.Lock(); // // re_mutex needs to be initially locked
						// so that the Gui thread will not signal the condition 
						// before the simThread start waiting on it!
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
	delete refreshCondition;
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
			//wxMutexLocker lock(mutexProtectSharedData);
			
			dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->setExternalForce(box_ext_f);

			if (frame->logDataCheckBox->IsChecked()) 
			{
				//cout<<"log data!"<<endl;
				logger->logData();
			}
			
			last_control_time = sim_time;
		}
		
		// Simulate

		Float dt = idt;
		G_integrator->simulate(dt);

		sim_time += idt;
		Integ_count++;
		if (Integ_count == 2)
		{
			Integ_count = 0;
			refreshCondition->Wait(); // Wait() atomically unlocks the re_mutex and start waiting
									  // once signaled by Gui(main) thread, it then locks re_mutex again and returns
		}
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
	refreshCondition->Signal(); // unWait ... 
	// waits for a joinable thread to terminate and returns the value
	Wait();
}


