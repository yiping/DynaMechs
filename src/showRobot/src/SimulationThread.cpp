//  SimulationThread.cpp
//  Nov 12, 2012
//	Project: showRobot
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
#ifdef SYNC_GRAPHICS
	refreshCondition  = new wxCondition(re_mutex);
	re_mutex.Lock(); // // re_mutex needs to be initially locked
						// so that the Gui thread will not signal the condition 
						// before the simThread start waiting on it!
#endif
	G_integrator = new dmIntegEuler();
	paused_flag = true;
	//mutexProtectSharedData.Unlock();

}

SimulationThread::~SimulationThread()
{
	cout<<"SimThread destructor "<<endl;
	delete unPauseCondition;
#ifdef SYNC_GRAPHICS
	delete refreshCondition;
#endif
	delete G_integrator;

}

void *SimulationThread::Entry()
{
	//dmTimespec tv_now, last_control_tv;
	//dmGetSysTime(&tv_now);
	//dmGetSysTime(&last_control_tv);

	stopRequested = false;
	
	while (!stopRequested) 
	{
		if (paused_flag) 
		{
			cout<<"Sleeping for pause"<<endl;
			mutex.Lock();
			unPauseCondition->Wait(); // hold until unPause is signaled (broadcasted)
			mutex.Unlock();
			cout<<"Awake now"<<endl;
		}


		// Check if it's time for control
		if ((sim_time - last_control_time) >= cdt) 
		{
			//wxMutexLocker lock(mutexProtectSharedData);
			
			// control for your robot


			
			last_control_time = sim_time;
		}
		
		// Simulate

		Float dt = idt;
		G_integrator->simulate(dt);

		sim_time += idt;

		// see whether to synchronize simulation with graphic updates
#ifdef SYNC_GRAPHICS
		//if (frame-> syncGraphicsCheckBox->IsChecked())
		//{
			Integ_count++;
			if (Integ_count == 30)
			{
				Integ_count = 0;
				refreshCondition->Wait(); // Wait() atomically unlocks the re_mutex and start waiting
										  // once signaled by Gui(main) thread, it then locks re_mutex again and returns
			}
		//}
#endif

		//dmGetSysTime(&tv_now);
	}
	return NULL;
}


void SimulationThread::unPause()
{
	unPauseCondition->Broadcast();
}

void SimulationThread::requestStop()	// Note: this function is to be called by main thread
{
	cout<<"SimThread stop requested ..."<<endl;
	stopRequested = true;
	unPause();
#ifdef SYNC_GRAPHICS
	refreshCondition->Signal(); // unWait ... 
	re_mutex.Unlock();		// Note: If pthread_mutex_destroy() is called on a mutex that is locked by another thread, the request fails with an EBUSY error. 

	frame->glPane->stopTimer();	// stop invoking glPane->updateSim()
								// to avoid "pthread_mutex_trylock(): mutex not initialized" error upon exiting
#endif
	// waits for a joinable thread to terminate and returns the value
	Wait();		// wxThread::Wait
	
}


