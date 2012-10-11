//  SimulationThread.cpp
//  Sep 3, 2012
//	Project: Optimization Test
//  YL


#include "globalVariables.h"
#include "SimulationThread.h"
#include "TraceableStateMachineControllerA.h"
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



	p_fExtTorso = Vector3F::Zero();
	fExtTorsoICS = Vector6F::Zero();

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

		// apply external forces
		if (frame->enableExtForcesCheckBox->IsChecked())
		{
			applyExternalForces();
		}

		// Check if it's time for control
		if ((sim_time - last_control_time) >= cdt) 
		{
			//wxMutexLocker lock(mutexProtectSharedData);
			
			humanoidCtrl->StateControl();

			/*if (frame->logDataCheckBox->IsChecked()) 
			{
				//cout<<"log data!"<<endl;
				humanoidCtrl->logData();
			}*/
			
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

void SimulationThread::applyExternalForces()
{
	duty_count ++;
	if (duty_count >= 200)
	{
		duty_count = 0;
	}

	if ( duty_count == 0 )
	{
		p_fExtTorso<< 0.05, 0, 0.3;	// in torso coordinate
		fExtTorsoICS <<	0,0,0, 0, 0 , -300;
		CartesianVector p;
		RotationMatrix R;
		G_robot->getLink(0)->getPose(R,p);
		Matrix3F Rmat;
		copyRtoMat(R, Rmat);
		fExtTorso = Vector6F::Zero();
		fExtTorso.tail<3>() = Rmat * fExtTorsoICS.tail<3>();
		Vector6F f1; 	
		Matrix6F X = Matrix6F::Identity();
		X.block(3,0,3,3) = -cr3(p_fExtTorso);
		f1 = X.transpose() * fExtTorso;	// X is the 6D motion transform from link frame to applying location 
		dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->setExternalForce(f1.data());		
	}
	else if ( duty_count == 50 )
	{
		fExtTorso = Vector6F::Zero();
		Vector6F f0 =  Vector6F::Zero();
		dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->setExternalForce(f0.data());		
	}

	
}

