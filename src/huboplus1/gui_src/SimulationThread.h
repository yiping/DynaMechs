
//  SimulationThread.h
//  Nov 26, 2012
//  YL

#ifndef __SIMULATION_THREAD_H__
#define __SIMULATION_THREAD_H__

#include "wx/thread.h"
#include <dm.h>
#include <dmIntegEuler.hpp>
#include "globalDefs.h"



class SimulationThread : public wxThread {
public:	
	SimulationThread();
	~SimulationThread();
	virtual void *Entry();
	
	void unPause();
	void requestStop();
	
	volatile Float idt, cdt, last_control_time, sim_time;
	dmIntegEuler *G_integrator;
	
	
	volatile bool paused_flag;


	//wxMutex mutexProtectSharedData;

#ifdef SYNC_GRAPHICS	
	wxMutex re_mutex;
	wxCondition * refreshCondition;
#endif
	


private:
	wxMutex mutex;
	wxCondition * unPauseCondition;
	Float Integ_count;


	
	volatile bool stopRequested;
	
};


#endif
