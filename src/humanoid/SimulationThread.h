/*
 *  SimulationThread.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __SIMULATION_THREAD_H__
#define __SIMUlATION_THREAD_H__

#include "wx/thread.h"

class SimulationThread : public wxThread {
public:	
	SimulationThread();
	virtual void *Entry();
    virtual void OnExit();
	void lockRobot();
	void unlockRobot();
	void requestStop();
	
private:
	wxMutex mutex;
	volatile bool stopRequested;
};


#endif