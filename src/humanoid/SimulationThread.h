/*
 *  SimulationThread.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __SIMULATION_THREAD_H__
#define __SIMULATION_THREAD_H__

#include "wx/thread.h"

class SimulationThread : public wxThread {
public:	
	SimulationThread();
	~SimulationThread();
	virtual void *Entry();
	
	void unPause();
	void lockRobot();
	void unlockRobot();
	void requestStop();
	
private:
	wxMutex mutex;
	wxCondition * unPauseCondition;
	
	volatile bool stopRequested;
	
};


#endif