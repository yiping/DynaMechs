

#ifndef __TRACEABLE_STATE_MACHINE_CONTROLLER_A_H__
#define __TRACEABLE_STATE_MACHINE_CONTROLLER_A_H__

#include "controlDefs.h"
#include "dmArticulation.hpp"
#include "StateMachineControllerA.h"
#include "DataLogger.h"

class TraceableStateMachineControllerA : public DataLogger, public StateMachineControllerA
{
public:	
	TraceableStateMachineControllerA(dmArticulation * robot);

	void logData();
	void saveToFile();
	//string dataSaveDirectory;

private:
	
	//
	int SIM_TIME;  // 

	// group
	int G_HCOM, G_HCOM_DES, G_HCOMDOT_OPT, G_HCOMDOT_DES;
};

#endif
