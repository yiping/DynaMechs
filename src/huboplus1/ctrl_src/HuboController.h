// HuboController.h
// Nov 27, 2012
// YL

#ifndef __HUBO_CONTROLLER_H__
#define __HUBO_CONTROLLER_H__

#include <string>
#include "DataLogger.h"
#include "HumanoidControllerStateMachine.h"

using namespace std;

class HuboController : public DataLogger, public HumanoidControllerStateMachine 
{
	
public:	
	HuboController(dmArticulation * robot, int stateSize);
	void logData();
	void saveData();
	string dataSaveDirectory;
private:
	//
	int SIM_TIME, STATE_CODE;   

	// group
	int G_COM_POS, G_COM_POS_DES, G_ZMP_WRENCH, G_ZMP_WRENCH_OPT, G_ZMP_POS, G_ZMP_POS_OPT;
	int G_HCOM, G_HCOM_DES, G_HCOMDOT_OPT, G_HCOMDOT_DES;
};

#endif

	
