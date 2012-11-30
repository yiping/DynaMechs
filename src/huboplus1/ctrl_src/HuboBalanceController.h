// HuboBalanceController.h
// Nov 27, 2012
// YL


#ifndef __HUBO_BALANCE_CONTROLLER_H__
#define __HUBO_BALANCE_CONTROLLER_H__


#include "HuboController.h"

class HuboBalanceController : public HuboController
{
	public:
	
	HuboBalanceController(dmArticulation * robot);
	void StateControl();


	enum BalanceStates 
	{
		DROP,
		BALANCE_MIDDLE,
		NUM_STATES
	};

	
	
	void Drop();
	void BalanceMiddle();

protected:	
	
	typedef void (HuboBalanceController::*StateFuncPtr)();
	vector<StateFuncPtr> stateFunctions;
	Float kpCM, kdCM, kdAM;

};

#endif


