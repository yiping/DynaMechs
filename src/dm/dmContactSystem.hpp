/*****************************************************************************
 *     File: dmContactSystem.hpp
 *   Author: 
 *  Summary: Jun 21, 2012
 *         : 
 *         : 
 *         : 
 *****************************************************************************/

#ifndef _DM_CONTACT_SYSTEM_HPP
#define _DM_CONTACT_SYSTEM_HPP

#include "dm.h"
#include "dmSystem.hpp"
#include "dmDynamicContactModel.hpp"
#include "dmArticulation.hpp"
#include <vector>

//============================================================================

/**

A  dmContactSystem  object is subclassed from the  dmSystem. This class collects
the internal states due to the interaction between the robot and the environment. 

See also  dmEnvironment,  dmRigidBody,  dmContactModel, dmDynamicContactModel */

//============================================================================

class DM_DLL_API dmContactSystem : public dmSystem
{

	
public:
   ///
	dmContactSystem();
	virtual ~dmContactSystem();

	// 
	unsigned int getNumDOFs() const;
	void dynamics(Float *qy, Float *qdy);

	void setState(Float q[], Float qd[]);
	void getState(Float q[], Float qd[]) const;

	// These really shouldn't be here - yl
	void pushForceStates() {}
	void popForceStates() {}
	Float getPotentialEnergy() const { return 0; }
	Float getKineticEnergy() const { return 0; }

	///
	void draw() const;

	//
	void scanRobot(dmArticulation* robot);

private:
	vector<dmDynamicContactModel*> contacts;
	bool OKtoProceed;
};

#endif
