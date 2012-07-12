/*****************************************************************************
 *     File: dmDynamicContactModel.hpp
 *   Author: 
 *  Summary: YL
 *         : 
 *         : 
 *         : 
 *****************************************************************************/

#ifndef _DM_DYNAMIC_CONTACT_MODEL_HPP
#define _DM_DYNAMIC_CONTACT_MODEL_HPP

#include "dm.h"
#include "dmEnvironment.hpp"
#include "dmForce.hpp"
#include "dmContactModel.hpp"
#include <vector>

/**

A  dmDynamicContactModel  object is subclassed from the  dmContactModel and computes
the compliant forces of contact with terrain. A dmContactSystem must be instantiated
to track the internal states for each dmDynamicContactModel object.

This contact model is based on Roy Featherstone's algorithm.

See also  dmEnvironment, dmContactForce, dmContactSystem  */



class DM_DLL_API dmDynamicContactModel : public dmContactModel
{
	// A derived class cannot access private members in the base class. 

public:


	dmDynamicContactModel();
	virtual ~dmDynamicContactModel();// polymorphic deletes require a virtual base destructor
	void initializeDeformationStates();
	// reimplementation
	void setContactPoints(unsigned int num_contact_points, CartesianVector *contact_pos);
	void computeForce(const dmABForKinStruct &val, SpatialVector force);
	void computeForce(const dmRNEAStruct &val, SpatialVector f_contact);

	void computeForceKernel(const CartesianVector p_ICS,
							const RotationMatrix R_ICS,
							const SpatialVector v,
                            SpatialVector f_contact);
	// rendering functions:
	void draw() const;
	

public:
	//vector<Float *> u;  //tip: you cannot use CartesianVector here, b/c arrays are not assignable
	//vector<Float *> ud;

	vector<vector<Float> > u;
	vector<vector<Float> > ud; // later should be somehow made private


};

#endif
