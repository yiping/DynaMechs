/*****************************************************************************
 *     File: dmContactSystem.cpp
 *   Author: 
 *  Summary: Jun 21, 2012
 *         : 
 *         : 
 *****************************************************************************/

#include "dm.h"
#include "dmDynamicContactModel.hpp"
#include "dmSystem.hpp"
#include "dmArticulation.hpp"
#include "dmRigidBody.hpp"
#include "dmContactSystem.hpp"


dmContactSystem::dmContactSystem(): dmSystem()
{
	OKtoProceed = false;
}

dmContactSystem::~dmContactSystem()
{
	
}

unsigned int dmContactSystem::getNumDOFs() const
{
	unsigned int sum = 0;
	vector<dmDynamicContactModel *>::const_iterator it;
	for ( it= contacts.begin() ; it < contacts.end(); it++ )	
		sum += 2*(*it)->getNumContactPoints();
	return sum;
}

/*! Only to be used by dmIntegEuler integrator */
void dmContactSystem::dynamics(Float *qy, Float *qdy)
{
	// Only to be used by dmIntegEuler integrator

	unsigned int base = getNumDOFs();
	unsigned int offset = 0;
	vector<dmDynamicContactModel *>::iterator it;
	for (it = contacts.begin(); it < contacts.end(); it++)
	{
		for (int i = 0; i<(*it)->getNumContactPoints(); i++)
		{
			(*it)->u[i][0] = qdy[offset+2*i  ] ; //update
			(*it)->u[i][1] = qdy[offset+2*i+1] ; //update
			qdy[base+offset + 2*i  ] = (*it)->ud[i][0];
			qdy[base+offset + 2*i+1] = (*it)->ud[i][1];
		}		
		offset += 2*(*it)->getNumContactPoints();
	}

}

void dmContactSystem::getState(Float q[], Float qd[]) const
{
	unsigned int offset = 0;
	vector<dmDynamicContactModel *>::const_iterator it;
	for (it = contacts.begin(); it < contacts.end(); it++)
	{
		for (int i = 0; i<(*it)->getNumContactPoints(); i++)
		{
			qd[offset+2*i] = (*it)->u[i][0];
			qd[offset+2*i+1] = (*it)->u[i][1];
			q[offset+2*i] = 99;  //dummy fillings
			q[offset+2*i+1] = 99; //dummy fillings
		}
		offset += 2*(*it)->getNumContactPoints();
	}
}


void dmContactSystem::setState(Float q[], Float qd[])
{
	unsigned int offset = 0;
	vector<dmDynamicContactModel *>::iterator it;
	for (it = contacts.begin(); it < contacts.end(); it++)
	{
		for (int i = 0; i<(*it)->getNumContactPoints(); i++)
		{
			(*it)->u[i][0] = qd[offset+2*i] ;
			(*it)->u[i][1] = qd[offset+2*i+1] ;
		}
		offset += 2*(*it)->getNumContactPoints();
	}

}


void dmContactSystem::scanRobot(dmArticulation* robot)
{

	for (unsigned int i =0; i< robot->getNumLinks(); i++)
	{
		dmLink* link;
		dmForce* force;
		link = robot->getLink(i);
		cout<<"link "<<i<<" DOF : "<<link->getNumDOFs()<<"   ";
		dmRigidBody* rb;
		// test for dmRigidbody
		rb = dynamic_cast<dmRigidBody*>(link);
		if (rb != NULL)
		{
			cout<<"is dmRigidBody, "<<"  ";
			force = rb->getForce(0);
			unsigned int nF = rb->getNumForces();
			
			for (unsigned int j =0; j< nF; j++)
			{
				dmDynamicContactModel* cf;
				// test for dmDynamicContactModel				
				cf = dynamic_cast<dmDynamicContactModel*>(rb->getForce(j));
				if (cf != NULL)
				{
					cout<<"f_"<<j<<"  ";
					contacts.push_back(cf);
				}
			}
			cout<<endl;
		} 
		else
			cout<<"Not a dmRigidBody ...  "<<endl;
	}
	
	if (contacts.size())
		 OKtoProceed = true;
	else
		cout<<"No dynamic contacts found!"<<endl;
}

void dmContactSystem::draw() const
{
}
