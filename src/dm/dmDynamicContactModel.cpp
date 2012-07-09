/*****************************************************************************
 *     File: dmDynamicContactModel.cpp
 *   Author: 
 *  Summary: YL
 *         : 
 *         : 
 *****************************************************************************/

#include "dm.h"
#include "dmForce.hpp"
#include "dmContactModel.hpp"
#include "dmEnvironment.hpp"
#include "dmDynamicContactModel.hpp"
#include <vector>

//----------------------------------------------------------------------------
dmDynamicContactModel::dmDynamicContactModel()
{
	// will call dmContactModel constructor first
}

//----------------------------------------------------------------------------
dmDynamicContactModel::~dmDynamicContactModel()
{
	// will call dmContactModel destrctor first
}

//----------------------------------------------------------------------------
//    Summary: set the list of contact points in local CS
// Parameters: num_contact_points - number of points to be initialized
//             contact_pts - Contact_Locations in local coordinate system
//    Returns: none
//----------------------------------------------------------------------------
void dmDynamicContactModel::setContactPoints(unsigned int num_contact_points,
                                      CartesianVector *contact_pos)
{
	dmContactModel::setContactPoints(num_contact_points, contact_pos);
	initializeDeformationStates();
}


void dmDynamicContactModel::initializeDeformationStates(void)
{
	vector<Float> tt (3, 0.0);
	// arrays are not assignable
	for (int i=0; i< m_num_contact_points; i++)
	{
		u.push_back(tt);
		ud.push_back(tt);
		cout<<"contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<"  "<<u[i][2]<<endl;
		cout<<"delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<"  "<<ud[i][2]<<endl;
	}
	cout<<"deformation initialized"<<endl;
}





//----------------------------------------------------------------------------
//    Summary: Compute the external force due to contact with terrain stored in
//             a dmEnvironment object
// Parameters: p_ICS - position of the rigid body (wrt ICS)
//             R_ICS - orientation of the rigid body (wrt ICS)
//             v - spatial velocity (wrt the body's CS)
//    Returns: f_contact - (total) spatial contact force exerted on the body wrt to the
//                   body's CS
//----------------------------------------------------------------------------
void dmDynamicContactModel::computeForceKernel(const CartesianVector p_ICS,
										const RotationMatrix R_ICS,
										const SpatialVector v,
                                  		SpatialVector f_contact)
{
	// Notation reminder
	// pc_ICS    : position of a contact point
	// vc        : linear velocity of a contact point (in body CS)
	// vc_ICS    : linear velocity of a contact point in ICS
	// dc_ICS    : displacement vector between pc_ICS and anchor point in ICS
	// vcn_mag   : magnitude of normal contact point velocity
	// dcn_mag   : magnitude of normal displacement component
	// fn_mag    : magnitude of normal force.
	// vt_ICS    : tangential velocity (perp. to ground normal) of a contact point in ICS
	// dt_ICS    : tangential displacement between pc_ICS and anchor point in ICS
	// ft_mag_stick   : magnitude of tangential contact force assuming sticking
	// ft_mag_slip    : magnitude of tangential contact force assuming slipping
	// ft_ICS    : tangential contact force in ICS

	CartesianVector pc_ICS;
	CartesianVector vc;
	CartesianVector vc_ICS;
	CartesianVector dc_ICS;
	CartesianVector vt_ICS;
	CartesianVector dt_ICS;
	CartesianVector ft_ICS;
	CartesianVector fn_ICS;
	Float vcn_mag;
	Float dcn_mag;
	Float fn_mag;
	Float ft_mag_stick;
	Float ft_mag_slip;
	CartesianVector u_ICS;
	CartesianVector ud_ICS;
	CartesianVector f_ICS;
	CartesianVector fn;
	CartesianVector nn;

	unsigned int j;
	Float ground_elevation;

	for (j = 0; j < 6; j++)
	{
		f_contact[j] = 0.0;
	}
	if (dmEnvironment::getEnvironment() == NULL)
	{
		cout<<"No environment found :( "<<endl;
		return;
	}

	for (unsigned int i = 0; i < m_num_contact_points; i++)
	{
		// for EACH contact point on this body:

		for (j = 0; j < 3; j++)  // compute the contact pos. wrt ICS.
		{                        
		 	pc_ICS[j] = 	 p_ICS[j] +
		                  	 R_ICS[j][0]*m_contact_pos[i][0] +
		                  	 R_ICS[j][1]*m_contact_pos[i][1] +
		                  	 R_ICS[j][2]*m_contact_pos[i][2]; //current contact position in ICS
	  	}

		ground_elevation = (dmEnvironment::getEnvironment())->getGroundElevation(current_pos, normal);


		if (pc_ICS[2] > ground_elevation)  // if NO contact
		{
			// Reset flags.
			if (m_contact_flag[i] == true)
			{
				m_contact_flag[i] = false;
				m_boundary_flag = true;
		 	}
		 	m_sliding_flag[i] = false;
			// reset
			cout<<"NO CONTACT | contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<"  "<<u[i][2]<<endl;
			cout<<"NO CONTACT | delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<"  "<<ud[i][2]<<endl;
		}
    	else   // if in contact
     	{
			if (m_contact_flag[i] == false)
			{
				m_contact_flag[i] = true;
				m_boundary_flag = true;
			}

			// Compute normal contact force
         	// contact point linear velocity and "spring" displacement wrt ICS.
         	crossproduct(&v[0], m_contact_pos[i], vc);
         	vc[0] += v[3];
         	vc[1] += v[4];
         	vc[2] += v[5];

			// TODO u: terrain patch surface coord -> ICS. 
			for (j = 0; j < 3; j++)
			{
				vc_ICS[j] =  R_ICS[j][0]*vc[0] +
						 	 R_ICS[j][1]*vc[1] +
						 	 R_ICS[j][2]*vc[2];
				u_ICS[j] = 	 u[i][j];
				dc_ICS[j] = u_ICS[j];
			}
			cout<<"IN CONTACT | contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<"  "<<u[i][2]<<endl;
			cout<<"IN CONTACT | delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<"  "<<ud[i][2]<<endl;

			// Magnitudes of normal components of velocity and delta position.
			vcn_mag = 	vc_ICS[0]*normal[0] +
				 		vc_ICS[1]*normal[1] +
				 		vc_ICS[2]*normal[2];
			dcn_mag = 	dc_ICS[0]*normal[0] +
				 		dc_ICS[1]*normal[1] +
				 		dc_ICS[2]*normal[2];

			// Magnitude of normal force.
			Float K, D, mu;
			D =  (dmEnvironment::getEnvironment())->getGroundNormalDamperConstant();
			K =  (dmEnvironment::getEnvironment())->getGroundNormalSpringConstant();
			mu = (dmEnvironment::getEnvironment())->getGroundKineticFrictionCoeff();
			fn_mag = -D*vcn_mag - K*dcn_mag;

			fn_mag = max(Float(0), fn_mag); // no sucking force
			if (fn_mag >0)
			{
		    	for (j = 0; j < 3; j++)
		    	{
					fn_ICS[j] = normal[j]*fn_mag;
		    	}			

				// Planar forces assuming sticking contact.
				for (j = 0; j < 3; j++)
				{
					vt_ICS[j] = vc_ICS[j] - normal[j]*vcn_mag;
					dt_ICS[j] = dc_ICS[j] - normal[j]*dcn_mag;
					ft_ICS[j] = -D*vt_ICS[j] -K*dt_ICS[j];
				}
				ft_mag_stick = sqrt(ft_ICS[0]*ft_ICS[0] +
							  		ft_ICS[1]*ft_ICS[1] +
							 		ft_ICS[2]*ft_ICS[2]);  //?

				// now testing for slipping
				ft_mag_slip = mu * fn_mag;
				Float attenuator = ft_mag_stick/ft_mag_slip;
				if ( attenuator >1) // if outside the friction cone, adjust contact force
				{
					m_sliding_flag[i] = true;
					for (j = 0; j < 3; j++)
						ft_ICS[j] = ft_ICS[j]/attenuator;
				}
				else
				{
					m_sliding_flag[i] = false;
				}
			
				for (j = 0; j < 3; j++)
				{
					ud_ICS[j] = -(ft_ICS[j] + K * u_ICS[j]) /D;
				}
			}

			// Add normal and planar forces.
			for (j = 0; j < 3; j++)
			{
			   f_ICS[j] = ft_ICS[j] + fn_ICS[j];
			}


			// Compute Contact Force at link CS
			for (j = 0; j < 3; j++)
			{
				fn[j] = R_ICS[0][j]*f_ICS[0] +
						R_ICS[1][j]*f_ICS[1] +
						R_ICS[2][j]*f_ICS[2];
				ud[i][j] = ud_ICS[j];
			}

			// TODO ud: ICS -> terrain patch surface coord.

			crossproduct(m_contact_pos[i], fn, nn); //moment

			// Accumulate for multiple contact points.
			for (j = 0; j < 3; j++)
			{
				f_contact[j] += nn[j];
				f_contact[j + 3] += fn[j];
			}
     	}
	}

}




//! DM 5.0 function
/*! Compute the external force, taking dmRNEAStruct as input parameter */
void dmDynamicContactModel::computeForce(const dmRNEAStruct &val,
                                  SpatialVector f_contact)
{
	SpatialVector v;
	for (int i=0; i<6;i++)
		v[i] = val.v(i);
	computeForceKernel(val.p_ICS, val.R_ICS, v, f_contact);
}

//! DM 5.0 function
/*! Compute the external force, taking dmABForKinStruct as input parameter */
void dmDynamicContactModel::computeForce(const dmABForKinStruct & val,
                                  SpatialVector f_contact)
{
	computeForceKernel(val.p_ICS, val.R_ICS, val.v, f_contact);
	cout<<"ABForKinStruct compute force"<<endl;
}

void dmDynamicContactModel::draw() const
{

}

