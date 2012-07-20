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

using namespace Eigen;

void getRotationMatrix(const Vector3F &z_ICS, const Vector3F &n_ICS, Matrix3F & patch_R_ICS)
{
	// z_ICS and n_ICS has to be normalized already !
	Float cosTheta = z_ICS.dot(n_ICS);
	Float Theta = acos(cosTheta);
	Float sinTheta = sin(Theta);

	if(Theta > 0) 
	{	
		Vector3F rotAxis_ICS; 
		rotAxis_ICS<< -n_ICS(1)/sinTheta, n_ICS(0)/sinTheta, 0;
		rotAxis_ICS.normalize();

		patch_R_ICS = Matrix3F::Identity()+ sinTheta*cr3(rotAxis_ICS) + (1-cosTheta)*cr3(rotAxis_ICS)*cr3(rotAxis_ICS);
	}
	else
	{
		patch_R_ICS = Matrix3F::Identity();
	}
}

bool isDifferent(const CartesianVector &a, const CartesianVector &b)
{
	if (sqrt(  (a[0]-b[0])*(a[0]-b[0]) 
			 + (a[1]-b[1])*(a[1]-b[1]) 
			 + (a[2]-b[2])*(a[2]-b[2]) ) > 1.0e-16 )
	{
		return true;
	}
	else
	{
		return false;
	}
}




//----------------------------------------------------------------------------
dmDynamicContactModel::dmDynamicContactModel()
{
	// will call dmContactModel constructor first
	cout<<"calling dmDynamicContactModel constructor ..."<<endl;
}

//----------------------------------------------------------------------------
dmDynamicContactModel::~dmDynamicContactModel()
{
	// will call dmContactModel destrctor first
	if (m_num_contact_points)
	{
		delete [] last_normal;
	}
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
	if (m_num_contact_points)
	{
		initializeDeformationStates();
		last_normal = new CartesianVector[m_num_contact_points];
		Matrix3F m = Matrix3F::Identity();
		for (int i = 0; i < m_num_contact_points; i++)
		{
			last_normal[i][0] = 0;
			last_normal[i][1] = 0;
			last_normal[i][2] = 1;
			last_patch_R_ICS.push_back(m);
		}
	}

}


void dmDynamicContactModel::initializeDeformationStates(void)
{
	vector<Float> tt (2, 0.0); // only 2 dof for each contact point!
	// arrays are not assignable

	
	for (int i=0; i< m_num_contact_points; i++)
	{
		u.push_back(tt);
		ud.push_back(tt);
		cout<<"contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<endl;
		cout<<"delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<endl;
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
	// Notation reminder  (all for a single contact point)

	// pc_ICS    				: position of a contact point
	// vc        				: linear velocity of a contact point (in body CS)
	// vc_ICS    				: linear velocity of a contact point in ICS
	// dc_ICS    				: displacement vector between pc_ICS and anchor point in ICS
	// vcn   	 				: normal component of a contact point velocity
	// dcn   					: normal component of displacement of a contact point 
	// vt_ICS    				: tangential velocity (perp. to ground normal) of a contact point in ICS
	// dt_ICS    				: tangential displacement between pc_ICS and anchor point in ICS

	// fe_normal_mag   			: magnitude of normal force 
	// fe_planar_mag_stick  	: magnitude of tangential contact force assuming sticking
	// fe_planar_mag_slip   	: magnitude of tangential contact force assuming slipping

	// fe_planar_ICS    		: tangential contact force in ICS [3x1]
	// fe_normal_ICS			: normal contact force in ICS [3x1]
	// fe_ICS					: (linear) contact force in ICS [3x1]
	// fe						: (linear) contact force in body CS [3x1]
	// ne						: moment in body CS [3x1]

	CartesianVector pc_ICS;
	CartesianVector vc;
	CartesianVector vc_ICS;
	CartesianVector dc_ICS;
	CartesianVector vt_ICS;
	CartesianVector dt_ICS;

	CartesianVector fe_planar_ICS;
	CartesianVector fe_normal_ICS;
	CartesianVector fe_ICS;
	CartesianVector fe;
	CartesianVector ne;
	CartesianVector ne2;  //-debug-

	Float vcn;
	Float dcn;
	Float fe_normal_mag;
	Float fe_planar_mag_stick;
	Float fe_planar_mag_slip;

	CartesianVector u_ICS;
	CartesianVector ud_ICS;


	unsigned int j;
	Float ground_elevation;

	for (j = 0; j < 6; j++)
	{
		f_contact[j] = 0.0;

		//-debug-
		f_contact_planar_damper[j] = 0.0; 
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

		ground_elevation = (dmEnvironment::getEnvironment())->getGroundElevation(pc_ICS, normal); //normal at pc_ICS
		//cout<<"pc_ICS(array): "<<pc_ICS[0]<<" "<<pc_ICS[1]<<" "<<pc_ICS[2]<<endl;

		if (pc_ICS[2] > ground_elevation)  // if NO contact
		{
			// Reset flags.
			if (m_contact_flag[i] == true)
			{
				m_contact_flag[i] = false;
				m_boundary_flag = true;
		 	}
		 	m_sliding_flag[i] = false;
			
			//cout<<"NO CONTACT | contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<"  "<<u[i][2]<<endl;
			//cout<<"NO CONTACT | delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<"  "<<ud[i][2]<<endl;
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
			//                                u[i] -> u_ICS
			Vector3F z_ICS;
			z_ICS<< 0,0,1;
		
			Matrix3F patch_R_ICS;
			
			Vector3F n_ICS;
			n_ICS = Map<Vector3F>(normal);
			//cout<<"normal_vec:   "<<n_ICS.transpose()<<endl;
			getRotationMatrix(z_ICS, n_ICS, patch_R_ICS);
			//cout<<patch_R_ICS<<endl;

			// if ground normal changes, re-project the u to the new patch surface.
			// i.e. modifying u outside Euler integrator is required...
			if (isDifferent(normal, last_normal[i]))
			{	
				Vector3F u_patch;
				u_patch << u[i][0], u[i][1], 0; // remember, only 2 dof for each contact point!
				Vector3F u_patch_reprojected;
				u_patch_reprojected = patch_R_ICS * last_patch_R_ICS[i].transpose() * u_patch; // R' = R^-1 
				u_patch_reprojected(0) = 0; //on surface
				for (j = 0; j < 3; j++)
				{
					u[i][j] = u_patch_reprojected(j);
				}
			}
			

			// TODO

			for (j = 0; j < 3; j++)
			{
				// u_ICS[j] = u[i][j];

				u_ICS[j] = patch_R_ICS(0,j)*u[i][0] + 
						   patch_R_ICS(1,j)*u[i][1] ; //   R' * u
			}

			
			for (j = 0; j < 3; j++)
			{
				vc_ICS[j] =  R_ICS[j][0]*vc[0] +
						 	 R_ICS[j][1]*vc[1] +
						 	 R_ICS[j][2]*vc[2];
				
			}
			//cout<<"IN CONTACT | contact       "<<i<<": "<<u[i][0]<<"  "<<u[i][1]<<"  "<<u[i][2]<<endl;
			//cout<<"IN CONTACT | delta contact "<<i<<": "<<ud[i][0]<<"  "<<ud[i][1]<<"  "<<ud[i][2]<<endl;

			// normal components of velocity and delta position.
			vcn = 	vc_ICS[0]*normal[0] +
				 	vc_ICS[1]*normal[1] +
				 	vc_ICS[2]*normal[2];
			dcn = 	(pc_ICS[2] - ground_elevation)*normal[2];

			// Magnitude of normal force.
			Float K, D, mu;
			D =  (dmEnvironment::getEnvironment())->getGroundNormalDamperConstant();
			K =  (dmEnvironment::getEnvironment())->getGroundNormalSpringConstant();
			mu = (dmEnvironment::getEnvironment())->getGroundKineticFrictionCoeff();

			fe_normal_mag = -D*vcn - K*dcn;

			fe_normal_mag = max(Float(0), fe_normal_mag); // no sucking force
			
			if (fe_normal_mag >0)
			{
		    	for (j = 0; j < 3; j++)
		    	{
					fe_normal_ICS[j] = normal[j]*fe_normal_mag; // normal contact force in ICS
		    	}			

				// Planar forces assuming sticking contact.
				for (j = 0; j < 3; j++)
				{
					vt_ICS[j] = vc_ICS[j] - normal[j]*vcn;
					dt_ICS[j] = u_ICS[j];
					fe_planar_ICS[j] = -D*vt_ICS[j] -K*dt_ICS[j];

					//-debug-
					fe_planar_damper_ICS[j] = -D*vt_ICS[j] ;
				}

				fe_planar_mag_stick = sqrt( fe_planar_ICS[0]*fe_planar_ICS[0] +
							  				fe_planar_ICS[1]*fe_planar_ICS[1] +
							 				fe_planar_ICS[2]*fe_planar_ICS[2]);  //?

				// now testing for slipping
				fe_planar_mag_slip = mu * fe_normal_mag;
				Float attenuator = fe_planar_mag_stick / fe_planar_mag_slip;
				if ( attenuator >1) // if outside the friction cone, adjust contact force
				{
					m_sliding_flag[i] = true;
					for (j = 0; j < 3; j++)
					{
						fe_planar_ICS[j] = fe_planar_ICS[j]/attenuator;

						//-debug-
						fe_planar_damper_ICS[j] /= attenuator;
					}
				}
				else
				{
					m_sliding_flag[i] = false;
				}
			
				for (j = 0; j < 3; j++)
				{
					ud_ICS[j] = -(fe_planar_ICS[j] + K * u_ICS[j]) /D;
				}
			}

			// Add normal and planar forces.
			for (j = 0; j < 3; j++)
			{
			   fe_ICS[j] = fe_planar_ICS[j] + fe_normal_ICS[j];
			}


			// Compute Contact Force at link CS
			for (j = 0; j < 3; j++)
			{
				fe[j] = R_ICS[0][j]*fe_ICS[0] +
						R_ICS[1][j]*fe_ICS[1] +
						R_ICS[2][j]*fe_ICS[2];

				//-debug-
            	fe_planar_damper[j] = R_ICS[0][j]*fe_planar_damper_ICS[0] +
                    			  	  R_ICS[1][j]*fe_planar_damper_ICS[1] +
                    			  	  R_ICS[2][j]*fe_planar_damper_ICS[2];

			}

			for (j = 0; j<2 ; j++)
			{
				// TODO ud: ICS -> terrain patch surface coord.
				// ud[i][j] = ud_ICS[j];
		
				ud[i][j] = patch_R_ICS(j,0)* ud_ICS[0] + 
						   patch_R_ICS(j,1)* ud_ICS[1] + 
						   patch_R_ICS(j,2)* ud_ICS[2]; //   R * ud_ICS
			}

			// update 'last'
			last_normal[i][0] = normal[0];
			last_normal[i][1] = normal[1];
			last_normal[i][2] = normal[2];
			last_patch_R_ICS[i] = patch_R_ICS;

			crossproduct(m_contact_pos[i], fe, ne); //moment

			//-debug-
			crossproduct(m_contact_pos[i], fe_planar_damper, ne2);

			// Accumulate for multiple contact points.
			for (j = 0; j < 3; j++)
			{
				f_contact[j] += ne[j];
				f_contact[j + 3] += fe[j];

				//-debug-
				f_contact_planar_damper[j] += ne2[j];
				f_contact_planar_damper[j+3] += fe_planar_damper[j];
			}
     	}


	}

	// cout<<"f_contact: "<<"[ "<<f_contact[0]<<" "<<f_contact[1]<<" "<<f_contact[2]<<" "<<f_contact[3]<<" "<<f_contact[4]<<" "<<f_contact[5]<<" ]"<<endl;
	for (j = 0; j < 6; j++)
	{
		m_last_computed_contact_force[j] = f_contact[j];

	  	//-debug-
	  	m_f_contact_planar_damper[j] = f_contact_planar_damper[j];
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
	//cout<<"ABForKinStruct compute force"<<endl;
}

void dmDynamicContactModel::draw() const
{

}

