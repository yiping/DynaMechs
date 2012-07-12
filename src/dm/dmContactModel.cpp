/*****************************************************************************
 * DynaMechs: A Multibody Dynamic Simulation Library
 *
 * Copyright (C) 1994-2001  Scott McMillan   All Rights Reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *****************************************************************************
 *     File: dmContactModel.cpp
 *   Author: Scott McMillan
 *  Summary: Class declaration for contact modelling (contacts with terrain).
 *         : Used to be class EndEffector, but now is a component of RigidBody
 *         : so that contact forces on each body can be computed.
 *****************************************************************************/

#include "dm.h"
#include "dmForce.hpp"
#include "dmContactModel.hpp"
#include "dmEnvironment.hpp"

//============================================================================
// class dmContactModel : public dmForce
//============================================================================

//----------------------------------------------------------------------------
dmContactModel::dmContactModel() :
      dmForce(),
      m_reset_flag(true),
      m_num_contact_points(0),
      m_contact_flag(NULL),
      m_sliding_flag(NULL),
      m_contact_pos(NULL),
      m_initial_contact_pos(NULL),
      m_contact_flag_stored(NULL),
      m_sliding_flag_stored(NULL),
      m_initial_contact_pos_stored(NULL)
{
	m_last_computed_contact_force = new Float [6];
}

//----------------------------------------------------------------------------
dmContactModel::~dmContactModel()
{
   if (m_num_contact_points)
   {
      delete[]m_contact_flag;
      delete[]m_sliding_flag;

      delete[]m_contact_pos;
      delete[]m_initial_contact_pos;

      delete[]m_contact_flag_stored;
      delete[]m_sliding_flag_stored;
      delete[]m_initial_contact_pos_stored;
   }

	// need to clear traces on the heap to prevent memory leak

	delete [] m_last_computed_contact_force;
}

//----------------------------------------------------------------------------
//    Summary: set the list of contact points in local CS
// Parameters: num_contact_points - number of points to be initialized
//             contact_pts - Contact_Locations in local coordinate system
//    Returns: none
//----------------------------------------------------------------------------
void dmContactModel::setContactPoints(unsigned int num_contact_points,
                                      CartesianVector *contact_pos)
{
   m_reset_flag = true;

   // delete any previous contact points
   if (m_num_contact_points)
   {
      m_num_contact_points = 0;
      delete[]m_contact_flag;
      delete[]m_sliding_flag;
      delete[]m_contact_pos;
      delete[]m_initial_contact_pos;
      delete[]m_contact_flag_stored;
      delete[]m_sliding_flag_stored;
      delete[]m_initial_contact_pos_stored;
   }

   if (num_contact_points)
   {
      m_num_contact_points = num_contact_points;

      /* FIXME - need to error check this allocation */
      m_contact_flag = new bool[m_num_contact_points];
      m_sliding_flag = new bool[m_num_contact_points];

      m_contact_pos = new CartesianVector[m_num_contact_points];
      m_initial_contact_pos = new CartesianVector[m_num_contact_points];

      m_contact_flag_stored = new bool[m_num_contact_points];
      m_sliding_flag_stored = new bool[m_num_contact_points];
      m_initial_contact_pos_stored = new CartesianVector[m_num_contact_points];

      for (unsigned int i = 0; i < m_num_contact_points; i++)
      {
         m_contact_flag[i] = false;
         m_sliding_flag[i] = false;

         m_contact_pos[i][0] = contact_pos[i][0];
         m_contact_pos[i][1] = contact_pos[i][1];
         m_contact_pos[i][2] = contact_pos[i][2];
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: get the i-th contact point
// Parameters:
//    Returns: true if returned point is valid
//----------------------------------------------------------------------------
bool dmContactModel::getContactPoint(unsigned int index,
                                     CartesianVector pos) const
{
   if (index < m_num_contact_points)
   {
      pos[0] = m_contact_pos[index][0];
      pos[1] = m_contact_pos[index][1];
      pos[2] = m_contact_pos[index][2];
      return true;
   }

   return false;
}

//----------------------------------------------------------------------------
//    Summary: get the contact state of the i-th contact point
//           : (for implementation of a contact sensor).
// Parameters:
//    Returns: true if in contact, false otherwise
//----------------------------------------------------------------------------
bool dmContactModel::getContactState(unsigned int index) const
{
   if (index < m_num_contact_points)
      return m_contact_flag[index];
   else
   {
      cerr << "ERROR: Contact point index out of range" << endl;
      return false;
   }
}

//----------------------------------------------------------------------------
//    Summary: get the sliding state of the i-th contact point
//           : (for implementation of a contact sensor).
// Parameters:
//    Returns: true if sliding, false otherwise
//----------------------------------------------------------------------------
bool dmContactModel::getSlidingState(unsigned int index) const
{
	if (index < m_num_contact_points)
		return m_sliding_flag[index];
	else
	{
		cerr << "ERROR: Contact point index out of range" << endl;
		return false;
	}
}


//----------------------------------------------------------------------------
//    Summary: Saves (pushes) the current contact state
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmContactModel::pushState()
{
   for (unsigned int i = 0; i < m_num_contact_points; i++)
   {
      m_contact_flag_stored[i] = m_contact_flag[i];
      m_sliding_flag_stored[i] = m_sliding_flag[i];

      m_initial_contact_pos_stored[i][0] = m_initial_contact_pos[i][0];
      m_initial_contact_pos_stored[i][1] = m_initial_contact_pos[i][1];
      m_initial_contact_pos_stored[i][2] = m_initial_contact_pos[i][2];
   }
}

//----------------------------------------------------------------------------
//    Summary: Restores (pops) the contact state from storage
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmContactModel::popState()
{
   for (unsigned int i = 0; i < m_num_contact_points; i++)
   {
      m_contact_flag[i] = m_contact_flag_stored[i];
      m_sliding_flag[i] = m_sliding_flag_stored[i];

      m_initial_contact_pos[i][0] = m_initial_contact_pos_stored[i][0];
      m_initial_contact_pos[i][1] = m_initial_contact_pos_stored[i][1];
      m_initial_contact_pos[i][2] = m_initial_contact_pos_stored[i][2];
   }
}


//----------------------------------------------------------------------------
//    Summary: Compute the external force due to contact with terrain stored in
//             a dmEnvironment object
// Parameters: val - struct containing position and orientation of the rigid
//                   body wrt the inertial CS, and the spatial velocity
//                   with respect to the body's CS.
//    Returns: f_contact - spatial contact force exerted on the body wrt to the
//                   body's CS
//----------------------------------------------------------------------------
void dmContactModel::computeForce(const dmABForKinStruct &val,
                                  SpatialVector f_contact)
{
   register int j;
   Float ground_elevation;

   for (j = 0; j < 6; j++)
   {
      f_contact[j] = 0.0;
   }
   if (dmEnvironment::getEnvironment() == NULL)
   {
      return;
   }

   for (unsigned int i = 0; i < m_num_contact_points; i++)
   {
      for (j = 0; j < 3; j++)  // compute the contact pos.
      {                        // wrt ICS.

         current_pos[j] = val.p_ICS[j] +
                          val.R_ICS[j][0]*m_contact_pos[i][0] +
                          val.R_ICS[j][1]*m_contact_pos[i][1] +
                          val.R_ICS[j][2]*m_contact_pos[i][2];
      }

      ground_elevation =
         (dmEnvironment::getEnvironment())->getGroundElevation(current_pos,
                                                               normal);

      if (current_pos[2] > ground_elevation)  // No contact
      {
         // Reset flags.
         if (m_contact_flag[i] == true)
         {
            m_contact_flag[i] = false;
            m_boundary_flag = true;
         }
         m_sliding_flag[i] = false;

         // Store last position
         m_initial_contact_pos[i][0] = current_pos[0];
         m_initial_contact_pos[i][1] = current_pos[1];
         m_initial_contact_pos[i][2] = current_pos[2];
      }
      else                      // Contact!
      {
         if (!m_contact_flag[i] || m_reset_flag)  // set initial contact Pos.
         {
#ifdef DEBUG
            cout << "Contact " << flush;
#endif
            m_initial_contact_pos[i][0] = current_pos[0];
            m_initial_contact_pos[i][1] = current_pos[1];
            m_initial_contact_pos[i][2] = ground_elevation;
         }

         if (m_contact_flag[i] == false)
         {
            m_contact_flag[i] = true;
            m_boundary_flag = true;
         }

         // End-effector linear velocity and "spring" displacement wrt ICS.
         crossproduct(&val.v[0], m_contact_pos[i], vnC_pos);
         vnC_pos[0] += val.v[3];
         vnC_pos[1] += val.v[4];
         vnC_pos[2] += val.v[5];

         for (j = 0; j < 3; j++)
         {
            veC_pos[j] = val.R_ICS[j][0]*vnC_pos[0] +
                         val.R_ICS[j][1]*vnC_pos[1] +
                         val.R_ICS[j][2]*vnC_pos[2];
            peC_pos[j] = current_pos[j] - m_initial_contact_pos[i][j];
         }

         // Magnitudes of normal components of velocity and delta position.
         vtemp = veC_pos[0]*normal[0] +
                 veC_pos[1]*normal[1] +
                 veC_pos[2]*normal[2];
         ptemp = peC_pos[0]*normal[0] +
                 peC_pos[1]*normal[1] +
                 peC_pos[2]*normal[2];

         // Magnitude of Normal force.
         fe_normal_mag = -(dmEnvironment::getEnvironment())->
                                      getGroundNormalDamperConstant()*vtemp -
            (dmEnvironment::getEnvironment())->
                                      getGroundNormalSpringConstant()*ptemp;

         if (fe_normal_mag < 0.0)   // Invalid...trying to suck into ground.
         {
            fe[0] = fe[1] = fe[2] = 0.0;
         }
         else
         {
            for (j = 0; j < 3; j++)
            {
               fe_normal[j] = normal[j]*fe_normal_mag;
            }

            // Planar forces for sticking contact.
            for (j = 0; j < 3; j++)
            {
               v_planar[j] = veC_pos[j] - normal[j]*vtemp;
               p_planar[j] = peC_pos[j] - normal[j]*ptemp;
               fe_planar[j] = -(dmEnvironment::getEnvironment())->
                                 getGroundPlanarDamperConstant()*v_planar[j] -
                  (dmEnvironment::getEnvironment())->
                                 getGroundPlanarSpringConstant()*p_planar[j];
            }
            fe_planar_mag = sqrt(fe_planar[0]*fe_planar[0] +
                                 fe_planar[1]*fe_planar[1] +
                                 fe_planar[2]*fe_planar[2]);

            // Check to see whether it should start sticking.
            if (m_sliding_flag[i])
            {
               // if sliding, is it going slow enough to stick.
               //Float speedSquared = (v_planar[0]*v_planar[0] +
               //                      v_planar[1]*v_planar[1] +
               //                      v_planar[2]*v_planar[2]);

               // Stick if speed less than a threshhold or
               // static planar force is less than the kinetic one.
               if (             //(speedSquared < 0.005) ||
                  (fe_planar_mag < (fe_normal_mag*
                                    (dmEnvironment::getEnvironment())->
                                         getGroundKineticFrictionCoeff())))
               {
                  m_sliding_flag[i] = false;
                  m_boundary_flag = true;
               }
            }
            // Check to see whether it should start sliding.
            else
            {
               if (fe_planar_mag > (fe_normal_mag*
                      (dmEnvironment::getEnvironment())->
                                            getGroundStaticFrictionCoeff()))
               {
                  //slippage!
                  m_sliding_flag[i] = true;
                  m_boundary_flag = true;
               }
            }

            // if sliding recompute a smaller planar force
            if (m_sliding_flag[i])
            {
               temp = (fe_normal_mag/fe_planar_mag)*
                  (dmEnvironment::getEnvironment())->
                                            getGroundKineticFrictionCoeff();
               fe_planar[0] *= temp;
               fe_planar[1] *= temp;
               fe_planar[2] *= temp;

               m_initial_contact_pos[i][0] = current_pos[0];
               m_initial_contact_pos[i][1] = current_pos[1];
               m_initial_contact_pos[i][2] = ground_elevation;
            }

            // Add normal and planar forces.
            for (j = 0; j < 3; j++)
            {
               fe[j] = fe_normal[j] + fe_planar[j];
            }
         }

         // Compute Contact Force at link CS
         for (j = 0; j < 3; j++)
         {
            fn[j] = val.R_ICS[0][j]*fe[0] +
                    val.R_ICS[1][j]*fe[1] +
                    val.R_ICS[2][j]*fe[2];
         }
         crossproduct(m_contact_pos[i], fn, nn);

         // Accumulate for multiple contact points.
         for (j = 0; j < 3; j++)
         {
            f_contact[j] += nn[j];
            f_contact[j + 3] += fn[j];
         }
      }
   }
   m_reset_flag = false;

	for (j = 0; j < 6; j++)
	{
		m_last_computed_contact_force[j] = f_contact[j];
	}

}








/////////////////////////////////////////////////////

//! DM 5.0 function
/*! Compute the external force due to contact with terrain stored in
 a dmEnvironment object. */
void dmContactModel::computeForce(const dmRNEAStruct &val,
                                  SpatialVector f_contact)
{
   register int j;
   Float ground_elevation;

   for (j = 0; j < 6; j++)
   {
      f_contact[j] = 0.0;
   }
   if (dmEnvironment::getEnvironment() == NULL)
   {
      return;
   }

   for (unsigned int i = 0; i < m_num_contact_points; i++)
   {
      for (j = 0; j < 3; j++)  // compute the contact pos.
      {                        // wrt ICS.

         current_pos[j] = val.p_ICS[j] +
                          val.R_ICS[j][0]*m_contact_pos[i][0] +
                          val.R_ICS[j][1]*m_contact_pos[i][1] +
                          val.R_ICS[j][2]*m_contact_pos[i][2];
      }

      ground_elevation =
         (dmEnvironment::getEnvironment())->getGroundElevation(current_pos,
                                                               normal);

      if (current_pos[2] > ground_elevation)  // No contact
      {
         // Reset flags.
         if (m_contact_flag[i] == true)
         {
            m_contact_flag[i] = false;
            m_boundary_flag = true;
         }
         m_sliding_flag[i] = false;

         // Store last position
         m_initial_contact_pos[i][0] = current_pos[0];
         m_initial_contact_pos[i][1] = current_pos[1];
         m_initial_contact_pos[i][2] = current_pos[2];
      }
      else                      // Contact!
      {
         if (!m_contact_flag[i] || m_reset_flag)  // set initial contact Pos.
         {
#ifdef DEBUG
            cout << "Contact " << flush;
#endif
            m_initial_contact_pos[i][0] = current_pos[0];
            m_initial_contact_pos[i][1] = current_pos[1];
            m_initial_contact_pos[i][2] = ground_elevation;
         }

         if (m_contact_flag[i] == false)
         {
            m_contact_flag[i] = true;
            m_boundary_flag = true;
         }

         // End-effector linear velocity and "spring" displacement wrt ICS.
	 CartesianVector w;
	 w[0] = val.v(0); 
	 w[1] = val.v(1);
	 w[2] = val.v(2);
         crossproduct(w, m_contact_pos[i], vnC_pos);
         vnC_pos[0] += val.v(3);
         vnC_pos[1] += val.v(4);
         vnC_pos[2] += val.v(5);

         for (j = 0; j < 3; j++)
         {
            veC_pos[j] = val.R_ICS[j][0]*vnC_pos[0] +
                         val.R_ICS[j][1]*vnC_pos[1] +
                         val.R_ICS[j][2]*vnC_pos[2];
            peC_pos[j] = current_pos[j] - m_initial_contact_pos[i][j];
         }

         // Magnitudes of normal components of velocity and delta position.
         vtemp = veC_pos[0]*normal[0] +
                 veC_pos[1]*normal[1] +
                 veC_pos[2]*normal[2];
         ptemp = peC_pos[0]*normal[0] +
                 peC_pos[1]*normal[1] +
                 peC_pos[2]*normal[2];

         // Magnitude of Normal force.
         fe_normal_mag = -(dmEnvironment::getEnvironment())->
                                      getGroundNormalDamperConstant()*vtemp -
            (dmEnvironment::getEnvironment())->
                                      getGroundNormalSpringConstant()*ptemp;

         if (fe_normal_mag < 0.0)   // Invalid...trying to suck into ground.
         {
            fe[0] = fe[1] = fe[2] = 0.0;
         }
         else
         {
            for (j = 0; j < 3; j++)
            {
               fe_normal[j] = normal[j]*fe_normal_mag;
            }

            // Planar forces for sticking contact.
            for (j = 0; j < 3; j++)
            {
               v_planar[j] = veC_pos[j] - normal[j]*vtemp;
               p_planar[j] = peC_pos[j] - normal[j]*ptemp;
               fe_planar[j] = -(dmEnvironment::getEnvironment())->
                                 getGroundPlanarDamperConstant()*v_planar[j] -
                  (dmEnvironment::getEnvironment())->
                                 getGroundPlanarSpringConstant()*p_planar[j];
            }
            fe_planar_mag = sqrt(fe_planar[0]*fe_planar[0] +
                                 fe_planar[1]*fe_planar[1] +
                                 fe_planar[2]*fe_planar[2]);

            // Check to see whether it should start sticking.
            if (m_sliding_flag[i])
            {
               // if sliding, is it going slow enough to stick.
               //Float speedSquared = (v_planar[0]*v_planar[0] +
               //                      v_planar[1]*v_planar[1] +
               //                      v_planar[2]*v_planar[2]);

               // Stick if speed less than a threshhold or
               // static planar force is less than the kinetic one.
               if (             //(speedSquared < 0.005) ||
                  (fe_planar_mag < (fe_normal_mag*
                                    (dmEnvironment::getEnvironment())->
                                         getGroundKineticFrictionCoeff())))
               {
                  m_sliding_flag[i] = false;
                  m_boundary_flag = true;
               }
            }
            // Check to see whether it should start sliding.
            else
            {
               if (fe_planar_mag > (fe_normal_mag*
                      (dmEnvironment::getEnvironment())->
                                            getGroundStaticFrictionCoeff()))
               {
                  //slippage!
                  m_sliding_flag[i] = true;
                  m_boundary_flag = true;
               }
            }

            // if sliding recompute a smaller planar force
            if (m_sliding_flag[i])
            {
               temp = (fe_normal_mag/fe_planar_mag)*
                  (dmEnvironment::getEnvironment())->
                                            getGroundKineticFrictionCoeff();
               fe_planar[0] *= temp;
               fe_planar[1] *= temp;
               fe_planar[2] *= temp;

               m_initial_contact_pos[i][0] = current_pos[0];
               m_initial_contact_pos[i][1] = current_pos[1];
               m_initial_contact_pos[i][2] = ground_elevation;
            }

            // Add normal and planar forces.
            for (j = 0; j < 3; j++)
            {
               fe[j] = fe_normal[j] + fe_planar[j];
            }
         }

         // Compute Contact Force at link CS
         for (j = 0; j < 3; j++)
         {
            fn[j] = val.R_ICS[0][j]*fe[0] +
                    val.R_ICS[1][j]*fe[1] +
                    val.R_ICS[2][j]*fe[2];
         }
         crossproduct(m_contact_pos[i], fn, nn);

         // Accumulate for multiple contact points.
         for (j = 0; j < 3; j++)
         {
            f_contact[j] += nn[j];
            f_contact[j + 3] += fn[j];
         }
      }
   }
   m_reset_flag = false;

	for (j = 0; j < 6; j++)
	{
		m_last_computed_contact_force[j] = f_contact[j];
	}

}



Float* dmContactModel::getLastComputedValue() const
{
	return m_last_computed_contact_force;
}
