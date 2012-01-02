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
 *     File: dmArticulation.cpp
 *   Author: Scott McMillan
 *  Summary: Class declaration for articulated structures (tree structures)
 *****************************************************************************/

#include "dm.h"
#include "dmLink.hpp"
#include "dmArticulation.hpp"
#include "dmEnvironment.hpp"

//============================================================================
// class dmArticulation : public dmSystem
//============================================================================

//----------------------------------------------------------------------------
//    Summary: default constructor for dmArticulation objects
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmArticulation::dmArticulation()
      : dmSystem(),
        m_num_state_vars(0)
{
   for (unsigned int i=0; i<6; i++)
      m_ref_val.v[i] = 0.0;
}

//----------------------------------------------------------------------------
//    Summary: destructor for dmArticulation instances which also frees any
//             simulation variables that were allocated as links were added.
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmArticulation::~dmArticulation()
{
   while (m_link_list.size())
   {
      LinkInfoStruct *elt = m_link_list.back();
      m_link_list.pop_back();
      delete elt;
   }
}

//----------------------------------------------------------------------------
//    Summary: adds a link to the articulation (tree structure) as a child
//             (successor) of the parent link.  dmTreeElementStructs of parent
//             and child are updated to reflect the parent/child relationship.
// Parameters: new_link - ptr to an instantiated link that is to be added
//             parent   - ptr to link that is to be the parent of new_link. If
//               new_link is the first link in the articulation, parent must be
//               NULL.  This parameter can only be NULL if there are no links
//               added to the articulation (i.e., there can only be one tree,
//               and hence, one root link, per articulation).
//    Returns: true if operation is successful
//             false if unsuccessful (parent not found, variable allocation
//               failure, etc.)
//----------------------------------------------------------------------------
bool dmArticulation::addLink(dmLink *new_link, dmLink *parent)
{
   if (new_link == NULL)
   {
      cerr << "dmArticulation::addLink error: trying to add NULL link."
           << endl;
      return false;
   }

   // check to see if link has already been added
   if (getLinkIndex(new_link) != -1)
   {
      cerr << "dmArticulation::addLink error: trying to add link twice."
           << endl;
      return false;
   }

   // Note also returns -1 if parent == NULL
   int parent_index = getLinkIndex(parent);

   if (parent && parent_index == -1)
   {
      cerr << "dmArticulation::addNode error: parent not found." << endl;
      return false;
   }

#ifdef DEBUG
   cerr << "parent index " << parent_index << endl;
#endif

   LinkInfoStruct *link_info = new LinkInfoStruct;

// Add the link to the lists (m_num_link is the current index)
   link_info->link = new_link;
   //link_info->link_index = m_link_list.size();
   if (parent_index == -1)
      link_info->parent = NULL;
   else
      link_info->parent = m_link_list[parent_index];

   if (parent)
   {
      m_link_list[parent_index]->child_list.push_back(link_info);
   }

   link_info->index = m_link_list.size(); // useful in dmClosedArticulation
   m_link_list.push_back(link_info);
   m_num_state_vars += new_link->getNumDOFs();

   return true;
}

//----------------------------------------------------------------------------
//    Summary: get a ptr to a link given its index
// Parameters: index - link's index from 0 to m_num_links - 1
//    Returns: ptr to corresponding link, or NULL if index is out of range.
//----------------------------------------------------------------------------
dmLink *dmArticulation::getLink(unsigned int index) const
{
   dmLink *link = NULL;
   if (index < m_link_list.size())
   {
      link = m_link_list[index]->link;
   }

   return link;
}


//----------------------------------------------------------------------------
//    Summary: get a link's articulation index
// Parameters: link - ptr to the link in question
//    Returns: index of the link from 0 to m_num_links-1, or -1 of link pointer
//             is NULL or not contained in this articulation.
//----------------------------------------------------------------------------
int dmArticulation::getLinkIndex(dmLink *link) const
{
   if (link == NULL) return -1;

   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      if (link == m_link_list[i]->link)
      {
         return (int)i;
      }
   }

   // link not found
   return -1;
}

//----------------------------------------------------------------------------
//    Summary: get a pointer to link's parent
// Parameters: link index of child
//    Returns: pointer to child's parent link, or NULL if index is out of range
//             or there is no parent.
//----------------------------------------------------------------------------
dmLink *dmArticulation::getLinkParent(unsigned int index) const
{
   dmLink *link = NULL;
   if (index < m_link_list.size())
   {
      // first verify that the parent is not null (i.e., at the root level)
      if (m_link_list[index]->parent)
         link = m_link_list[index]->parent->link;
   }

   return link;
}

//----------------------------------------------------------------------------
//    Summary: Set the joint inputs (either torques, forces, or motor voltages)
//             for all the links in the articulation
// Parameters: packed array of joint inputs.  The array length equals the total
//             number of DOFS in the articulation, and the elements correspond
//             to DOFs of links in the order they were added to the
//             articulation.
//    Returns: none
//----------------------------------------------------------------------------
void dmArticulation::setJointInput(Float joint_input[])
{
   int joint_index_offset = 0;
   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      m_link_list[i]->link->setJointInput(&joint_input[joint_index_offset]);
      joint_index_offset += m_link_list[i]->link->getNumDOFs();
   }
}

//----------------------------------------------------------------------------
//    Summary: set the joint positions and velocities of the links in this
//             articulation.
// Parameters: joint_pos - packed array containing all joint positions.
//             joint_vel - packed array containing all joint velocities.
//    Returns: none
//----------------------------------------------------------------------------
void dmArticulation::setState(Float joint_pos[], Float joint_vel[])
{
   int joint_index_offset = 0;
   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      m_link_list[i]->link->setState(&joint_pos[joint_index_offset],
                                     &joint_vel[joint_index_offset]);
      joint_index_offset += m_link_list[i]->link->getNumDOFs();
   }
}

//----------------------------------------------------------------------------
//    Summary: get the joint positions and velocities of all the DOFs in the
//             articulation.
// Parameters: joint_pos - packed array to be filled with joint positions
//             joint_vel - packed array to be filled with joint velocities
//    Returns: none
//----------------------------------------------------------------------------
void dmArticulation::getState(Float joint_pos[], Float joint_vel[]) const
{
   int joint_index_offset = 0;
   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      m_link_list[i]->link->getState(&joint_pos[joint_index_offset],
                                     &joint_vel[joint_index_offset]);
      joint_index_offset += m_link_list[i]->link->getNumDOFs();
   }
}


//----------------------------------------------------------------------------
//    Summary: Tells every force object in articulation to store its state.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmArticulation::pushForceStates()
{
   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      m_link_list[i]->link->pushForceStates();
   }
}

//----------------------------------------------------------------------------
//    Summary: Tells every force object in articulation to restore its state.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmArticulation::popForceStates()
{
   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      m_link_list[i]->link->popForceStates();
   }
}

//----------------------------------------------------------------------------
//    Summary: Computes the gravitational potential energy of the articulation.
// Parameters: none
//    Returns: gravitational potential energy
//----------------------------------------------------------------------------
Float dmArticulation::getPotentialEnergy() const
{
   Float potentialEnergy = 0;
   CartesianVector g_ICS;
   dmEnvironment::getEnvironment()->getGravity(g_ICS);

   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      potentialEnergy +=
         m_link_list[i]->link->getPotentialEnergy(m_link_list[i]->link_val,
                                                  g_ICS);
   }

   return potentialEnergy;
}

//----------------------------------------------------------------------------
//    Summary: Computes the kinetic energy of the articulation.
// Parameters: none
//    Returns: kinetic energy
//----------------------------------------------------------------------------
Float dmArticulation::getKineticEnergy() const
{
   Float kineticEnergy = 0;

   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      kineticEnergy +=
         m_link_list[i]->link->getKineticEnergy(m_link_list[i]->link_val);
   }

   return kineticEnergy;
}

//----------------------------------------------------------------------------
//    Summary: compose the transformation (position and orientation) of the
//             link specified by the link_index.  Performs forward kinematics
//             from the base of the articulation to the specified link and adds
//             it to the transformation contained in fk -- and stores the final
//             result in fk.
// Parameters: link_index - desired link to compute to.
//             fk->R_ICS, fk->p_ICS - contain transformations to the base of
//                   the articulation (when invoked), and contains the final
//                   answer on return.
//    Returns: true if the operation was successful, false if the link_index is
//                   out of range.
//----------------------------------------------------------------------------
bool dmArticulation::forwardKinematics(unsigned int link_index,
                                       dmABForKinStruct &fk) const
{
   bool success = false;

   if (link_index < m_link_list.size())
   {
      unsigned int i, j;

      dmSystem::forwardKinematics(fk);

      for (i=0; i<=link_index; i++)
      {
         LinkInfoStruct *curr_link = m_link_list[i];

         if (m_link_list[i]->parent)
         {
            curr_link->link->forwardKinematics(curr_link->parent->link_val,
                                               curr_link->link_val);
         }
         else
         {
            curr_link->link->forwardKinematics(fk, curr_link->link_val);
         }
      }

      // copy result
      for (i=0; i<3; i++)
      {
         fk.p_ICS[i] = m_link_list[link_index]->link_val.p_ICS[i];
         for (j=0; j<3; j++)
         {
            fk.R_ICS[i][j] = m_link_list[link_index]->link_val.R_ICS[i][j];
         }
      }

      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
//   Function: forwardKinematics
//    Summary: Compute position and orientation of the specified link in a
//             specified chain.
// Parameters: articulation index [0, m_num_articulations-1] and link index,
//             or articulation index == -1 for the reference member.
//    Returns: 4x4 homogeneous transform matrix
//----------------------------------------------------------------------------
bool dmArticulation::forwardKinematics(
   unsigned int link_index, HomogeneousTransformationMatrix mat) const
{
   dmABForKinStruct fk;         // starts with ^{ICS}R_i, ^{ICS}p_i
                                // from dmSystem::forwardKinematics()

   if (forwardKinematics(link_index, fk))
   {
      // copy matrix
      for (int i = 0; i < 3; i++)
      {
         mat[i][3] = fk.p_ICS[i];
         mat[3][i] = 0.0;
         for (int j = 0; j < 3; j++)
         {
            mat[i][j] = fk.R_ICS[i][j];
         }
      }
      mat[3][3] = 1.0;

      return true;
   }
   else
   {
      cerr << "dmArticulation::forwardKinematics error: invalid link index "
           << link_index << endl;
      return false;
   }
}

// ---------------------------------------------------------------------------
// Function : dynamics
// Purpose  : routine to execute AB dynamic simulation
// Inputs   : ref mem and joint current state
// Outputs  : ref nem and joint velocities and accelerations (state
//            derivative).
// ---------------------------------------------------------------------------
void dmArticulation::dynamics(Float *qy, Float *qdy)
{
   register unsigned int i, j;

   for (i = 0; i < 6; i++)
   {
      m_beta_star_ref[i] = 0.0;
      for (j = i; j < 6; j++)
      {
         m_I_star_ref[i][j] = 0.0;
      }
   }

   // I. Forward Kinematics
   for (i = 0; i < 3; i++)
   {
      m_ref_val.v[i] = m_ref_val.v[i+3] = 0.0;
      m_ref_val.p_ICS[i] = m_p_ICS[i];
      for (j = 0; j < 3; j++)
      {
         m_ref_val.R_ICS[i][j] = m_R_ICS[j][i];
      }
   }

#ifdef DM_HYDRODYNAMICS
   CartesianVector fvel = {0,0,0}, facc = {0,0,0}, tem2;

   dmEnvironment::getEnvironment()->getFluidVel(fvel);
   dmEnvironment::getEnvironment()->getFluidAcc(facc);

   // compute a_f-a_g.
   rtxFromICS(fvel, m_ref_val.v_f);

   dmEnvironment::getEnvironment()->getGravity(tem2);

   facc[0] -= tem2[0];
   facc[1] -= tem2[1];
   facc[2] -= tem2[2];
   rtxFromICS(facc, m_ref_val.a_fg);
#endif

   ABForwardKinematics(&qy[0], &qy[getNumDOFs()], m_ref_val);

   // II. Backward Dynamics
   ABBackwardDynamics();

   // III. Forward Accelerations

   //------------------------------------------------------
   // compute biased acceleration for forward recursion.
   // XXX - maybe this should be done when the reference system is set.
   //       however, if the gravity vector should change in the environment,
   //       this object has no way of knowing
   m_accel_ref[0] = 0.0;
   m_accel_ref[1] = 0.0;
   m_accel_ref[2] = 0.0;

   CartesianVector g_ICS, g_ref;
   dmEnvironment::getEnvironment()->getGravity(g_ICS);
   rtxFromICS(g_ICS, g_ref);

   m_accel_ref[3] = -g_ref[0];
   m_accel_ref[4] = -g_ref[1];
   m_accel_ref[5] = -g_ref[2];
   //------------------------------------------------------

   ABForwardAccelerations(m_accel_ref, &qdy[0], &qdy[getNumDOFs()]);
}

//----------------------------------------------------------------------------
//    Summary: Perform first outward recursion of AB algorithm.
// Parameters: joint_{pos,vel} - packed array of joint pos/vel for the entire
//                 articulation as computed by the numerical integration
//                 algorithms (usually).  This is a way to set the state of the
//                 system without incurring the extra function call overhead by
//                 separately calling setState and then this function.
//             ref_val - pointer to struct filled with the kinematic parameters
//                 of the reference member.
//    Returns:
//----------------------------------------------------------------------------
void dmArticulation::ABForwardKinematics(Float joint_pos[],
                                         Float joint_vel[],
                                         const dmABForKinStruct &ref_val)
{
   int joint_index_offset = 0;

   for (unsigned int i=0; i < m_link_list.size(); i++)
   {
      LinkInfoStruct *curr_link = m_link_list[i];

      if (curr_link->parent)
      {
         curr_link->link->ABForwardKinematics(
            &joint_pos[joint_index_offset],
            &joint_vel[joint_index_offset],
            curr_link->parent->link_val,
            curr_link->link_val);
      }
      else
      {
         curr_link->link->ABForwardKinematics(
            &joint_pos[joint_index_offset],
            &joint_vel[joint_index_offset],
            ref_val,
            curr_link->link_val);
      }

      joint_index_offset += curr_link->link->getNumDOFs();
   }
}

//----------------------------------------------------------------------------
//    Summary: get the AB-computed forward kinematics values
// Parameters: link_index - desired link
//    Returns: dmABForKinStruct computed during ABForwardKinematics,
//             NULL if link_index is invalid
//----------------------------------------------------------------------------
const dmABForKinStruct *
dmArticulation::getForKinStruct(unsigned int link_index) const
{
   dmABForKinStruct *elt = NULL;

   if (link_index < m_link_list.size())
   {
      elt = &(m_link_list[link_index]->link_val);
   }

   return elt;
}

//----------------------------------------------------------------------------
//    Summary: 2nd (inward) iteration of AB dynamics algorithm.
// Parameters: none
//    Returns:
//----------------------------------------------------------------------------
void dmArticulation::ABBackwardDynamics()
{
   SpatialVector f_star_tmp;
   SpatialTensor I_refl_tmp;
   SpatialVector f_star_ref;
   SpatialTensor I_refl_ref;

   // =================================================================
   // clear variables at branch points in preparation for accumulation
   for (unsigned int k=0; k<m_link_list.size(); k++)
   {
      if (m_link_list[k]->child_list.size() > 1)
      {
         for (unsigned int i=0; i<6; i++)
         {
            m_link_list[k]->f_star[i] = 0;
            for (unsigned int j=i; j<6; j++)
            {
               m_link_list[k]->I_refl[i][j] =
                  m_link_list[k]->I_refl[j][i] = 0.0;
            }
         }
      }
   }

   // backward recursion through all links except the first.
   for (int i=m_link_list.size()-1; i>=0; i--)
   {
      // the 'first' links are always 'attached' to the reference system.
      // Note: dmArticulation should gaurantee at least one link (I hope)
      if (m_link_list[i]->parent == NULL)
      {
         if (m_link_list[i]->child_list.size() == 0)
         {
            m_link_list[i]->link->ABBackwardDynamicsN(
               m_link_list[i]->link_val,
               f_star_ref,
               I_refl_ref);
         }
         else
         {
            m_link_list[i]->link->ABBackwardDynamics(
               m_link_list[i]->link_val,
               m_link_list[i]->f_star,
               m_link_list[i]->I_refl,
               f_star_ref,
               I_refl_ref);
         }
      }

      // non-first links with siblings
      else if (m_link_list[i]->parent->child_list.size() > 1)
      {
         if (m_link_list[i]->child_list.size() == 0)
         {
            m_link_list[i]->link->ABBackwardDynamicsN(
               m_link_list[i]->link_val,
               f_star_tmp,
               I_refl_tmp);
         }
         else
         {
            m_link_list[i]->link->ABBackwardDynamics(
               m_link_list[i]->link_val,
               m_link_list[i]->f_star,
               m_link_list[i]->I_refl,
               f_star_tmp,
               I_refl_tmp);
         }

         // accumulate result
         for (unsigned int j=0; j<6; j++)
         {
            m_link_list[i]->parent->f_star[j] += f_star_tmp[j];
            for (unsigned int k=j; k<6; k++)
            {
               m_link_list[i]->parent->I_refl[j][k] =
                  m_link_list[i]->parent->I_refl[k][j] += I_refl_tmp[j][k];
            }
         }
      }

      // for non-first links with no siblings
      else
      {
         if (m_link_list[i]->child_list.size() == 0)
         {
            m_link_list[i]->link->ABBackwardDynamicsN(
               m_link_list[i]->link_val,
               m_link_list[i]->parent->f_star,
               m_link_list[i]->parent->I_refl);
         }
         else
         {
            m_link_list[i]->link->ABBackwardDynamics(
               m_link_list[i]->link_val,
               m_link_list[i]->f_star,
               m_link_list[i]->I_refl,
               m_link_list[i]->parent->f_star,
               m_link_list[i]->parent->I_refl);
         }
      }
   }
}

//----------------------------------------------------------------------------
//    Summary: 3rd (outward) recursion of AB alg. to compute joint accels.
// Parameters: a_ref - spatial acceleration of reference member expressed in
//                     body coordinate systems
//    Returns: the derivative of state:
//             joint_vel - packed array of joint velocities which correspond
//                         to the time derivatives of position vectors.
//                         This distinction is especially important when
//                         having to transform ang. vel to Euler angle rates
//             joint_acc - packed array, accelerations (time derivative of
//                         state vector velocities)
//----------------------------------------------------------------------------
void dmArticulation::ABForwardAccelerations(SpatialVector a_ref,
                                            Float joint_vel[],
                                            Float joint_acc[])
{
   int joint_index_offset = 0;

   for (unsigned int i=0; i<m_link_list.size(); i++)
   {
      if (m_link_list[i]->parent)
      {
         m_link_list[i]->link->ABForwardAccelerations(
            m_link_list[i]->parent->accel,
            m_link_list[i]->accel,
            &joint_vel[joint_index_offset],
            &joint_acc[joint_index_offset]);
      }
      else
      {
         m_link_list[i]->link->ABForwardAccelerations(
            a_ref,
            m_link_list[i]->accel,
            &joint_vel[joint_index_offset],
            &joint_acc[joint_index_offset]);

      }

      joint_index_offset += m_link_list[i]->link->getNumDOFs();
   }
}
