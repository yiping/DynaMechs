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
 *     File: dmClosedArticulation.cpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Class definition for articulations with closed loops.
 *****************************************************************************/

typedef long int integer;
typedef float real;
typedef double doublereal;

extern "C" int ssvdc_(real *x, integer *ldx, integer *n, integer *p,
                      real *s, real *e, real *u, integer *ldu,
                      real *v, integer *ldv, real *work, integer *job,
                      integer *info);

extern "C" int dsvdc_(doublereal *x, integer *ldx, integer *n, integer *p,
                      doublereal *s, doublereal *e, doublereal *u,
                      integer *ldu,
                      doublereal *v, integer *ldv, doublereal *work,
                      integer *job,
                      integer *info);

#include "dmClosedArticulation.hpp"
#include <algorithm>

//============================================================================
// class dmClosedArticulation : public dmArticulation
//============================================================================

//----------------------------------------------------------------------------
//    Summary: default constructor for dmClosedArticulation objects
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmClosedArticulation::dmClosedArticulation() :
      dmArticulation(),
      m_num_elements_LB(NULL),
      m_LB(NULL),
      m_num_elements_LR(NULL),
      m_LR(NULL),
      m_num_elements_LC(NULL),
      m_LC(NULL),
      m_num_elements_LJ_star(NULL),
      m_LJ_star(NULL),
      m_Xik(NULL),
      m_Bmn(NULL),
      m_zetak(NULL),
      m_Binv_zetai(NULL),
      m_Binv_Xi(NULL),
      m_Binv_Bim(NULL),
      m_constraints_at_root(NULL),
      m_lambda_c(NULL),
      m_sec_joints(),
      m_soft_joints(),
      m_loop_root_index(NULL),
      m_num_constraints(NULL)
{

#ifdef DEBUG
   cout << "dmClosedArticulation constructor: enter\n" << flush;
#endif

#ifdef DEBUG
   cout << "dmClosedArticulation constructor: exit\n" << flush;
#endif
}

//----------------------------------------------------------------------------
//    Summary: destructor for dmClosedArticulation instances which
//           : also frees any simulation variables that were allocated as
//           : links were added.
// Parameters: none.
//    Returns: none.
//----------------------------------------------------------------------------
dmClosedArticulation::~dmClosedArticulation()
{
#ifdef DEBUG
   cout << "dmClosedArticulation destructor:  \n" << flush;
#endif
   freeKinematicLoopVars();
}

//----------------------------------------------------------------------------
void dmClosedArticulation::freeKinematicLoopVars()
{
   unsigned int i;
//        m_lambda_c(NULL),
   for (i=0; i<m_sec_joints.size(); ++i)
      delete [] m_lambda_c[i];
   delete [] m_lambda_c;

//        m_Binv_Bim(NULL),
   for (i = 0; i < m_link_list.size(); i++)
   {
      for (unsigned int j = 0; j < m_num_elements_LJ_star[i]; j++)
      {
         unsigned int m = m_LJ_star[i][j];

         for (unsigned int r = 0; r < m_constraints_at_root[i]; r++)
            delete [] m_Binv_Bim[i][m][r];

         delete [] m_Binv_Bim[i][m];
      }
      delete [] m_Binv_Bim[i];
   }
   delete [] m_Binv_Bim;

//        m_Binv_Xi(NULL),
   for (i = 0; i < m_link_list.size(); i++)
   {
      for (unsigned int j = 0; j < m_constraints_at_root[i]; j++)
         delete [] m_Binv_Xi[i][j];

      delete [] m_Binv_Xi[i];
   }
   delete [] m_Binv_Xi;

//        m_Binv_zetai(NULL),
   for (i = 0; i < m_link_list.size(); i++)
      delete [] m_Binv_zetai[i];
   delete [] m_Binv_zetai;

//        m_constraints_at_root(NULL),
   delete [] m_constraints_at_root;

//        m_Bmn(NULL),
   for (unsigned int m = 0; m < m_sec_joints.size(); m++)
   {
      for (unsigned int i = 0; i < m_num_elements_LC[m]; i++)
      {
         unsigned int n = m_LC[m][i];

         for (unsigned int r = 0; r < m_num_constraints[m]; r++)
            delete [] m_Bmn[m][n][r];

         delete [] m_Bmn[m][n];
      }

      // deallocate Bmm.
      for (unsigned int r = 0; r < m_num_constraints[m]; r++)
         delete [] m_Bmn[m][m][r];
      delete [] m_Bmn[m][m];

      delete [] m_Bmn[m];
   }
   delete [] m_Bmn;

//        m_zetak(NULL),
   for (i = 0; i < m_sec_joints.size(); ++i)
      delete [] m_zetak[i];
   delete [] m_zetak;

//        m_Xik(NULL),
   for (i = 0; i < m_link_list.size(); i++)
   {
      for (unsigned int k = 0; k < m_sec_joints.size(); k++)
      {
         // Only allocate the matrix if k is an element of LB(i) or LR(i).
         unsigned int j;
         bool member_flag = false;
         for (j = 0; j < m_num_elements_LR[i]; j++)
            if (m_LR[i][j] == k)
               member_flag = true;

         for (j = 0; j < m_num_elements_LB[i]; j++)
            if (m_LB[i][j] == k)
               member_flag = true;

         if (member_flag)
         {
            for (unsigned int r = 0; r < 6; r++)
            {
               delete [] m_Xik[i][k][r];
            }

            delete [] m_Xik[i][k];
         }
      }
      delete [] m_Xik[i];
   }
   delete [] m_Xik;

//        m_num_elements_LJ_star(NULL),
//        m_LJ_star(NULL),
   for (i = 0; i < m_link_list.size(); i++)
   {
      delete [] m_LJ_star[i];
   }
   delete [] m_LJ_star;
   delete [] m_num_elements_LJ_star;

//        m_num_elements_LC(NULL),
//        m_LC(NULL),
   for (i = 0; i < m_sec_joints.size(); i++)
   {
      if (m_LC[i]) free(m_LC[i]);  // what a HACK deletes and frees together
   }
   delete [] m_LC;
   delete [] m_num_elements_LC;

//        m_num_elements_LR(NULL),
//        m_LR(NULL),
//        m_num_elements_LB(NULL),
//        m_LB(NULL),
   for (i=0; i<m_link_list.size(); i++)
   {
      if (m_LR[i]) free(m_LR[i]);
      if (m_LB[i]) free(m_LB[i]);
   }
   delete [] m_LR;
   delete [] m_num_elements_LR;
   delete [] m_LB;
   delete [] m_num_elements_LB;

//        m_num_constraints(NULL)
//        m_loop_root_index(NULL),
   delete [] m_num_constraints;
   delete [] m_loop_root_index;
}

//----------------------------------------------------------------------------
//    Summary: get a ptr to a hard secondary joint given its index
// Parameters: index - secondary joint's index
//    Returns: ptr to corresponding secondary joint,or NULL if out of range.
//----------------------------------------------------------------------------
dmSecondaryJoint *dmClosedArticulation::getHardSecondaryJoint(
   unsigned int index) const
{
   dmSecondaryJoint *secJoint = NULL;
   if (index < m_sec_joints.size())
   {
      secJoint = m_sec_joints[index];
   }

   return secJoint;
}

//----------------------------------------------------------------------------
//    Summary: get a ptr to a soft secondary joint given its index
// Parameters: index - secondary joint's index
//    Returns: ptr to corresponding secondary joint,or NULL if out of range.
//----------------------------------------------------------------------------
dmSecondaryJoint *dmClosedArticulation::getSoftSecondaryJoint(
   unsigned int index) const
{
   dmSecondaryJoint *secJoint = NULL;
   if (index < m_soft_joints.size())
   {
      secJoint = m_soft_joints[index];
   }

   return secJoint;
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
void dmClosedArticulation::ABForwardKinematics(Float joint_pos[],
                                               Float joint_vel[],
                                               const dmABForKinStruct &ref_val)
{
#ifdef DEBUG
   cout << "dmClosedArticulation::ABForwardKinematics\n" << flush;
#endif

   dmArticulation::ABForwardKinematics(joint_pos, joint_vel, ref_val);

   unsigned int i,k;

   for (k = 0; k < m_sec_joints.size(); k++)
   {
      m_sec_joints[k]->computeState();
      m_sec_joints[k]->computeAppliedForce();
   }

   for (k = 0; k < m_soft_joints.size(); k++)
   {
      m_soft_joints[k]->computeState();
      m_soft_joints[k]->computeAppliedForce();
      m_soft_joints[k]->applyPenaltyForce();
   }

   // ********************************************************
   // Save some time if all secondary joints are 'soft'.
   // This just eliminates empty loop executions.
   if (m_sec_joints.empty())
      return;
   // ********************************************************


   // Initialize Xik's of p1(k) and p2(k) links.
   // Clear Xik's of loop roots for accumulation.
   for (i = 0; i < m_link_list.size(); i++)
      for (k = 0; k < m_sec_joints.size(); k++)
         if (m_Xik[i][k] != NULL)
            m_sec_joints[k]->initXik(m_Xik[i][k], i, m_loop_root_index[k]);

   for (k = 0; k < m_sec_joints.size(); k++)
   {
      m_sec_joints[k]->computeEtas();
      m_sec_joints[k]->getZeta(m_zetak[k]);
   }

   // Clear Bmn's.
   unsigned int m, n, r, c;
   for (m = 0; m < m_sec_joints.size(); m++)
      for (n = 0; n < m_sec_joints.size(); n++)
         if (m_Bmn[m][n] != NULL)
         {
            for (r = 0; r < m_num_constraints[m]; r++)
               for (c = 0; c < m_num_constraints[n]; c++)
                  m_Bmn[m][n][r][c] = 0.0;
         }
}


//----------------------------------------------------------------------------
//    Summary: 2nd (inward) iteration of AB dynamics algorithm.
// Parameters: none
//    Returns:
//----------------------------------------------------------------------------
void dmClosedArticulation::ABBackwardDynamics()
{
   // ********************************************************
   // Save some time if all secondary joints are 'soft'.
   // This just eliminates empty loop executions.
   if (m_sec_joints.empty())
   {
      dmArticulation::ABBackwardDynamics();
      return;
   }
   // ********************************************************

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
   for (unsigned int i=m_link_list.size()-1; i>=1; i--)
   {
      // Eliminate loops at root.
      if (m_num_elements_LR[i] > 0)
         eliminateLoops(i);

      // non-first links with siblings
      if (m_link_list[i]->parent->child_list.size() > 1)
      {
         if (m_link_list[i]->child_list.empty())
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
         if (m_link_list[i]->child_list.empty())
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

      // Propagate secondary joint constraints to previous link.
      propagateConstraints(i);
   }


   // Do the first link - always attached to the reference member.

   // Eliminate loops at first link.
   if (m_num_elements_LR[0] > 0)
      eliminateLoops(0);

   if (m_link_list[0]->child_list.empty())
   {
      m_link_list[0]->link->ABBackwardDynamicsN(
         m_link_list[0]->link_val,
         f_star_ref,
         I_refl_ref);
   }
   else
   {
      m_link_list[0]->link->ABBackwardDynamics(
         m_link_list[0]->link_val,
         m_link_list[0]->f_star,
         m_link_list[0]->I_refl,
         f_star_ref,
         I_refl_ref);
   }

   // No propagateConstraints(0), since all loops in articulation closed
   // at or before first link, or LB(0) = Null Set.
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
void dmClosedArticulation::ABForwardAccelerations(SpatialVector a_ref,
                                                  Float joint_vel[],
                                                  Float joint_acc[])
{
   // ********************************************************
   // Save some time if all secondary joints are 'soft'.
   // This just eliminates empty loop executions.
   if (m_sec_joints.empty())
   {
      dmArticulation::ABForwardAccelerations(a_ref, joint_vel, joint_acc);
      return;
   }
   // ********************************************************


   m_link_list[0]->link->ABForwardAccelerations(a_ref,
                                                m_link_list[0]->accel,
                                                &joint_vel[0],
                                                &joint_acc[0]);

   // Compute constraint forces of loops whose root is first link.
   if (m_num_elements_LR[0] > 0)
      computeConstraintForces(0);

   unsigned int joint_index_offset = m_link_list[0]->link->getNumDOFs();

   for (unsigned int i = 1; i < m_link_list.size(); i++)
   {
      m_link_list[i]->link->ABForwardAccelerations(
         m_link_list[i]->parent->accel,
         m_LB[i],
         m_num_elements_LB[i],
         m_Xik[i],
         m_lambda_c,
         m_num_constraints,
         m_link_list[i]->accel,
         &joint_vel[joint_index_offset],
         &joint_acc[joint_index_offset]);

      // Compute constraint forces of loops whose root is link i.
      if (m_num_elements_LR[i] > 0)
         computeConstraintForces(i);

      joint_index_offset += m_link_list[i]->link->getNumDOFs();
   }
}

//----------------------------------------------------------------------------
//    Summary: Compute constraint forces of loops rooted at given link.
//             Called during the forward accelerations recursion of the AB
//             algorithm to compute the final numerical value of the forces.
// Parameters: i - link index of root link
//    Returns: sets constraint force member variables
//----------------------------------------------------------------------------
void dmClosedArticulation::computeConstraintForces(unsigned int i)
{
   Float *lambda_star = (Float *)malloc(m_constraints_at_root[i]*
                                        sizeof(Float));

   for (unsigned int j = 0; j < m_constraints_at_root[i]; j++)
   {
      lambda_star[j] = (m_Binv_zetai[i][j]
                        -  m_Binv_Xi[i][j][0]*(m_link_list[i]->accel)[0]
                        -  m_Binv_Xi[i][j][1]*(m_link_list[i]->accel)[1]
                        -  m_Binv_Xi[i][j][2]*(m_link_list[i]->accel)[2]
                        -  m_Binv_Xi[i][j][3]*(m_link_list[i]->accel)[3]
                        -  m_Binv_Xi[i][j][4]*(m_link_list[i]->accel)[4]
                        -  m_Binv_Xi[i][j][5]*(m_link_list[i]->accel)[5]);

      for (unsigned int p = 0; p < m_num_elements_LJ_star[i]; p++)
      {
         int m = m_LJ_star[i][p];

         for (unsigned int q = 0; q < m_num_constraints[m]; q++)
         {
            lambda_star[j] -= (m_Binv_Bim[i][m][j][q]*
                               m_lambda_c[m][q]);
         }
      }
   }


   // Pull constraint force elements out of lambda_star.
   unsigned int force_offset = 0;
   for (unsigned int jj = 0; jj < m_num_elements_LR[i]; jj++)
   {
      int m = m_LR[i][jj];

      for (unsigned int p = 0; p < m_num_constraints[m]; p++)
         m_lambda_c[m][p] = lambda_star[force_offset+p];

      force_offset += m_num_constraints[m];
   }

   free(lambda_star);
}


//----------------------------------------------------------------------------
//    Summary: adds a 'Hard' secondary joint to the closed articulation.
// Parameters: joint - pointer to the secondary joint
//    Returns: true if operation is successful
//             false if unsuccessful
//----------------------------------------------------------------------------
bool dmClosedArticulation::addHardSecondaryJoint(dmSecondaryJoint *joint)
{
   bool success = false;
   if (joint && find(m_sec_joints.begin(),
                     m_sec_joints.end(), joint) == m_sec_joints.end())
   {
      m_sec_joints.push_back(joint);
      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
//    Summary: adds a 'Soft' secondary joint to the closed articulation.
// Parameters: joint - pointer to the secondary joint
//    Returns: true if operation is successful
//             false if unsuccessful
//----------------------------------------------------------------------------
bool dmClosedArticulation::addSoftSecondaryJoint(dmSecondaryJoint *joint)
{
   bool success = false;
   if (joint && find(m_soft_joints.begin(),
                     m_soft_joints.end(), joint) == m_soft_joints.end())
   {
      m_soft_joints.push_back(joint);
      success = true;
   }

   return success;
}

//----------------------------------------------------------------------------
//    Summary: Called after the system is fully defined to initialize all
//             of the sets involved in simulating the kinematic loops.
// Parameters: none
//    Returns: set membership functions initialized
//----------------------------------------------------------------------------
void dmClosedArticulation::initKinematicLoopVars()
{
   unsigned int i, j, k, l;

   m_loop_root_index = new unsigned int[m_sec_joints.size()];
   m_num_constraints = new unsigned int[m_sec_joints.size()];

   m_num_elements_LB = new unsigned int[m_link_list.size()];
   m_LB = new unsigned int*[m_link_list.size()];

   m_num_elements_LR = new unsigned int[m_link_list.size()];
   m_LR = new unsigned int*[m_link_list.size()];

   for (i = 0; i < m_link_list.size(); i++)
   {
      m_num_elements_LB[i] = 0;
      m_LB[i] = NULL;

      m_num_elements_LR[i] = 0;
      m_LR[i] = NULL;
   }

   // Determine which links belong to loop k and also the root of k.
   for (k = 0; k < m_sec_joints.size(); k++)
   {
      unsigned int a = m_sec_joints[k]->getLinkAIndex();
      unsigned int b = m_sec_joints[k]->getLinkBIndex();

      // Determine links which belong to loop k.
      while (a != b)
      {
         if (a>b) // a isn't root of loop k, so add k to LB(a)
         {
            m_LB[a] = (unsigned int *)realloc(m_LB[a],
                                              (m_num_elements_LB[a]+1)*
                                              sizeof(unsigned int));
            m_LB[a][m_num_elements_LB[a]] = k;
            m_num_elements_LB[a] += 1;
            a = (m_link_list[a]->parent)->index;
         }
         else  // b isn't root of loop k, so add k to LB(b)
         {
            m_LB[b] =
               (unsigned int *)realloc(m_LB[b],
                                       (m_num_elements_LB[b]+1)*
                                       sizeof(unsigned int));
            m_LB[b][m_num_elements_LB[b]] = k;
            m_num_elements_LB[b] += 1;
            b = (m_link_list[b]->parent)->index;
         }
      }

      // At root of loop since a == b.  Add k to LR(a).
      m_loop_root_index[k] = a;
      m_LR[a] = (unsigned int *)realloc(
         m_LR[a], (m_num_elements_LR[a]+1)*sizeof(unsigned int));
      m_LR[a][m_num_elements_LR[a]] = k;
      m_num_elements_LR[a] += 1;
   }

   m_num_elements_LC = new unsigned int[m_sec_joints.size()];
   m_LC = new unsigned int*[m_sec_joints.size()];

   // Compute connected loops (LC(k)) for each secondary joint.
   for (k = 0; k < m_sec_joints.size(); k++)
   {
      // Initialize LC(k).
      m_num_elements_LC[k] = 0;
      m_LC[k] = NULL;

      for (i = 0; i < m_link_list.size(); i++)
      {
         unsigned int flag = 0;
         for (j = 0; j < m_num_elements_LB[i]; j++)
         {
            if (m_LB[i][j] == k)
               flag = 1;
         }

         if (flag == 1) // k is in LB(i)
         {
            // add elements of LB(i) to LC(k)
            for (j = 0; j < m_num_elements_LB[i]; j++)
            {
               // Test if element aleady in LC(k).
               unsigned int already_connected = 0;
               for (l = 0; l < m_num_elements_LC[k]; l++)
               {
                  if (m_LC[k][l] == m_LB[i][j])
                     already_connected = 1;
               }

               // Add to LC(k) if not already included and not k itself.
               if ((!already_connected) && (m_LB[i][j]!=k))
               {
                  m_LC[k] = (unsigned int *)realloc(m_LC[k],
                                                    (m_num_elements_LC[k]+1)
                                                    *sizeof(unsigned int));
                  m_LC[k][m_num_elements_LC[k]] = m_LB[i][j];
                  m_num_elements_LC[k] += 1;
               }
            }
         }
      }
   }

   // Compute LJ*(i) for each link.
   m_num_elements_LJ_star = new unsigned int[m_link_list.size()];
   m_LJ_star = new unsigned int*[m_link_list.size()];

   bool *LJ_work = new bool[m_sec_joints.size()];

   for (i = 0; i < m_link_list.size(); i++)
   {
      m_num_elements_LJ_star[i] = 0;
      m_LJ_star[i] = (unsigned int *)NULL;

      // Initialize membership of each loop in LJ*(i) = False.
      for (k = 0; k < m_sec_joints.size(); k++)
         LJ_work[k] = false;

      // Loop through all loops in LR(i).
      for (j = 0; j < m_num_elements_LR[i]; j++)
      {
         unsigned int loop_in_LR = m_LR[i][j];

         // Loop through loops connected to current element of LR.
         for (k = 0; k < m_num_elements_LC[loop_in_LR]; k++)
         {
            unsigned int connected_loop = m_LC[loop_in_LR][k];

            // Mark loops for membership in LJ*(i).
            if (m_loop_root_index[connected_loop] < i)
               LJ_work[connected_loop] = true;
         }
      }

      // Add marked loops to LJ*(i).
      unsigned int num = 0;
      for (k = 0; k < m_sec_joints.size(); k++)
      {
         if (LJ_work[k] == true)
            num++;
      }
      m_LJ_star[i] = new unsigned int[num];

      for (k = 0; k < m_sec_joints.size(); k++)
      {
         if (LJ_work[k] == true)
         {
            m_LJ_star[i][m_num_elements_LJ_star[i]] = k;
            m_num_elements_LJ_star[i] += 1;
         }
      }
   }

   delete [] LJ_work;


   // Allocate the Xik's. (2D array of matrices m_Xik[i][k][r][c])
   m_Xik = new Float***[m_link_list.size()];
   for (i = 0; i < m_link_list.size(); i++)
   {
      m_Xik[i] = new Float**[m_sec_joints.size()];

      for (k = 0; k < m_sec_joints.size(); k++)
      {
         // Only allocate the matrix if k is an element of LB(i) or LR(i).

         bool member_flag = false;
         for (j = 0; j < m_num_elements_LR[i]; j++)
            if (m_LR[i][j] == k)
               member_flag = true;

         for (j = 0; j < m_num_elements_LB[i]; j++)
            if (m_LB[i][j] == k)
               member_flag = true;

         if (member_flag == false)
            m_Xik[i][k] = NULL;
         else
         {
            // Matrix has 6 rows.
            m_Xik[i][k] = new Float*[6];

            unsigned int r, columns;
            for (r = 0; r < 6; r++)
            {
               columns = 6 - m_sec_joints[k]->getNumDOFs();
               m_Xik[i][k][r] = new Float[columns];
            }
         }
      }
   }

   // Fill in the number of constrained DOF's for each loop k.
   for (k = 0; k < m_sec_joints.size(); k++)
      m_num_constraints[k] = 6 - m_sec_joints[k]->getNumDOFs();

   // Allocate the Zeta_k's.
   m_zetak = new Float*[m_sec_joints.size()];
   for (k = 0; k < m_sec_joints.size(); k++)
      m_zetak[k] = new Float[m_num_constraints[k]];

   // Allocate the Bmn's. (2D array of matrices m_Bmn[m][n][r][c])
   m_Bmn = new Float***[m_sec_joints.size()];
   for (unsigned int m = 0; m < m_sec_joints.size(); m++)
   {
      m_Bmn[m] = new Float**[m_sec_joints.size()];

      // Initialize all Bmn's to null pointers.
      for (unsigned int n = 0; n < m_sec_joints.size(); n++)
         m_Bmn[m][n] = (Float **)NULL;

      // Allocate Bmn's which are actually used (connected loops).
      for (unsigned int i = 0; i < m_num_elements_LC[m]; i++)
      {
         unsigned int n = m_LC[m][i];

         m_Bmn[m][n] = new Float*[m_num_constraints[m]];

         for (unsigned int r = 0; r < m_num_constraints[m]; r++)
            m_Bmn[m][n][r] = new Float[m_num_constraints[n]];
      }

      // Allocate Bmm.
      m_Bmn[m][m] = new Float*[m_num_constraints[m]];
      for (unsigned int r = 0; r < m_num_constraints[m]; r++)
         m_Bmn[m][m][r] = new Float[m_num_constraints[m]];
   }

   // Compute how many constraint force elements are resolved at each link.
   m_constraints_at_root = new unsigned int[m_link_list.size()];
   for (i = 0; i < m_link_list.size(); i++)
   {
      m_constraints_at_root[i] = 0;
      for (unsigned int j = 0; j < m_num_elements_LR[i]; j++)
         m_constraints_at_root[i] += m_num_constraints[m_LR[i][j]];
   }

   // Allocate the m_Binv_zeta's.
   m_Binv_zetai = new Float*[m_link_list.size()];
   for (i = 0; i < m_link_list.size(); i++)
      m_Binv_zetai[i] = new Float[m_constraints_at_root[i]];

   // Allocate the m_Binv_Xi's.
   m_Binv_Xi = new Float**[m_link_list.size()];
   for (i = 0; i < m_link_list.size(); i++)
   {
      m_Binv_Xi[i] = new Float*[m_constraints_at_root[i]];

      for (unsigned int j = 0; j < m_constraints_at_root[i]; j++)
         m_Binv_Xi[i][j] = new Float[6];
   }

   // Allocate the m_Binv_Bim's.
   m_Binv_Bim = new Float***[m_link_list.size()];
   for (i = 0; i < m_link_list.size(); i++)
   {
      m_Binv_Bim[i] = new Float**[m_sec_joints.size()];
      for (unsigned int j = 0; j < m_num_elements_LJ_star[i]; j++)
      {
         unsigned int m = m_LJ_star[i][j];

         m_Binv_Bim[i][m] = new Float*[m_constraints_at_root[i]];

         for (unsigned int r = 0; r < m_constraints_at_root[i]; r++)
            m_Binv_Bim[i][m][r] = new Float[m_num_constraints[m]];
      }
   }

   // Allocate storage for the constraint forces.
   m_lambda_c = new Float*[m_sec_joints.size()];
   for (i = 0; i < m_sec_joints.size(); i++)
      m_lambda_c[i] = new Float[m_num_constraints[i]];

#if 0
   for (i = 0; i < m_link_list.size(); i++)
   {
      cerr << "\nLB(" << i << "): ";
      for (unsigned int j = 0; j < m_num_elements_LB[i]; j++)
         cerr << m_LB[i][j] << " ";
   }
   cerr << endl << endl;

   for (i = 0; i < m_link_list.size(); i++)
   {
      cerr << "\nLR(" << i << "): ";
      for (unsigned int j = 0; j < m_num_elements_LR[i]; j++)
         cerr << m_LR[i][j] << " ";
   }
   cerr << endl << endl;

   for (k = 0; k < m_sec_joints.size(); k++)
   {
      cerr << "\nLC(" << k << "): ";
      for (unsigned int l = 0; l < m_num_elements_LC[k]; l++)
         cerr << m_LC[k][l] << " ";
   }
   cerr << endl << endl;

   for (i = 0; i < m_link_list.size(); i++)
   {
      cerr << "\nLJ*(" << i << "): ";
      for (unsigned int k = 0; k < m_num_elements_LJ_star[i]; k++)
         cerr << m_LJ_star[i][k] << " ";
   }
   cerr << endl << endl;

   for (k = 0; k < m_sec_joints.size(); k++)
   {
      cerr << "\nRoot(" << k << ") = " << m_loop_root_index[k];
   }
   cerr << endl << endl;
#endif
}


//----------------------------------------------------------------------------
//    Summary: Function called during backward dynamics recursion of AB
//             algorithm at the given root link.  The function eliminates the
//             constraint forces of all loops rooted at the given link from
//             the remaining force-balance and loop-closure constraint
//             equations.
// Parameters: i - link index of root link
//    Returns: none
//----------------------------------------------------------------------------
void dmClosedArticulation::eliminateLoops(unsigned int i)
{
   unsigned int j;
   Float **X_star = (Float **)malloc(6*sizeof(Float *));
   for (j = 0; j < 6; j++)
      X_star[j] = (Float *)malloc(m_constraints_at_root[i]
                                  *sizeof(Float));

   Float *zeta_star = (Float *)malloc(m_constraints_at_root[i]
                                      *sizeof(Float));

   Float ***Bim_star = (Float ***)malloc(m_sec_joints.size()*
                                         sizeof(Float **));
   for (j = 0; j < m_num_elements_LJ_star[i]; j++)
   {
      unsigned int m = m_LJ_star[i][j];

      Bim_star[m] = (Float **)malloc(m_constraints_at_root[i]
                                     *sizeof(Float *));

      for (unsigned int n = 0; n < m_constraints_at_root[i]; n++)
         Bim_star[m][n] = (Float *)calloc(m_num_constraints[m],
                                          sizeof(Float));
   }

   unsigned int offset = 0;
   for (j = 0; j < m_num_elements_LR[i]; j++)
   {
      // Set up X*(i).
      for (unsigned int l = 0; l < 6; l++)
      {
         for (unsigned int k = 0; k < m_num_constraints[m_LR[i][j]]; k++)
            X_star[l][k+offset] = m_Xik[i][m_LR[i][j]][l][k];
      }

      // Set up zeta*(i).
      for (unsigned int k = 0; k < m_num_constraints[m_LR[i][j]]; k++)
      {
         zeta_star[k+offset] = m_zetak[m_LR[i][j]][k];
      }

      // Set up B*im.
      for (unsigned int n = 0; n < m_num_elements_LC[m_LR[i][j]]; n++)
      {
         unsigned int m = m_LC[m_LR[i][j]][n];

         if (m_loop_root_index[m] < i)
            for (unsigned int r = 0; r < m_num_constraints[m_LR[i][j]]; r++)
               for (unsigned int c = 0; c < m_num_constraints[m]; c++)
                  Bim_star[m][r+offset][c] = m_Bmn[m_LR[i][j]][m][r][c];
      }

      offset += m_num_constraints[m_LR[i][j]];
   }


   // ******************* Linpack SVD ********************************

   // Allocate a 1D array for B*(i) (to interface with Fortran SVD).
   // SVD routines come in single and double single precision, so use can
   // use the 'Float' typedef, which could be defined as float or double.
   // SVD routine will destroy contents of Bi_star.
   Float *Bi_star = (Float *)calloc(m_constraints_at_root[i]*
                                    m_constraints_at_root[i],
                                    sizeof(Float));

   unsigned int row_offset = 0;
   for (j = 0; j < m_num_elements_LR[i]; j++)
   {
      unsigned int k = m_LR[i][j];
      unsigned int col_offset = 0;

      for (unsigned int p = 0; p < m_num_elements_LR[i]; p++)
      {
         unsigned int m = m_LR[i][p];

         if (m_Bmn[k][m] != NULL) // k and m connected
         {
            // Copy Bkm into B*(i).
            for (unsigned int r = 0; r < m_num_constraints[k]; r++)
               for (unsigned int c = 0; c < m_num_constraints[m]; c++)
               {
                  unsigned int index = ((r+row_offset) +
                                        m_constraints_at_root[i]*
                                        (c+col_offset));
                  Bi_star[index] = m_Bmn[k][m][r][c];
               }
         }

         col_offset += m_num_constraints[m];
      }

      row_offset += m_num_constraints[k];
   }

   Float *u;           /* svd decomp u */
   Float *v;           /* svd decomp v */
   Float *s;           /* array of singular values (descending order)*/
   Float *e;           /* don't worry about */
   Float *work;        /* scratch array used by ssvdc. */
   long int n;         /* Matrix dimensions = nxn. */
   long int info;      /* Should equal zero upon return from ssvdc. */
   long int job = 11;

   n = (long int) m_constraints_at_root[i];  // HACK cast
   u = (Float *)malloc(n*n*sizeof(Float));
   v = (Float *)malloc(n*n*sizeof(Float));
   s = (Float *)malloc(n*sizeof(Float));
   e = (Float *)malloc(n*sizeof(Float));
   work = (Float *)malloc(n*sizeof(Float));

   if ((Bi_star == NULL) || (u == NULL) || (v == NULL) ||
       (s == NULL) || (e == NULL) || (work == NULL))
      cout << "SVD ALLOCATION ERROR" << endl;

#ifdef DM_DOUBLE_PRECISION
   dsvdc_(Bi_star, &n, &n, &n, s, e, u, &n, v, &n, work, &job, &info);
#else
   ssvdc_(Bi_star, &n, &n, &n, s, e, u, &n, v, &n, work, &job, &info);
#endif

   if (info != 0)
   {
      cout << "\n\nEROOR:  Singular Value Decomposition failed." << endl;
      exit(-1);
   }

   // ********************** End Linpack SVD ***************************

#if 0
   //  cout << "\ninfo: " << info << endl;

   cout << "\nU:";
   for (unsigned int rr = 0; rr < n; rr++)
   {
      cout << endl;
      for (unsigned int c = 0; c < n; c++)
         cout << "  " << u[rr+c*n];
   }
   cout << endl;

   cout << "\ns:";
   for (rr = 0; rr < n; rr++)
      cout << "  " << s[rr];
   cout << endl;

   cout << "\nV:";
   for (rr = 0; rr < n; rr++)
   {
      cout << endl;
      for (unsigned int c = 0; c < n; c++)
         cout << "  " << v[rr+c*n];
   }
   cout << endl;
#endif



   // Invert singular value diagonal matrix,
   // zeroing infinite elements.
   float max_singular_value = s[0];
   s[0] = 1.0/s[0];
   for (j = 1; j < (unsigned int)n; j++)  // HACK (cast)
   {
      if (50000*s[j] < max_singular_value)
         s[j] = 0;
      else
         s[j] = 1.0/s[j];
   }

   //  Compute inverse(B*(i)).
   Float **inv_Bi_star = (Float **)malloc(m_constraints_at_root[i]*
                                          sizeof(Float *));
   for (j = 0; j < m_constraints_at_root[i]; j++)
      inv_Bi_star[j] = (Float *)malloc(m_constraints_at_root[i]*
                                       sizeof(Float));

   unsigned int r;
   for (r = 0; r < m_constraints_at_root[i]; r++)
      for (unsigned int c = 0; c < m_constraints_at_root[i]; c++)
      {
         inv_Bi_star[r][c] = 0;

         for (j = 0; j < m_constraints_at_root[i]; j++)
            inv_Bi_star[r][c] += (v[r+m_constraints_at_root[i]*j]*s[j]*
                                  u[c+m_constraints_at_root[i]*j]);
      }


   // Compute inv(B*(i)) x (X*(i))^T
   for (r = 0; r < m_constraints_at_root[i]; r++)
      for (unsigned int c = 0; c < 6; c++)
      {
         m_Binv_Xi[i][r][c] = 0;
         for (j = 0; j < m_constraints_at_root[i]; j++)
            m_Binv_Xi[i][r][c] += inv_Bi_star[r][j]*X_star[c][j];
      }

   // *******************************
   // Compute/Add in reflected inertia due to the loop.
   for (r = 0; r < 6; r++)
      for (unsigned int c = 0; c < 6; c++)
         for (j = 0; j < m_constraints_at_root[i]; j++)
            (m_link_list[i]->I_refl)[r][c] += X_star[r][j]*m_Binv_Xi[i][j][c];

   // Compute inv(B*(i)) x zeta*(i)
   for (r = 0; r < m_constraints_at_root[i]; r++)
   {
      m_Binv_zetai[i][r] = 0;

      for (j = 0; j < m_constraints_at_root[i]; j++)
         m_Binv_zetai[i][r] += inv_Bi_star[r][j]*zeta_star[j];
   }

   // ******************************************************************
   // Constraint stabilization with springs and dampers.
   // Add known stabilization forces to lambda_star in inv(B*(i)) x zeta*(i)

   unsigned int force_offset = 0;
   for (j = 0; j < m_num_elements_LR[i]; j++)
   {
      unsigned int k = m_LR[i][j];

      if (m_sec_joints[k]->getStabilizationType() ==
          dmSecondaryJoint::SPRING_DAMPER)
      {
         // Get spring/damper stabilization wrench for secondary joint k.
         Float lambda_c[6];
         m_sec_joints[k]->computeStabilizationForce(lambda_c);

         for (unsigned int p = 0; p < m_num_constraints[k]; p++)
            m_Binv_zetai[i][force_offset+p] += lambda_c[p];
      }

      force_offset += m_num_constraints[k];
   }
   // ******************************************************************

   // Modify bias force.
   for (r = 0; r < 6; r++)
      for (j = 0; j < m_constraints_at_root[i]; j++)
         (m_link_list[i]->f_star)[r] += X_star[r][j]*m_Binv_zetai[i][j];

   //  Compute changes to Xim's for m in LJ*(i).
   for (j = 0; j < m_num_elements_LJ_star[i]; j++)
   {
      unsigned int m = m_LJ_star[i][j];

      // Compute inv(B*(i)) x B*im
      for (r = 0; r < m_constraints_at_root[i]; r++)
         for (unsigned int c = 0; c < m_num_constraints[m]; c++)
         {
            m_Binv_Bim[i][m][r][c] = 0;

            for (unsigned int p = 0; p < m_constraints_at_root[i]; p++)
               m_Binv_Bim[i][m][r][c] += (inv_Bi_star[r][p]*
                                          Bim_star[m][p][c]);
         }

      for (r = 0; r < 6; r++)
         for (unsigned int c = 0; c < m_num_constraints[m]; c++)
            for (unsigned int p = 0; p < m_constraints_at_root[i]; p++)
               m_Xik[i][m][r][c] -= X_star[r][p]*m_Binv_Bim[i][m][p][c];
   }

   //  Compute changes to Bmn's in LJ*(i).
   for (j = 0; j < m_num_elements_LJ_star[i]; j++)
   {
      unsigned int m = m_LJ_star[i][j];

      for (unsigned int p = 0; p < m_num_elements_LJ_star[i]; p++)
      {
         unsigned int n = m_LJ_star[i][p];

         for (r = 0; r < m_num_constraints[m]; r++)
            for (unsigned int c = 0; c < m_num_constraints[n]; c++)
               for (unsigned int q = 0; q < m_constraints_at_root[i]; q++)
                  m_Bmn[m][n][r][c] -= (Bim_star[m][q][r]*
                                        m_Binv_Bim[i][n][q][c]);
      }
   }

   //  Compute changes to m_zetak's in  LJ*(i).
   for (j = 0; j < m_num_elements_LJ_star[i]; j++)
   {
      unsigned int m = m_LJ_star[i][j];

      for (r = 0; r < m_num_constraints[m]; r++)
         for (unsigned int p = 0; p < m_constraints_at_root[i]; p++)
            m_zetak[m][r] -=  Bim_star[m][p][r]*m_Binv_zetai[i][p];
   }


   // **********************************************************

   // Free temporary variables.
   for (j = 0; j < 6; j++)
      free(X_star[j]);
   free(X_star);

   for (j = 0; j < m_constraints_at_root[i]; j++)
   {
      free(inv_Bi_star[j]);
   }
   free(inv_Bi_star);

   for (j = 0; j < m_num_elements_LJ_star[i]; j++)
   {
      unsigned int m = m_LJ_star[i][j];

      for (unsigned int n = 0; n < m_constraints_at_root[i]; n++)
         free(Bim_star[m][n]);

      free(Bim_star[m]);
   }
   free(Bim_star);

   free(zeta_star);
   free(Bi_star);
   free(u);
   free(v);
   free(s);
   free(e);
   free(work);
}


//----------------------------------------------------------------------------
//    Summary: Called during backward dynamics recursion of the AB
//             algorithm to propagate secondary joint constraints to the
//             parent link for a link that is not a root link (or whose
//             rooted loops have already been eliminated).
// Parameters: i - link index of link being eliminated from constraint eqs.
//    Returns: none
//----------------------------------------------------------------------------
void dmClosedArticulation::propagateConstraints(unsigned int i)
{
   unsigned int j;
   for (j = 0; j < m_num_elements_LB[i]; j++)
   {
      unsigned int k = m_LB[i][j];

      // get the parent's index
      unsigned int prev = m_link_list[i]->parent->index;

      // Update the coefficient matrix of loop k's constraint forces in the
      // parent link's force-balance equation due to the elimination of the
      // current link.  This matrix also appears in the loop-closure
      // constraint equations.

      if (prev != m_loop_root_index[k])
      {
         // Set Xik directly for parent link.
         m_link_list[i]->link->XikToInboard(m_Xik[i][k], m_Xik[prev][k],
                                            m_num_constraints[k]);
      }
      else
      {
         // Must accumulate Xik for parent link (loop root).
         // Note that m_Xik was set to zero in ABForwardKinematics
         // for the loop root.
         Float **tmp;
         unsigned int r;

         tmp = (Float **)malloc(6*sizeof(Float *));
         for (r = 0; r < 6; r++)
            tmp[r] = (Float *)malloc(m_num_constraints[k]*sizeof(Float));

         m_link_list[i]->link->XikToInboard(m_Xik[i][k], tmp,
                                            m_num_constraints[k]);

         for (r = 0; r < 6; r++)
         {
            for (unsigned int c = 0; c < m_num_constraints[k]; c++)
               m_Xik[prev][k][r][c] += tmp[r][c];

            free(tmp[r]);
         }
         free(tmp);
      }
   }

   for (j = 0; j < m_num_elements_LB[i]; j++)
   {
      unsigned int k = m_LB[i][j];

      for (unsigned int l = 0; l < m_num_elements_LB[i]; l++)
      {
         unsigned int n = m_LB[i][l];

         // Update coefficient matrix of loop n's constraint forces in loop
         // k's constraint equation due to elimination of link i.
         m_link_list[i]->link->BToInboard(m_Bmn[k][n],
                                          m_Xik[i][k], m_num_constraints[k],
                                          m_Xik[i][n], m_num_constraints[n]);
      }

      // Update bias term in constraint equations for loop k due to
      // elimination of link i.
      m_link_list[i]->link->xformZetak(m_zetak[k],
                                       m_Xik[i][k], m_num_constraints[k]);
   }
}
