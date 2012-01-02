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
 *     File: dmSecondaryJoint.cpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Abstract base class definition for secondary joints.
 *****************************************************************************/

#include "dmSecondaryJoint.hpp"

#include "dmSecondaryForce.hpp"
#include "dmMobileBaseLink.hpp"
#include "dmPrismaticLink.hpp"
#include "dmRevoluteLink.hpp"
#include "dmSphericalLink.hpp"
#include "dmQuaternionLink.hpp"
#include "dmStaticRootLink.hpp"

#include <typeinfo>

//============================================================================
// class dmSecondaryJoint : public dmObject
//============================================================================


//----------------------------------------------------------------------------
//    Summary: default class constructor
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
dmSecondaryJoint::dmSecondaryJoint()
      : m_link_A_index(-1),
        m_link_B_index(-1),
        m_articulation(NULL),
        m_stabilization(NONE),
        m_joint_friction(0.0)
{
}

//----------------------------------------------------------------------------
//    Summary: Initializes the secondary joint with a pointer to the
//             articulation which it will be added to.  Note that this
//             addition to the articulation is done through a call to
//             dmClosedArticulation::addHardSecondaryJoint() or
//             dmClosedArticulation::addSoftSecondaryJoint().
// Parameters: art - the articulation to which the secondary joint is added
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryJoint::setArticulation(dmArticulation *art)
{
   m_articulation = art;
}


//----------------------------------------------------------------------------
//    Summary: Initializes the secondary joint by providing a pointer
//             to the A link of the joint.  Also, adds a force widget to
//             the provided link for transmitting known joint forces.
//             Known joint forces include applied joint inputs and frictional
//             forces along the free modes of the joint.  Known forces
//             also include spring/damper forces in the constrained
//             directions when using compliant (soft) secondary joints.
// Parameters: link - pointer to the A link of the joint
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryJoint::setLinkA(dmLink *link)
{
   m_link_A_index = m_articulation->getLinkIndex(link);

   // Create the force widget for link A.
   dmSecondaryForce *forceA = new dmSecondaryForce(dmSecondaryForce::LINK_A,
                                                   this);

   if (typeid(*link) == typeid(dmMobileBaseLink))
      ((dmMobileBaseLink *)link)->addForce(forceA);
   else if (typeid(*link) == typeid(dmPrismaticLink))
      ((dmPrismaticLink *)link)->addForce(forceA);
   else if (typeid(*link) == typeid(dmRevoluteLink))
      ((dmRevoluteLink *)link)->addForce(forceA);
   else if (typeid(*link) == typeid(dmSphericalLink))
      ((dmSphericalLink *)link)->addForce(forceA);
   else if (typeid(*link) == typeid(dmQuaternionLink))
      ((dmQuaternionLink *)link)->addForce(forceA);
   else if (typeid(*link) == typeid(dmStaticRootLink))
   {
      // Do nothing.  Forces applied to static link won't affect it.
   }
   else
   {
      cerr << "Can't attach secondary joint to link of type "
           << (typeid(*link)).name() << " ." << endl;
      exit(3);
   }
}

//----------------------------------------------------------------------------
//    Summary: Initializes the secondary joint by providing a pointer
//             to the B link of the joint.  Also, adds a force widget to
//             the provided link for transmitting known joint forces.
//             Known joint forces include applied joint inputs and frictional
//             forces along the free modes of the joint.  Known forces
//             also include spring/damper forces in the constrained
//             directions when using compliant (soft) secondary joints.
// Parameters: link - pointer to the B link of the joint
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryJoint::setLinkB(dmLink *link)
{
   m_link_B_index = m_articulation->getLinkIndex(link);

   // Create the force widget for link B.
   dmSecondaryForce *forceB = new dmSecondaryForce(dmSecondaryForce::LINK_B,
                                                   this);

   if (typeid(*link) == typeid(dmMobileBaseLink))
      ((dmMobileBaseLink *)link)->addForce(forceB);
   else if (typeid(*link) == typeid(dmPrismaticLink))
      ((dmPrismaticLink *)link)->addForce(forceB);
   else if (typeid(*link) == typeid(dmRevoluteLink))
      ((dmRevoluteLink *)link)->addForce(forceB);
   else if (typeid(*link) == typeid(dmSphericalLink))
      ((dmSphericalLink *)link)->addForce(forceB);
   else if (typeid(*link) == typeid(dmQuaternionLink))
      ((dmQuaternionLink *)link)->addForce(forceB);
   else if (typeid(*link) == typeid(dmStaticRootLink))
   {
      // Do nothing.  Forces applied to static link won't affect it.
   }
   else
   {
      cerr << "Can't attach secondary joint to link of type "
           << (typeid(*link)).name() << " ." << endl;
      exit(3);
   }
}


//----------------------------------------------------------------------------
//    Summary: Initializes the secondary joint by providing position and
//             orientation of the outboard coordinate systems of the
//             connected links relative to the link coordinate systems of
//             the connected links.
// Parameters: pos_a - position of link A outboard c.s.(oa), {^a}p_{oa}
//             pos_b - position of link B outboard c.s.(k) , {^b}p_{k}
//             rot_a - orientation of link A outboard c.s.(oa), {^a}R_{oa}
//             rot_b - orientation of link B outboard c.s.(k) , {^b}R_{k}
//    Returns: none.
//----------------------------------------------------------------------------
void dmSecondaryJoint::setKinematics(const CartesianVector pos_a,
                                     const CartesianVector pos_b,
                                     const RotationMatrix rot_a,
                                     const RotationMatrix rot_b)
{
   int i,j;

   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         m_a_R_oa[i][j] = rot_a[i][j];
         m_b_R_k[i][j] = rot_b[i][j];
      }
      m_a_p_oa[i] = pos_a[i];
      m_b_p_k[i] = pos_b[i];
   }
}

//----------------------------------------------------------------------------
//    Summary: Computes basic secondary joint state-type information common
//             to all secondary joints as a function of the state of
//             the connected links.
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmSecondaryJoint::computeState()
{
#ifdef DEBUG
   cout << "dmSecondaryJoint::computeState() enter\n" << flush;
#endif

   const dmABForKinStruct *kinStructA;
   const dmABForKinStruct *kinStructB;

   kinStructA = m_articulation->getForKinStruct(m_link_A_index);
   kinStructB = m_articulation->getForKinStruct(m_link_B_index);

   register int i, j;


   // Compute m_oa_R_k, to specify orientation across secondary joint.

   RotationMatrix w_R_k;   //  w_R_k = w_R_b * m_b_R_k
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
      {
         w_R_k[i][j] = (kinStructB->R_ICS[i][0]*m_b_R_k[0][j] +
                        kinStructB->R_ICS[i][1]*m_b_R_k[1][j] +
                        kinStructB->R_ICS[i][2]*m_b_R_k[2][j]);
      }

   //  m_a_R_k = (w_R_a)^T * w_R_k
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
      {
         m_a_R_k[i][j] = (kinStructA->R_ICS[0][i]*w_R_k[0][j] +
                          kinStructA->R_ICS[1][i]*w_R_k[1][j] +
                          kinStructA->R_ICS[2][i]*w_R_k[2][j]);
      }

   // m_oa_R_k = (m_a_R_oa)^T * m_a_R_k
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         m_oa_R_k[i][j] = (m_a_R_oa[0][i]*m_a_R_k[0][j] +
                           m_a_R_oa[1][i]*m_a_R_k[1][j] +
                           m_a_R_oa[2][i]*m_a_R_k[2][j]);


   // Compute m_k_w_rel, to specify angular velocity across secondary joint.


   CartesianVector k_w_k;    //  k_w_k = k_R_b * b_w_b
   //  m_oa_w_oa = oa_R_a * a_w_a
   for (i = 0; i < 3; i++)
   {
      k_w_k[i] = (m_b_R_k[0][i]*kinStructB->v[0] +
                  m_b_R_k[1][i]*kinStructB->v[1] +
                  m_b_R_k[2][i]*kinStructB->v[2]);

      m_oa_w_oa[i] = (m_a_R_oa[0][i]*kinStructA->v[0] +
                      m_a_R_oa[1][i]*kinStructA->v[1] +
                      m_a_R_oa[2][i]*kinStructA->v[2]);
   }

   //  m_k_w_oa = k_R_oa * m_oa_w_oa
   //  m_k_w_rel = k_w_k - m_k_w_oa
   for (i = 0; i < 3; i++)
   {
      m_k_w_oa[i] = (m_oa_R_k[0][i]*m_oa_w_oa[0] +
                     m_oa_R_k[1][i]*m_oa_w_oa[1] +
                     m_oa_R_k[2][i]*m_oa_w_oa[2]);

      m_k_w_rel[i] = k_w_k[i] - m_k_w_oa[i];
   }


   // Compute position variables across secondary joint.

   CartesianVector w_p_k;   // w_p_k  = w_R_b * m_b_p_k + w_p_b
   CartesianVector w_p_oa;  // w_p_oa = w_R_a * m_a_p_oa + w_p_a
   CartesianVector w_p_rel; // w_p_rel = w_p_k - w_p_oa

   for (i = 0; i < 3; i++)
   {
      w_p_k[i] = (kinStructB->R_ICS[i][0]*m_b_p_k[0] +
                  kinStructB->R_ICS[i][1]*m_b_p_k[1] +
                  kinStructB->R_ICS[i][2]*m_b_p_k[2] +
                  kinStructB->p_ICS[i]);

      w_p_oa[i] = (kinStructA->R_ICS[i][0]*m_a_p_oa[0] +
                   kinStructA->R_ICS[i][1]*m_a_p_oa[1] +
                   kinStructA->R_ICS[i][2]*m_a_p_oa[2] +
                   kinStructA->p_ICS[i]);

      w_p_rel[i] = w_p_k[i] - w_p_oa[i];
   }

   CartesianVector a_p_rel;  // = (w_R_a)^T * w_p_rel
   for (i = 0; i < 3; i++)
   {
      a_p_rel[i] = (kinStructA->R_ICS[0][i]*w_p_rel[0] +
                    kinStructA->R_ICS[1][i]*w_p_rel[1] +
                    kinStructA->R_ICS[2][i]*w_p_rel[2]);
      m_a_p_k[i] = m_a_p_oa[i] + a_p_rel[i];
   }

   // m_d = (m_a_R_oa)^T * a_p_rel
   for (i = 0; i < 3; i++)
   {
      m_d[i] = (m_a_R_oa[0][i]*a_p_rel[0] +
                m_a_R_oa[1][i]*a_p_rel[1] +
                m_a_R_oa[2][i]*a_p_rel[2]);
   }


   // Compute derivatives of secondary joint position variables.

   CartesianVector b_v_k;    // b_v_k = b_v_b + b_w_b x m_b_p_k
   CartesianVector k_v_k;    // k_v_k = k_R_b * b_v_k
   CartesianVector oa_v_k;   // oa_v_k = m_oa_R_k * k_v_k
   CartesianVector a_v_oa;   // a_v_oa = a_v_a + a_w_a x m_a_p_oa
   CartesianVector oa_v_oa;  // oa_v_oa = oa_R_a * a_v_oa
   CartesianVector bias;     // bias = m_oa_w_oa x d

   CartesianVector tmpCross;
   crossproduct(kinStructB->v, m_b_p_k, tmpCross);
   for (i = 0; i < 3; i++)
      b_v_k[i] = kinStructB->v[i+3] + tmpCross[i];

   for (i = 0; i < 3; i++)
   {
      k_v_k[i] = (m_b_R_k[0][i]*b_v_k[0] +
                  m_b_R_k[1][i]*b_v_k[1] +
                  m_b_R_k[2][i]*b_v_k[2]);
   }

   for (i = 0; i < 3; i++)
   {
      oa_v_k[i] = (m_oa_R_k[i][0]*k_v_k[0] +
                   m_oa_R_k[i][1]*k_v_k[1] +
                   m_oa_R_k[i][2]*k_v_k[2]);
   }

   crossproduct(kinStructA->v, m_a_p_oa, tmpCross);
   for (i = 0; i < 3; i++)
      a_v_oa[i] = kinStructA->v[i+3] + tmpCross[i];

   for (i = 0; i < 3; i++)
   {
      oa_v_oa[i] = (m_a_R_oa[0][i]*a_v_oa[0] +
                    m_a_R_oa[1][i]*a_v_oa[1] +
                    m_a_R_oa[2][i]*a_v_oa[2]);
   }

   // m_d_dot = oa_v_k - oa_v_oa  - bias
   crossproduct(m_oa_w_oa, m_d, bias);
   for (i = 0; i < 3; i++)
      m_d_dot[i] = oa_v_k[i] - oa_v_oa[i] - bias[i];
}


//----------------------------------------------------------------------------
//    Summary: Computes the known forces applied to the A link of the joint.
// Parameters: none
//    Returns: force - the known force applied through the secondary
//                     joint to the A link expressed w.r.t. A link
//                     coordinates. See setLinkA() for meaning of 'known'.
//----------------------------------------------------------------------------
void dmSecondaryJoint::getAppliedAForce(SpatialVector force)
{
   register int i;
   CartesianVector r_cross_f;

   // Convert from wrench exerted on Link B at K wrt. K coordinates
   // into wrench exerted onto link A's coordinate frame.
   for (i = 0; i < 3; i++)
   {
      force[i] = -(m_a_R_k[i][0]*m_k_f_k[0] +
                   m_a_R_k[i][1]*m_k_f_k[1] +
                   m_a_R_k[i][2]*m_k_f_k[2]);

      force[i+3] = -(m_a_R_k[i][0]*m_k_f_k[3] +
                     m_a_R_k[i][1]*m_k_f_k[4] +
                     m_a_R_k[i][2]*m_k_f_k[5]);
   }

   crossproduct(m_a_p_k, &force[3], r_cross_f);
   force[0] += r_cross_f[0];
   force[1] += r_cross_f[1];
   force[2] += r_cross_f[2];
}


//----------------------------------------------------------------------------
//    Summary: Computes the known forces applied to the B link of the joint.
// Parameters: none
//    Returns: force - the known force applied through the secondary
//                     joint to the B link expressed w.r.t. B link
//                     coordinates. See setLinkB() for meaning of 'known'.
//----------------------------------------------------------------------------
void dmSecondaryJoint::getAppliedBForce(SpatialVector force)
{
   register int i;
   CartesianVector r_cross_f;

   // Convert from wrench exerted on Link B at K wrt. K coordinates
   // into wrench exerted onto link B's coordinate frame.
   for (i = 0; i < 3; i++)
   {
      force[i] = (m_b_R_k[i][0]*m_k_f_k[0] +
                  m_b_R_k[i][1]*m_k_f_k[1] +
                  m_b_R_k[i][2]*m_k_f_k[2]);

      force[i+3] = (m_b_R_k[i][0]*m_k_f_k[3] +
                    m_b_R_k[i][1]*m_k_f_k[4] +
                    m_b_R_k[i][2]*m_k_f_k[5]);
   }

   crossproduct(m_b_p_k, &force[3], r_cross_f);
   force[0] += r_cross_f[0];
   force[1] += r_cross_f[1];
   force[2] += r_cross_f[2];
}
