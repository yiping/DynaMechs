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
 *     File: dmArticulation.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for general articulated structures
 *****************************************************************************/

#ifndef _DM_ARTICULATION_HPP
#define _DM_ARTICULATION_HPP

#include "dm.h"
#include "dmLink.hpp"
#include "dmSystem.hpp"

#include <vector>

//============================================================================
/**

The dmArticulation class would be better named the \b dmTreeStructure
class.  It is a subclass of the dmSystem class that implements the
functionality needed to model a multibody system organized in a tree topology
(Note: a subclass of  dmArticulation  extends this class to support
closed-chain structures).  This class contains a database of links and the
predecessor/successor relationships between links that define the tree
topology.  As such, the first link and any other link added to the articulation
using the \b addLink function with the second argument set to NULL are
considered that root of the tree.  Then, as additional links are added to the
articulation, a pointer to a parent link (already added to the articulation) is
used to specify its location in the tree.  As a result, each link in a tree has
only one parent.  The only exceptions are the root links that are defined
relative to the  dmSystem 's inertial reference coordinate system.

The default constructor returns an empty articulation (that is, with no
links).  As links are added to the articulation, internal variables are
allocated that will be used to perform the dynamic computations.  When the
destructor is called these internal variables are freed but any links that
have been added to the articulation are not.

The \b addLink function is called to add a link to the tree structure that
is maintained by the dmArticulation objects.  The first parameter is a
pointer to a (previously instantiated) link object that is to be added, and the
second parameter is a pointer to a link that has already been added to the
articulation and will be the "parent" to the new link and placed in the
database accordingly.  Note that the first link added has no parent so the \b
parent parameter is set to \b NULL.  If the operation is successful, this
function returns \b true; otherwise it will return \b false for any one
of a number of reasons (parent not found, variable allocation failure, etc.)

The articulation can be queried for the number of links in the tree by calling
\b getNumLinks.  A pointer to a specific link can be obtained by calling
\b getLink with the appropriate index.  The index corresponds to the order
in which the links were added to the tree: 0 for the first link,
1 for the second, etc.  If the index is out of range, \b getLink returns
\b NULL.  The inverse function, \b getLinkIndex takes a  dmLink
pointer and returns its corresponding index, or \b -1 if the link is not
contained in the tree or the pointer is \b NULL.

Three functions, \b setJointInput, \b setState, and \b getState, are
used to set and query joint states and inputs for the links in the entire
tree.  The first, \b setJointInput, sets the joint inputs (either torques,
forces, or motor voltages) for all the links in the articulation.  Its \b
joint_input argument is a packed (one-dimensional) array with length equal
to the total number of DOFs in the articulation.  The elements of this array
correspond to the DOFs of each link in the order they were added to the
articulation. \b setState sets the state of the DOFs in the articulation.
The two arguments are both packed arrays (like \b joint_input) containing
the joint positions and velocities of the links.  The reverse operation, \b
getState, fills packed arrays with the joint positions velocities.  A
convenience function, \b getNumDOFs, returns the total number of degrees
of freedom in the articulation and can be used to determine the appropriate
size of the above arrays.

Two functions are provided to compute the forward kinematics of the
articulation.  One \b forwardKinematics function computes the homogeneous
transformation matrix (4x4) describing the position (last column) and
orientation (upper left 3x3 submatrix) with respect to the inertial coordinate
system of the link specified by the link index.  If the link index is valid (in
range), the function returns \b true and the result is in \b mat;
otherwise, it returns \b false and the result in \b mat is unchanged.
The second \b forwardKinematics function is called by the first, and
composes the result in the second parameter, \b fk, which is a reference to
a \b dmABForKinStruct (from \b dm.h):

\verbatim
   struct dmABForKinStruct
   {
      RotationMatrix  R_ICS;     // orientation of links wrt ICS - ^{ICS}R_{i}
      CartesianVector p_ICS;     // position of links wrt ICS - ^{ICS}p_{i}
      SpatialVector   v;         // velocity of link wrt to i.
        ...
   };
\endverbatim

Upon exit this function will fill \b R_ICS and \b p_ICS with the
resulting transformation information and return \b true.  If the \b
link_index is out of range, this function also returns \b false.

The \b dynamics function is the entry point for computation of the dynamics
of the system.  For this class, this is a wrapper around the Articulated Body
(AB) dynamics computation for the multibody system.  The three functions, \b 
ABForwardKinematics, \b ABBackwardDynamics, and \b 
ABForwardAccelerations, comprise the implementation of the Articulated-Body
(AB) Simulation alogrithm recursions and are hidden from the user.
However, during the \b ABForwardKinematics traversal,  dmABForKinStruct s are
computed for each link which can be accessed by calling the \b 
getForKinStruct function with the index of the desired link.  This functions
returns a pointer to the requested struct, or NULL if the index is out of
range.

See also  dmSystem, dmLink.

*/

//============================================================================

class DM_DLL_API dmArticulation : public dmSystem
{
public:
   ///
   dmArticulation();
   ///
   virtual ~dmArticulation();

   ///
   bool addLink(dmLink *new_link, dmLink *parent_link);
   ///
   unsigned int getNumLinks() const { return m_link_list.size(); }
   ///
   dmLink *getLink(unsigned int index) const;
   ///
   int getLinkIndex(dmLink *link) const;
   ///
   dmLink *getLinkParent(unsigned int index) const;

   ///
   unsigned int getNumDOFs() const { return m_num_state_vars; }

   ///
   void setJointInput(Float joint_input[]);
   ///
   void setState(Float q[], Float qd[]);
   ///
   void getState(Float q[], Float qd[]) const;

   /// 
   //! calculates the Jacobian matrix for a single chain
   /**
   DM v5.0 function<CR>
   We will 
   \param index of the target link, the function will trace back to root (if ini_idx is not otherwise set)
   \return the 6xn Jacobian matrix, 
   */
   Matrix6XF calculateJacobian(unsigned int target_idx, Matrix6F & X_target, unsigned int ini_idx = 0);


   ///
   bool forwardKinematics(unsigned int link_index,
                          HomogeneousTransformationMatrix mat) const;

   ///
   virtual bool forwardKinematics(unsigned int link_index,
                                  dmABForKinStruct &fk) const;

   ///
   const dmABForKinStruct *getForKinStruct(unsigned int link_index) const;

   ///
   virtual void pushForceStates();
   ///
   virtual void popForceStates();
   ///
   virtual Float getPotentialEnergy() const;
   ///
   virtual Float getKineticEnergy() const;

   // dynamic algorithm
   ///
   void dynamics(Float *qy, Float *qdy);

   ///
   //! DM v5.0 function, Inverse Dynamics (RNEA); For articulation tree only, 
   void inverseDynamics();
	
	//! v5.0, CRB Inerta Algorithm
	void computeH();

   // rendering function:
   ///
   void draw() const;

protected:
   // not implemented
   dmArticulation(const dmArticulation &);
   dmArticulation &operator=(const dmArticulation &);

   // AB algorithm functions:
   virtual void ABForwardKinematics(Float q[], Float qd[],
                                    const dmABForKinStruct &ref_val);
   virtual void ABBackwardDynamics();
   virtual void ABForwardAccelerations(SpatialVector a_ref,
                                       Float qd[], Float qdd[]);

   struct LinkInfoStruct
   {
      unsigned int index, index_ext, dof;
      dmLink *link;

      LinkInfoStruct *parent;
      vector<LinkInfoStruct*> child_list;

      dmABForKinStruct link_val;

      //! DM v5.0, a struct containing intermediate variables for recursive NE algorithm.
      dmRNEAStruct link_val2; //spatial velocity, acceleration, force and qdd.

      //! v5.0, composite rigid body inertia	   
      CrbInertia I_C;

      // AB algorithm temporaries
      SpatialVector accel;
      SpatialVector f_star;
      SpatialTensor I_refl;
   };

   void drawTraversal(LinkInfoStruct *node) const;

protected:
   vector<LinkInfoStruct*> m_link_list;
   unsigned int m_num_state_vars;

private:
   // preallocated ABDynamics() simulation temporary variables.
   dmABForKinStruct m_ref_val;
   SpatialTensor    m_I_star_ref;
   SpatialVector    m_beta_star_ref;
   SpatialVector    m_accel_ref;
	
	MatrixXF H;
};

#endif
