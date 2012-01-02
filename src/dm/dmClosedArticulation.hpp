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
 *     File: dmClosedArticulation.hpp
 *   Author: Duane Marhefka
 *  Created: 1999
 *  Summary: Class declaration for articulations with closed loops.
 *****************************************************************************/

#ifndef _DM_CLOSED_ARTICULATION_HPP
#define _DM_CLOSED_ARTICULATION_HPP

#include "dm.h"
#include "dmSecondaryJoint.hpp"
#include "dmArticulation.hpp"

/**
The {\tt dmClosedArticulation} class is derived from {\tt dmArticulation}
and is used for simulating systems containing kinematic loops.  The
{\tt dmClosedArticulation} class adds loop-closing joints known as secondary
joints to the tree-structure in the {\dmArticulation}.  A secondary joint
may be added to the {\tt dmClosedArticulation} as a `Hard' joint or as a `Soft'
joint using the functions {\tt addHardSecondaryJoint} and
{\tt addSoftSecondaryJoint}.  A `Hard' joint closes a kinematic loop by
modifying the AB algorithm with loop-closure constraint equations, while a
`Soft' joint closes a kinematic loop through the use of compliant elements in the
constrained directions of the joint.  See {\tt dmSecondaryJoint} for further
details.

The functions, {\tt getNumHardSecondaryJoints} and
{\tt getNumSoftSecondaryJoints} return the number of `Hard' and `Soft'
secondary joints in the system, respectively.  Additionally, the function
{\tt getHardSecondaryJoint} will return a pointer to the secondary joint
with given `Hard' secondary joint index, while {\tt getSoftSecondaryJoint}
will return a pointer to the secondary joint with given `Soft' secondary
joint index.

The function, {\tt initKinematicLoopVars}, must be called after all of the
links and secondary joints have been added to the closed articulation.  This
function computes all of the sets used for processing the loops in the
simulation algorithm.  Note that the provided loader (see {\tt dmLoadFile_dm})
automatically calls this function.

The {\tt eliminateLoops} and {\tt propagateConstraints} functions are called
by the backwards dynamics functions of the closed articulation.  The
{\tt eliminateLoops} function is called when the current link to eliminate is
a root link, in order to eliminate the constraint forces of the rooted loops.
The {\tt propagateConstraints} function is then called to eliminate the current
link (with rooted loops eliminated) from the remaining loop constraint equations.
Finally, the {\tt computeConstraintForces} function is called during the
forward accelerations recursion of the AB algorithm in order to completely solve
for the constraint forces of loops rooted at the current link.
 */

class DM_DLL_API dmClosedArticulation : public dmArticulation
{
public:
   ///
   dmClosedArticulation();
   ///
   ~dmClosedArticulation();

   ///
   bool addHardSecondaryJoint(dmSecondaryJoint *joint);
   ///
   bool addSoftSecondaryJoint(dmSecondaryJoint *joint);

   ///
   unsigned int getNumHardSecondaryJoints() const
      { return m_sec_joints.size(); }
   ///
   unsigned int getNumSoftSecondaryJoints() const
      { return m_soft_joints.size(); }

   ///
   dmSecondaryJoint *getHardSecondaryJoint(unsigned int index) const;
   ///
   dmSecondaryJoint *getSoftSecondaryJoint(unsigned int index) const;


   ///
   void initKinematicLoopVars();
   ///
   void freeKinematicLoopVars();

   ///
   void eliminateLoops(unsigned int i);
   ///
   void propagateConstraints(unsigned int i);
   ///
   void computeConstraintForces(unsigned int i);

protected:
   ///
   virtual void ABForwardKinematics(Float q[], Float qd[],
                                    const dmABForKinStruct &ref_val);
   ///
   virtual void ABBackwardDynamics();
   ///
   virtual void ABForwardAccelerations(SpatialVector a_ref,
                                       Float qd[], Float qdd[]);

private:   // not implemented
   dmClosedArticulation(const dmClosedArticulation &);
   dmClosedArticulation &operator=(const dmClosedArticulation &);

protected:
   unsigned int *m_num_elements_LB;
   unsigned int **m_LB;

   unsigned int *m_num_elements_LR;
   unsigned int **m_LR;

   unsigned int *m_num_elements_LC;
   unsigned int **m_LC;

   unsigned int *m_num_elements_LJ_star;
   unsigned int **m_LJ_star;

   Float ****m_Xik;
   Float ****m_Bmn;
   Float **m_zetak;

   Float **m_Binv_zetai;
   Float ***m_Binv_Xi;
   Float ****m_Binv_Bim;
   unsigned int *m_constraints_at_root;

   Float **m_lambda_c;

   vector<dmSecondaryJoint*> m_sec_joints;  // 'Hard' secondary joints.
   vector<dmSecondaryJoint*> m_soft_joints; // 'Soft' secondary joints.

   unsigned int *m_loop_root_index; // HACK - must put in root link req.
   unsigned int *m_num_constraints;
};

#endif
