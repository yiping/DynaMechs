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
 *     File: dmRigidBody.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for dmRigidBody
 *****************************************************************************/

#ifndef _DM_RIGID_BODY_HPP
#define _DM_RIGID_BODY_HPP

#include "dm.h"
#include "dmForce.hpp"
#include "dmLink.hpp"

#include <vector>

//======================================================================
/**

This class is the repository for all the dynamic parameters for rigid bodies
(links) that are needed for the AB dynamics algorithm.  It is subclassed
from {\tt dmLink} and subclassed by all links that have degrees of freedom to
be simulated.  Remember: {\tt dmLink} is a misleading name as it specifies the
API needed by the AB algorithm and could also correspond to non-variable
transformations like the {\tt dmZScrewTxLink} class that is only used for
kinematic manipulation (transformations), but do not have a physical
manifestation.

The {\tt setInertiaParameters} and {\tt getInertiaParameters} functions is used
set and retrieve the inertial parameters.  The first argument specifies the
mass, $m$, of the body, the second specifies the $3\times 3$ inertia matrix,
$\bar{\mbox{\boldmath $I$}}$, of the body with respect to the body's coordinate
system, and the third specifies the position of the body's center of gravity,
$\mbox{\boldmath $p$}_{cg}$, with respect to the body's coordinate system
origin.  Note that from these parameters, the {\tt setInertiaParameters}
function constructs the the spatial inertial matrix from the mass, center of
gravity and inertia tensor and verifies that it is positive definite:
\begin{eqnarray*}
{\bf I} & = & \left[ \begin{array}{cc}
    \bar{\mbox{\boldmath $I$}} &
                  m\:\tilde{\mbox{\boldmath $p$}}_{cg} \\
    m\:\tilde{\mbox{\boldmath $p$}}^T_{cg}  &  m{\bf 1}_3
  \end{array} \right].
\end{eqnarray*}
Note that $\bar{\mbox{\boldmath $I$}}$ is related to the body's inertia about
its principle axes, $\bar{\mbox{\boldmath $I$}}_{cg}$, as follows:
\begin{eqnarray*}
\bar{\mbox{\boldmath $I$}}  & = &
   \mbox{\boldmath $R$}^T
            \bar{\mbox{\boldmath $I$}}_{cg}
                              \mbox{\boldmath $R$} +
   m \tilde{\mbox{\boldmath $p$}}_{cg} \tilde{\mbox{\boldmath $p$}}_{cg}^T,
\end{eqnarray*}
where $\mbox{\boldmath $R$}$ specifies the rotation between the principle axes
and the body's coordinate system.

Note that for underwater simulation, an entirely new library is made with the
{\tt DM_HYDRODYNAMICS} environment variable defined.  In this case, this class
requires additional hydrodynamic parameters to compute the additional
hydrodynamic effects.  These are set with the {\tt setHydrodynamicParameters}
function.  The {\tt volume} parameter is the volume of fluid that this rigid
body displaces.  The {\tt I\_added\_mass} parameter is a $6\times 6$ positive
semi-definite matrix corresponding to the added mass.  The third parameter,
{\tt cb\_pos} is the position of the center of bouyancy with the respect to the
body's coordinate system.  The remaining parameters are used to compute a rough
approximation of a drag force.  In this algorithm, the body must be
approximated with an axis aligned cylinder.  The {\tt drag\_axis} indicates
which axis (0 for x axis, 1 for y, or 2 for z), {\tt cyl\_min} and {\tt
cyl\_max} gives the coordinates along this axis which defines the extent of the
approximating cylinder, and {\tt cyl\_radius} defines its radius.  Finally,
{\tt C\_d} defines the coefficient of drag.

This discussion requires the obvious note about units.  DynaMechs makes no
assumptions about length and mass units (only their consistency).  It is
required that once a convention is chosen (MKS, cgs, etc..), it is adhered to
in the specification of ALL the parameters in all of the classes.

Motion of rigid bodies can be under the influence of a variety of external
forces.  Complaint forces (a function of the rigid body's state) can be
implemented by subclassing from {\tt dmForce} and "adding" instantiations of
them to this class.  The {\tt dmContactModel} is an example of a derived force
class.  Any number of previously instantiated {\tt dmForce} objects can be
added to the {\tt dmRigidBody} class using the {\tt addForce} function.
The number of force objects associated with this class is returned when a call
to {\tt getNumForces} is made.

Force objects are added to an internal list in order such that the first one is
index 0, the second is index 1, and so forth.  The current index of a force
object in the list of associated objects is returned when a call to {\tt
getForceIndex} is made and the pointer to a {\tt dmForce} object, in question,
is provided.  Alternatively a pointer to a force object is returned when a call
to {\tt getForce} is made with a given index.  This function returns NULL if
the index is out of range (0,..,num\_forces-1).

Force objects can be removed with a call to either {\tt removeForce} function,
which will return true if the operation is successful.  The force to be removed
can be referenced either by its index or a pointer.  If the index is out of
range or the pointer doesn't match any of the current force objects in the
list, or there was an internal failure, the function will return false.  Note
that when a force object is removed all indices for forces added later than the
one in question will be renumbered (one lower).

In addition to the force objects, this class also provides the functions, {\tt
setExternalForce} and {\tt getExternalForce}, which can be used to directly
specify an external force exerted on the rigid body.  The parameter, {\tt
force}, is a spatial vector consisting of the moment and linear force expressed
with respect to the body's coordinate system.

The {\tt computeBeta}, and, in the case of hydrodynamic simulation, {\tt
computeDrag} and {\tt computeHydrodynamicBias} are all functions called by the
{\tt ABForwardKinematics} functions of the various derived objects to compute
state (position and velocity) dependent bias forces experience by the bodies.

See also {\tt dmLink}, {\tt dmForce}, {\tt dmContactModel}, {\tt dmLoadFile\_dm}.

 */

class DM_DLL_API dmRigidBody : public dmLink
{
public:
   ///
   dmRigidBody();
   ///
   virtual ~dmRigidBody();

   ///
   bool setInertiaParameters(Float mass,
                             CartesianTensor inertia,
                             CartesianVector cg_pos);

   ///
   void getInertiaParameters(Float &mass,
                             CartesianTensor inertia,
                             CartesianVector cg_pos) const;

   // manage a list of force objects.
   ///
   unsigned int getNumForces() const { return m_force.size(); }
   ///
   bool addForce(dmForce *force);
   ///
   int getForceIndex(dmForce *force) const;
   ///
   dmForce *getForce(unsigned int index) const;
   ///
   bool removeForce(dmForce *force);
   ///
   bool removeForce(unsigned int index);
   ///
   virtual void pushForceStates();
   ///
   virtual void popForceStates();
   ///
   virtual Float getPotentialEnergy(const dmABForKinStruct &link_val,
                                    CartesianVector a_gravity) const;
   ///
   virtual Float getKineticEnergy(const dmABForKinStruct &link_val) const;

   ///
   void getExternalForce(SpatialVector force)
      {
         for (int i=0; i<6; i++) force[i] = m_external_force[i];
      }

   ///
   void setExternalForce(SpatialVector force)
      {
         for (int i=0; i<6; i++) m_external_force[i] = force[i];
      }

   ///
   void computeBeta(const dmABForKinStruct &link_curr_val,
                    SpatialVector beta);

#ifdef DM_HYDRODYNAMICS
   ///
   void setHydrodynamicParameters(Float volume,
                                  SpatialTensor I_added_mass,
                                  CartesianVector cb_pos,
                                  int drag_axis,
                                  Float cyl_min,
                                  Float cyl_max,
                                  Float cyl_radius,
                                  Float C_d);

   ///
   void computeDrag(SpatialVector v_rel, SpatialVector f_D);
   ///
   void computeHydrodynamicBias(const dmABForKinStruct &val,
                                SpatialVector beta_H);
#endif

protected:
   // not implemented
   dmRigidBody(const dmRigidBody &);
   dmRigidBody &operator=(const dmRigidBody &);

   // the following is used to initialize AB vars
   virtual void initABVars() = 0;

protected:
   SpatialTensor m_SpInertia;   // 6x6 spatial inertia matrix
                                //    this also contains the addedMass when
                                //    hydrodynamic simulation is being
                                //    performed.

   // Note: the spatial inertia matrix is computed as follows:
   //          | I_bar                \tilde{mass*cg_pos} |
   //      I = |                                          |
   //          | \tilde{mass*cg_pos}'            mass*E_3 |
   //
   //      where E_3 is the 3x3 indentity matrix,
   //            h = mass*cg_pos, and
   //        tilde = crossproduct operator

   // AB algorithm variables common to all rigid bodies
   SpatialVector m_beta;        // body bias force
   SpatialVector m_beta_star;   // AB bias force
   SpatialTensor m_I_star;      // AB Inertia

   SpatialVector m_external_force;  // direct external force wrt to BCS

   // list of force objects that act on rigid body like compliant contacts
   vector<dmForce*> m_force;

private:
   // This are variables that only the rigid body object seems to use:
   Float           m_mass;
   CartesianVector m_cg_pos;    // position of center of gravity in BCS
   CartesianVector m_h;         // first mass moment = mass*cg_pos.
   CartesianTensor m_I_bar;     // 3x3 inertia tensor about body axes.


#ifdef DM_HYDRODYNAMICS
   Float m_displaced_fluid_vol;   // volume of fluid displaced by body.
   Float m_displaced_fluid_mass;  // mass of fluid displaced by body
   CartesianVector m_cb_pos;      // position of center of buoyancy in BCS
   SpatialTensor m_I_added_mass;  // 6x6 added mass/inertia matrix

   int m_axis;                    // drag parameters (for cylinder)
   SpatialVector m_Cd_A_p;        // spatial coefficients for cube's drag
   Float m_x0, m_length, m_radius;
   Float m_Ca, m_C2rl;
#endif
};

#endif
