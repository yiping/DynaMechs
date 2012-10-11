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
 *     File: dmRigidBody.cpp
 *   Author: Scott McMillan
 *  Summary: Class declaration for rigid bodies
 *****************************************************************************/

#include "dm.h"
#include "dmRigidBody.hpp"
#include "dmForce.hpp"
#include "dmEnvironment.hpp"

//============================================================================
// class dmRigidBody
//============================================================================

//----------------------------------------------------------------------------
//    Summary: default constructor - builds a generic rigid body with arbitrary
//             (but valid) inertia parameters.
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
dmRigidBody::dmRigidBody() : dmLink()
{
   m_mass = 1.0;

   m_I_bar[0][0] = 1.0;  m_I_bar[0][1] = 0.0;  m_I_bar[0][2] = 0.0;
   m_I_bar[1][0] = 0.0;  m_I_bar[1][1] = 1.0;  m_I_bar[1][2] = 0.0;
   m_I_bar[2][0] = 0.0;  m_I_bar[2][1] = 0.0;  m_I_bar[2][2] = 1.0;

   m_cg_pos[0] = m_cg_pos[1] = m_cg_pos[2] = 0.0;

   for (int k=0; k<6; k++) m_external_force[k] = 0.0;

#ifdef DM_HYDRODYNAMICS
   m_displaced_fluid_vol = 0.0;
   m_displaced_fluid_mass = 0.0;
   m_cb_pos[0] = m_cb_pos[1] = m_cb_pos[2] = 0.0;

   for (int i=0; i<6; i++)
   {
      m_Cd_A_p[i] = 0.0;

      for (int j=0; j<6; j++)
         m_I_added_mass[i][j] = 0.0;
   }

   m_axis = 0;
   m_x0 = 0.0;
   m_length = 0.0;
   m_radius = 0.0;
   m_Ca = 0.0;
   m_C2rl = 0.0;
#endif
}


//----------------------------------------------------------------------------
dmRigidBody::~dmRigidBody()
{
}

//----------------------------------------------------------------------------
bool dmRigidBody::setInertiaParameters(Float mass,
                                       CartesianTensor I_bar,
                                       CartesianVector cg_pos)
{
   register int i,j;

   // Verify positive-definiteness of spatial Inertia matrix before going on:
   SpatialTensor I_star;

   for (j = 0; j < 6; j++)
      for (i = 0; i < 6; i++)
         I_star[j][i] = 0.0;

   for (j = 0; j < 3; j++)
   {
      I_star[j + 3][j + 3] = mass;
      for (i = 0; i < 3; i++)
         I_star[j][i] = I_bar[j][i];
   }

// off diagonal block of spatial inertia:
   I_star[2][4] = I_star[4][2] =  mass*cg_pos[0];
   I_star[1][5] = I_star[5][1] = -mass*cg_pos[0];
   I_star[0][5] = I_star[5][0] =  mass*cg_pos[1];
   I_star[2][3] = I_star[3][2] = -mass*cg_pos[1];
   I_star[1][3] = I_star[3][1] =  mass*cg_pos[2];
   I_star[0][4] = I_star[4][0] = -mass*cg_pos[2];

   // Perform the root-free cholesky decomposition (IStar = L D L').
   for (int k = 0; k < 5; k++)
   {
      for (i = 5; i > k; i--)
      {
         Float tem = I_star[i][k]/I_star[k][k];

         for (j = i; j > k; j--)
         {
            I_star[i][j] -= I_star[j][k]*tem;
         }
         I_star[i][k] = tem;
      }
   }

#ifdef DEBUG
   cout << "dmRigidBody constructor: PD check" << endl;
#endif
   bool pd_flag = true;
   for (i=0; i<6; i++)
   {
      if (I_star[i][i] <= 0.0) pd_flag = false;
#ifdef DEBUG
      cout << ' ' << I_star[i][i];
#endif
   }
#ifdef DEBUG
   cout << endl;
#endif

   // regardless of whether or not the matrix is PD, go ahead and set the
   // member variables
   for (j = 0; j < 6; j++)
      for (i = 0; i < 6; i++)
         m_SpInertia[j][i] = 0.0;

   m_mass = mass;

   for (i = 0; i < 3; i++)
   {
      m_SpInertia[i + 3][i + 3] = m_mass;

      m_cg_pos[i] = cg_pos[i];
      m_h[i] = m_mass*m_cg_pos[i];

      for (j = 0; j < 3; j++)
      {
         m_I_bar[i][j] = I_bar[i][j];
         m_SpInertia[i][j] = m_I_bar[i][j];
      }
   }

// off diagonal block of spatial inertia:
   m_SpInertia[2][4] = m_SpInertia[4][2] =  m_h[0];
   m_SpInertia[1][5] = m_SpInertia[5][1] = -m_h[0];
   m_SpInertia[0][5] = m_SpInertia[5][0] =  m_h[1];
   m_SpInertia[2][3] = m_SpInertia[3][2] = -m_h[1];
   m_SpInertia[1][3] = m_SpInertia[3][1] =  m_h[2];
   m_SpInertia[0][4] = m_SpInertia[4][0] = -m_h[2];

   initABVars();

   // Let the user know if the matrices were PD or not (Note under some very
   // specific conditions the simulation can withstand a positive semidefinite
   // inertia matrix
   if (!pd_flag)
   {
      cerr << "Error: ["<< m_name <<"] Rigid Body inertia is not positive definite." << endl;
   }

   return pd_flag;
}

//----------------------------------------------------------------------------
void dmRigidBody::getInertiaParameters(Float &mass,
                                       CartesianTensor I_bar,
                                       CartesianVector cg_pos) const
{
   mass = m_mass;

   for (int i=0; i<3; i++)
   {
      cg_pos[i] = m_cg_pos[i];
      for (int j=0; j<3; j++)
         I_bar[i][j] = m_I_bar[i][j];
   }
}

#ifdef DM_HYDRODYNAMICS
//----------------------------------------------------------------------------
void dmRigidBody::setHydrodynamicParameters(Float volume,
                                            SpatialTensor I_added_mass,
                                            CartesianVector cb_pos,
                                            int drag_axis,
                                            Float cyl_min,
                                            Float cyl_max,
                                            Float cyl_radius,
                                            Float C_d)
{
   register int i, j;

   Float density = dmEnvironment::getEnvironment()->getFluidDensity();

// hydrodynamic properties:
   m_displaced_fluid_vol = volume;
   m_displaced_fluid_mass = m_displaced_fluid_vol*density;

   for (i = 0; i < 6; i++) {
      for (j = 0; j < 6; j++) {
         m_I_added_mass[i][j] = I_added_mass[i][j];
         m_SpInertia[i][j] += m_I_added_mass[i][j];
      }
   }

   for (i=0; i<3; i++)
   {
      m_cb_pos[i] = cb_pos[i];
   }

// drag parameters
   m_axis = drag_axis;

   /* FIXME - oops this is no longer supported
   if (m_axis == -1) {
      readConfigParameterLabel(cfg_ptr, "Drag_Coefficients");
      for (j = 0; j < 6; j++)
         cfg_ptr >> m_Cd_A_p[j];
   }
   else
   {
   */

   m_x0 = cyl_min;
   m_length = cyl_max - m_x0;
   m_radius = cyl_radius;

   Float area = M_PI*m_radius*m_radius;
   Float C2 = density*C_d;
   m_Ca = 0.5*C2*area;
   m_C2rl = C2*m_radius*m_length;
}
#endif

//----------------------------------------------------------------------------
//    Summary: add a force computation object to the rigid body.
//             This function does not check to see if this force object
//             has been added previously (error).
// Parameters: pointer to the force.
//    Returns: true if the operation is successful, else false
//----------------------------------------------------------------------------
bool dmRigidBody::addForce(dmForce *force)
{
   if (force == NULL)
   {
      cerr << "dmRigidBody::addForce error: NULL force pointer."
           << endl;
      return false;
   }

   m_force.push_back(force);

   return true;
}

//----------------------------------------------------------------------------
//    Summary: get a pointer to a particular force as specified by an
//             index (in the order they are added starting at zero).
// Parameters: index from 0 to m_num_forces-1
//    Returns: a pointer to the desired force, or NULL if the index is
//             out of range.
//----------------------------------------------------------------------------
dmForce *dmRigidBody::getForce(unsigned int index) const
{
   if (index >= m_force.size())
   {
      cerr << "dmRigidBody::getForce error: index out of range "
           << index << endl;
      return NULL;
   }
   return m_force[index];
}

//----------------------------------------------------------------------------
//    Summary: get the index of the force pointed to by the parameter
//             provided it has been added to this rigid body
// Parameters: pointer to the force
//    Returns: index of the force, or -1 if it is not contained in this
//             rigid body.
//----------------------------------------------------------------------------
int dmRigidBody::getForceIndex(dmForce *force) const
{
   for (unsigned int i=0; i<m_force.size(); i++)
   {
      if (force == m_force[i])
      {
         return (int) i;
      }
   }

   return -1;
}

//----------------------------------------------------------------------------
bool dmRigidBody::removeForce(dmForce *force)
{
   int index = getForceIndex(force);

   if (index < 0)
   {
      cerr << "dmRigidBody::removeForce(force) error: force not added before."
           << endl;
      return false;
   }

   m_force.erase(m_force.begin() + index);

   return true;
}

//----------------------------------------------------------------------------
bool dmRigidBody::removeForce(unsigned int index)
{
   if (index >= m_force.size())
   {
      cerr << "dmRigidBody::removeForce(index) error: index out of range."
           << endl;
      return false;
   }

   m_force.erase(m_force.begin() + index);

   return true;
}

//----------------------------------------------------------------------------
//    Summary: saves the state of all attached force objects
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmRigidBody::pushForceStates()
{
   for (unsigned int i=0; i<m_force.size(); i++)
      m_force[i]->pushState();
}

//----------------------------------------------------------------------------
//    Summary: restores state of all attached force objects from saved state
// Parameters: none
//    Returns: none
//----------------------------------------------------------------------------
void dmRigidBody::popForceStates()
{
   for (unsigned int i=0; i<m_force.size(); i++)
      m_force[i]->popState();
}

//----------------------------------------------------------------------------
//    Summary: Computes and returns the gravitational potential energy of
//             the rigid body link.  The potential energy zero is defined
//             by  the plane passing through the origin of the inertial
//             coordinate system with the gravity vector defining the normal.
// Parameters: link_val - pointer to struct filled with the kinematic
//                        parameters of the rigid body link
//             a_gravity - gravitational acceleration vector (w.r.t. ICS)
//    Returns: potential energy of the rigid body link
//----------------------------------------------------------------------------
Float dmRigidBody::getPotentialEnergy(const dmABForKinStruct &link_val,
                                      CartesianVector a_gravity) const
{
   CartesianVector p_COM; // link center of mass wrt ICS

   for (int i = 0; i < 3; i++)
      p_COM[i] = (link_val.p_ICS[i]
                  + link_val.R_ICS[i][0]*m_cg_pos[0]
                  + link_val.R_ICS[i][1]*m_cg_pos[1]
                  + link_val.R_ICS[i][2]*m_cg_pos[2]);

   Float potentialEnergy = - m_mass*(a_gravity[0]*p_COM[0] +
                                     a_gravity[1]*p_COM[1] +
                                     a_gravity[2]*p_COM[2]);

   return potentialEnergy;
}

//----------------------------------------------------------------------------
//    Summary: Computes and returns the kinetic energy of the rigid body link.
// Parameters: link_val - pointer to struct filled with the kinematic
//                        parameters of the rigid body link
//    Returns: kinetic energy of the rigid body link
//----------------------------------------------------------------------------
Float dmRigidBody::getKineticEnergy(const dmABForKinStruct &link_val) const
{
   CartesianVector Ibar_w;
   for (int i = 0; i < 3; i++)
      Ibar_w[i] = (m_I_bar[i][0]*link_val.v[0] +
                   m_I_bar[i][1]*link_val.v[1] +
                   m_I_bar[i][2]*link_val.v[2]);

   Float w_Ibar_w = (link_val.v[0]*Ibar_w[0] +
                     link_val.v[1]*Ibar_w[1] +
                     link_val.v[2]*Ibar_w[2]);

   Float m_v_v = m_mass*(link_val.v[3]*link_val.v[3] +
                         link_val.v[4]*link_val.v[4] +
                         link_val.v[5]*link_val.v[5]);


   CartesianVector w_cross_cg;
   crossproduct(&link_val.v[0], m_cg_pos, w_cross_cg);

   Float m_v_w_cg = m_mass*(link_val.v[3]*w_cross_cg[0] +
                            link_val.v[4]*w_cross_cg[1] +
                            link_val.v[5]*w_cross_cg[2]);

   Float kineticEnergy = 0.5*(w_Ibar_w + m_v_v) + m_v_w_cg;

   return kineticEnergy;
}

//----------------------------------------------------------------------------
//    Summary: compute the state-dependent bias force term need for all rigid
//             body objects
// Parameters: omega - angular velocity of body express in body's CS
//    Returns: beta - spatial bias force vector
//----------------------------------------------------------------------------
void dmRigidBody::computeBeta(const dmABForKinStruct &link_val_curr,
                              SpatialVector beta)
{
   unsigned int i;
   CartesianVector tem;

   for (i = 0; i < 3; i++)
   {
      tem[i] = m_I_bar[i][0]*link_val_curr.v[0] +
               m_I_bar[i][1]*link_val_curr.v[1] +
               m_I_bar[i][2]*link_val_curr.v[2];
   }
   crossproduct(tem, link_val_curr.v, &beta[0]);  // beta_ref[0..2] = Iw x w;

   crossproduct(link_val_curr.v, m_h, tem);
   crossproduct(tem, link_val_curr.v, &beta[3]);  // beta_ref[3..5] = (wxh)xw;

#ifdef DM_HYDRODYNAMICS
   SpatialVector beta_H;
   computeHydrodynamicBias(link_val_curr, beta_H);
   for (i = 0; i < 6; i++)
   {
      m_beta[i] += beta_H[i];
   }
#endif
}

#ifdef DM_HYDRODYNAMICS
//----------------------------------------------------------------------------
//    Summary: compute the hydrodynamic drag on the submerged body
// Parameters: v_rel - spatial velocity relative to the fluid
//    Returns: f_D - spatial drag force expressed wrt body's CS
//----------------------------------------------------------------------------
void dmRigidBody::computeDrag(SpatialVector v_rel, SpatialVector f_D)
{
   register int k;
   register Float x, vn_mag, tmp;
   CartesianVector vn, tem;

   static Float gqx[4] = {0.069431844,     // Gauss-quadrature constants.
                          0.330009478,
                          0.669990521,
                          0.930568155};
   static Float gqk[4] = {0.1739274225687,
                          0.3260725774312,
                          0.3260725774312,
                          0.1739274225687};

   for (k = 0; k < 6; k++)
      f_D[k] = 0.0;

   if (m_axis == -1)
   {
      for (k = 0; k < 6; k++)
         f_D[k] = -m_Cd_A_p[k]*v_rel[k]*fabs(v_rel[k]);
   }
   else if (m_axis == 0)
   {
      // x-axis moment and force
      f_D[3] = -m_Ca*v_rel[3]*fabs(v_rel[3]);

      // y,z components - using Gauss-quadrature
      for (k = 0; k < 4; k++)
      {
         x = m_x0 + m_length*gqx[k];
         vn[1] = v_rel[4] + v_rel[2]*x;
         vn[2] = v_rel[5] - v_rel[1]*x;

         vn_mag = sqrt(vn[1]*vn[1] + vn[2]*vn[2]);

         tmp = gqk[k]*vn_mag;
         tem[1] = tmp*vn[1];
         tem[2] = tmp*vn[2];

         f_D[1] += (-x*tem[2]);
         f_D[2] += x*tem[1];
         f_D[4] += tem[1];
         f_D[5] += tem[2];
      }
      f_D[1] *= -m_C2rl;
      f_D[2] *= -m_C2rl;
      f_D[4] *= -m_C2rl;
      f_D[5] *= -m_C2rl;
   }
   else if (m_axis == 1)
   {
      // y-axis moment and force
      f_D[4] = -m_Ca*v_rel[4]*fabs(v_rel[4]);

      // z,x components
      for (k = 0; k < 4; k++)
      {
         x = m_x0 + m_length*gqx[k];
         vn[0] = v_rel[3] - v_rel[2]*x;
         vn[2] = v_rel[5] + v_rel[0]*x;

         vn_mag = sqrt(vn[2]*vn[2] + vn[0]*vn[0]);

         tmp = gqk[k]*vn_mag;
         tem[0] = tmp*vn[0];
         tem[2] = tmp*vn[2];

         f_D[0] += (x*tem[2]);
         f_D[2] += -x*tem[0];
         f_D[3] += tem[0];
         f_D[5] += tem[2];
      }
      f_D[0] *= -m_C2rl;
      f_D[2] *= -m_C2rl;
      f_D[3] *= -m_C2rl;
      f_D[5] *= -m_C2rl;
   }
   else if (m_axis == 2)
   {
      // z-axis moment and force
      f_D[5] = -m_Ca*v_rel[5]*fabs(v_rel[5]);

      // y,x components
      for (k = 0; k < 4; k++)
      {
         x = m_x0 + m_length*gqx[k];
         vn[0] = v_rel[3] + v_rel[1]*x;
         vn[1] = v_rel[4] - v_rel[0]*x;

         vn_mag = sqrt(vn[1]*vn[1] + vn[0]*vn[0]);

         tmp = gqk[k]*vn_mag;
         tem[0] = tmp*vn[0];
         tem[1] = tmp*vn[1];

         f_D[0] += (-x*tem[1]);
         f_D[1] += x*tem[0];
         f_D[3] += tem[0];
         f_D[4] += tem[1];
      }
      f_D[0] *= -m_C2rl;
      f_D[1] *= -m_C2rl;
      f_D[3] *= -m_C2rl;
      f_D[4] *= -m_C2rl;
   }
}
#endif

#ifdef DM_HYDRODYNAMICS
//----------------------------------------------------------------------------
//    Summary: compute the hydrodynamic portion of the bias (state-dependent)
//             force.
// Parameters: val - the struct containing all the necessary parameters to
//                   compute this force, including all of the hydrodynamic
//                   terms.
//    Returns: beta_H - spatial hydro bias force
//----------------------------------------------------------------------------
void dmRigidBody::computeHydrodynamicBias(const dmABForKinStruct &val,
                                          SpatialVector beta_H)
{
   register int i;
   CartesianVector tem1, tem2, tem3;
   SpatialVector tema, v_rel, f_TB, f_D;

// 2.4
   v_rel[0] = val.v[0];
   v_rel[1] = val.v[1];
   v_rel[2] = val.v[2];
   v_rel[3] = val.v[3] - val.v_f[0];
   v_rel[4] = val.v[4] - val.v_f[1];
   v_rel[5] = val.v[5] - val.v_f[2];

// 2.8 compute total bouyancy force.
   f_TB[3] = m_displaced_fluid_mass*val.a_fg[0];
   f_TB[4] = m_displaced_fluid_mass*val.a_fg[1];
   f_TB[5] = m_displaced_fluid_mass*val.a_fg[2];
   crossproduct(m_cb_pos, &f_TB[3], f_TB);

// drag force.
   computeDrag(v_rel, f_D);

// added mass bias force.
   crossproduct(&val.v[0], &v_rel[3], tem1);
   tem1[0] += val.a_fg[0];
   tem1[1] += val.a_fg[1];
   tem1[2] += val.a_fg[2];

   for (i = 0; i < 6; i++)
   {
      beta_H[i] = m_I_added_mass[i][3]*tem1[0] +
                  m_I_added_mass[i][4]*tem1[1] +
                  m_I_added_mass[i][5]*tem1[2];
      tema[i] = m_I_added_mass[i][0]*v_rel[0] + m_I_added_mass[i][3]*v_rel[3] +
                m_I_added_mass[i][1]*v_rel[1] + m_I_added_mass[i][4]*v_rel[4] +
                m_I_added_mass[i][2]*v_rel[2] + m_I_added_mass[i][5]*v_rel[5];
   }

   crossproduct(&val.v[0], tema, tem1);
   crossproduct(&v_rel[3], &tema[3], tem2);
   crossproduct(&val.v[0], &tema[3], tem3);

   beta_H[0] -= (tem1[0] + tem2[0]);
   beta_H[1] -= (tem1[1] + tem2[1]);
   beta_H[2] -= (tem1[2] + tem2[2]);
   beta_H[3] -= tem3[0];
   beta_H[4] -= tem3[1];
   beta_H[5] -= tem3[2];

// accumulate all forces into _beta
   for (i = 0; i < 6; i++)
   {
      beta_H[i] += f_TB[i] + f_D[i];
   }
}
#endif

void dmRigidBody::initializeCrbInertia(CrbInertia & IC_curr) const
{
	IC_curr.m = m_mass;
	for (int i=0; i<3; i++) {
		IC_curr.h[i]=m_h[i];
		for (int j=0; j<3; j++) {
			IC_curr.IBar[i][j]=m_I_bar[i][j];
		}
	}
}

//-----------------------------------------------------------------------------------------
Matrix6F dmRigidBody::getSpatialInertiaMatrix()
{
	Matrix6F I;
	for (int i = 0; i<6; i++)
	{
		for (int j = 0; j<6; j++)
		{
			I(i,j) = m_SpInertia[i][j];
		}
	}
		
//	I<< m_SpInertia[0][0], m_SpInertia[0][1], m_SpInertia[0][2], m_SpInertia[0][3], m_SpInertia[0][4], m_SpInertia[0][5], 
//	    m_SpInertia[1][0], m_SpInertia[1][1], m_SpInertia[1][2], m_SpInertia[1][3], m_SpInertia[1][4], m_SpInertia[1][5], 
//	    m_SpInertia[2][0], m_SpInertia[2][1], m_SpInertia[2][2], m_SpInertia[2][3], m_SpInertia[2][4], m_SpInertia[2][5],
//	    m_SpInertia[3][0], m_SpInertia[3][1], m_SpInertia[3][2], m_SpInertia[3][3], m_SpInertia[3][4], m_SpInertia[3][5], 
//	    m_SpInertia[4][0], m_SpInertia[4][1], m_SpInertia[4][2], m_SpInertia[4][3], m_SpInertia[4][4], m_SpInertia[4][5], 
//	    m_SpInertia[5][0], m_SpInertia[5][1], m_SpInertia[5][2], m_SpInertia[5][3], m_SpInertia[5][4], m_SpInertia[5][5];
	return I;
}



