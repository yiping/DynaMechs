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
 *     File: dmEnvironment.hpp
 *   Author: Scott McMillan
 *  Summary: Class definition for dmEnvironment
 *****************************************************************************/

#ifndef _DM_ENVIRONMENT_HPP
#define _DM_ENVIRONMENT_HPP

#include "dm.h"
#include "dmSystem.hpp"

//======================================================================

/**

The environment class encapsulates all the information the a dynamic simulation
needs for interaction (read forces of contact) between rigid bodies and the
surrounding environment (terrain, friction, gravity, etc...).  The default
constructor initializes an  \e \b "empty" environment with no terrain, zero gravity,
and zero friction coefficients.

This class contains a static member variable that will point to the currently
 \e \b "active" environment.  It is implemented this way because the dynamic
simulation algorithm needs one environment to query during simulation.  The
current environment is set with a call to the static member function, 
dmEnvironment::setEnvironment , and passing it a pointer to a previously
instantiated environment object.  Likewise, you can get a pointer to the
current environment with a call to the static member function 
dmEnvironment::getEnvironment.

\b IMPORTANT : The simulation algorithm will fail with a runtime error if an
environment has not been set in this fashion because various classes directly
reference this object to obtain information about the environment.  Gravity is
defined by a vector (magnitude \e and direction) and is set by calling the
\b setGravity function (and can be queried with the \b getGravity function).

This base class only supports \b prismatic \b terrains (i.e., gridded elevation data,
evenly spaced in a plane).  Currently the only way to input terrain data
is to call the \b loadTerrainData with the name of the file containing the
grid spacing and elevation data in the following format:
\verbatim
    4 5 5.0
    0   0    0    0    0
    0  -2.0 -2.0 -2.0  0
    0  -2.0 -2.0 -2.0  0
    0   0    0    0    0
\endverbatim
where the first two numbers define the dimensions of the grid (in numbers of
points along the \c x and \c y axes) and the third defines the spacing between the
points in both dimensions.  The rest define the elevation along the z axis.
The \b getTerrainFilename function returns a pointer to the terrain filename
that was loaded.

There are many other (reasonably) self-explanatory \c set and \c get member functions
that assign and return spring and damper parameters.  These parameters define
the characteristics of the contact surfaces (slipperiness, rigidity, etc.)
which are primarily by the  dmContactModel::computeForce
function which calls the \b getGroundElevation function (in the case of
\c z -axis up prismatic terrains) to help detect collisions of various contact
points with the terrain.  \b NOTE : <em> This is a very preliminary class that, like the
 dmContactModel , could stand a little bit of redesign. </em>  See the programmer's
manual (in process) for more details on the current contact algorithm.
<em> A more sophisticated contact model is under development at the Ohio State
University.</em>

The functions \b getGroundElevation and \b  getGroundDepth  both take a
position vector and returns a distance through the terrain, and passes back the
normal vector of the terrain at the point of contact.  Currently only the \b
getGroundElevation  has been tested for use with prismatic terrain defined on a
regular grid in the \c x-y plane with the \c z -axis pointing up.

Finally two functions, \b draw and \b drawInit  are not defined in the
library which the user must write to render the system using whatever 3D API
the user desires.  The framework (no graphics code) for these draw
functions for all classes are provided in \c draw.C.  Examples of these with
OpenGL code is provided with the example that accompanies this
distribution (in \b gldraw.cpp).

In the case of hydrodynamic simulation (when the library is built with the
\c DM_HYDRODYNAMICS environment variable set, or with the equivalent define in
dm.h uncommented), fluid density and motion are also stored in this class. 
The default constructor initializes an \e \b "empty"  environment with zero fluid
density and motion.  The \b getState and \b setState  member functions are
also added that get and set fluid velocity and acceleration (assumed to be the
same over the entire field), and fluid density.  The \b dynamics function
will also be used by a  dmIntegrator  object to update the fluid's
velocity based on the current acceleration.

A configuration file reader, \b dmLoadFile_env  is being supplied in the
\b dmu (DynaMechs utilities) library that can be used to instantiate and intialize
contact model objects.  The lines in the configuration file are expected to be
in the following form:
\verbatim
Environment {
    Gravity_Vector                        G_x G_y G_z
    Terrain_Data_Filename                 "filename"
    Ground_Planar_Spring_Constant         k_p
    Ground_Normal_Spring_Constant         k_n
    Ground_Planar_Damper_Constant         b_p
    Ground_Normal_Damper_Constant         b_n
    Ground_Static_Friction_Coeff          u_s
    Ground_Kinetic_Friction_Coeff         u_k

    Fluid_Density                         p        # if DM_HYDRODYNAMICS set
}
\endverbatim

See also  dmContactModel ,  dmLoadFile_env.  */

//======================================================================

class DM_DLL_API dmEnvironment : public dmSystem
{
public:
   ///
   dmEnvironment();
   ///
   virtual ~dmEnvironment();

   ///
   static void setEnvironment(dmEnvironment *env) {m_env = env;}
   ///
   static dmEnvironment *getEnvironment() {return m_env;}

   // dmSystem functions
   /// for hydrodynamics, state is cartesian velocity (not position) - unusual
   virtual unsigned int getNumDOFs() const;
   ///
   virtual void setState(Float fluid_vel[], Float fluid_acc[]);
   ///
   virtual void getState(Float fluid_vel[], Float fluid_acc[]) const;

   ///
   void loadTerrainData(const char *filename);
   ///
   Float **getTerrainData(int &xdim, int &ydim, Float &spacing)
      {
         xdim = m_x_dim; ydim = m_y_dim;
         spacing = m_grid_resolution;
         return m_depth;
      }

   ///
   const char *getTerrainFilename() const {return m_terrain_filename;}

// accessor functions:
   ///
   inline void setGravity(CartesianVector a_g);
   ///
   inline void getGravity(CartesianVector a_g) const;

   ///
   inline void setGroundPlanarSpringConstant(Float k);
   ///
   inline void setGroundNormalSpringConstant(Float k);
   ///
   inline void setGroundPlanarDamperConstant(Float k);
   ///
   inline void setGroundNormalDamperConstant(Float k);
   ///
   inline void setFrictionCoeffs(Float u_s, Float u_k);

   ///
   inline Float getGroundPlanarSpringConstant() const;
   ///
   inline Float getGroundNormalSpringConstant() const;
   ///
   inline Float getGroundPlanarDamperConstant() const;
   ///
   inline Float getGroundNormalDamperConstant() const;
   ///
   inline Float getGroundStaticFrictionCoeff() const;
   ///
   inline Float getGroundKineticFrictionCoeff() const;

   ///
   virtual Float getGroundDepth(CartesianVector contact_pos,
                                CartesianVector ground_normal) const;

   ///
   virtual Float getGroundElevation(CartesianVector contact_pos,
                                    CartesianVector ground_normal) const;

   void getTerrainDepthsArray(Float ** & depths ) const;


   // Do these really make sense in the dmEnvironment
   ///
   virtual void pushForceStates() {}
   ///
   virtual void popForceStates() {}
   ///
   virtual Float getPotentialEnergy() const { return 0; }
   ///
   virtual Float getKineticEnergy() const { return 0; }

#ifdef DM_HYDRODYNAMICS
   ///
   inline Float getFluidDensity() const { return m_fluid_density; }
   ///
   inline void  setFluidDensity(Float d)
      {
         if (d >= 0) m_fluid_density = d;
      }

   // convenience functions
   ///
   void getFluidVel(CartesianVector v) const
      { v[0] = m_fluid_vel[0]; v[1] = m_fluid_vel[1]; v[2] = m_fluid_vel[2]; }

   ///
   void getFluidAcc(CartesianVector a) const
      { a[0] = m_fluid_acc[0]; a[1] = m_fluid_acc[1]; a[2] = m_fluid_acc[2]; }
#endif

   // dynamics algorithm
   ///
   virtual void dynamics(Float *qy, Float *qdy);

   // rendering functions: should be called with a graphics context current.
   ///
   virtual void drawInit();
   ///
   virtual void draw() const;

private:
   // not implemented
   dmEnvironment(const dmEnvironment &);
   dmEnvironment &operator=(const dmEnvironment &);

private:
   static dmEnvironment *m_env;  // "there can be only one"

   CartesianVector m_gravity;    // gravitational acceleration vector ("down")
                                 // e.g. [0,0,-9.81] m/s^2 avg at Earth's
                                 // surface

   // terrain characteristics for gridded data specification
   int     m_x_dim, m_y_dim;     // dimension of terrain grid data
   Float   m_grid_resolution;    // distance b/w grid points
   Float **m_depth;              // 2D array of depth data.

   char *m_terrain_filename;
   int   m_terrain_model_index;

   // ground contact force characteristics
   Float m_ground_planar_spring_constant;
   Float m_ground_normal_spring_constant;

   Float m_ground_planar_damper_constant;
   Float m_ground_normal_damper_constant;

   Float m_ground_static_friction_coeff;
   Float m_ground_kinetic_friction_coeff;

#ifdef DM_HYDRODYNAMICS
   Float m_fluid_density;         // e.g. 1020.0 kg/m^3 avg sea water.

   CartesianVector m_fluid_vel;
   CartesianVector m_fluid_acc;
#endif
};

//----------------------------------------------------------------------------
// Inlined functions:

inline void dmEnvironment::setGravity(CartesianVector a_g)
{
   m_gravity[0] = a_g[0];
   m_gravity[1] = a_g[1];
   m_gravity[2] = a_g[2];
}

//----------------------------------------------------------------------------
inline void dmEnvironment::getGravity(CartesianVector a_g) const
{
   a_g[0] = m_gravity[0];
   a_g[1] = m_gravity[1];
   a_g[2] = m_gravity[2];
}


//----------------------------------------------------------------------------
inline void dmEnvironment::setGroundPlanarSpringConstant(Float k)
{
   m_ground_planar_spring_constant = k;
}

//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundPlanarSpringConstant() const
{
   return m_ground_planar_spring_constant;
}


//----------------------------------------------------------------------------
inline void dmEnvironment::setGroundNormalSpringConstant(Float k)
{
   m_ground_normal_spring_constant = k;
}

//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundNormalSpringConstant() const
{
   return m_ground_normal_spring_constant;
}


//----------------------------------------------------------------------------
inline void dmEnvironment::setGroundPlanarDamperConstant(Float k)
{
   m_ground_planar_damper_constant = k;
}

//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundPlanarDamperConstant() const
{
   return m_ground_planar_damper_constant;
}


//----------------------------------------------------------------------------
inline void dmEnvironment::setGroundNormalDamperConstant(Float k)
{
   m_ground_normal_damper_constant = k;

}

//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundNormalDamperConstant() const
{
   return m_ground_normal_damper_constant;
}


//----------------------------------------------------------------------------
inline void dmEnvironment::setFrictionCoeffs(Float u_s, Float u_k)
{
   m_ground_static_friction_coeff = u_s;
   m_ground_kinetic_friction_coeff = u_k;

   if (m_ground_kinetic_friction_coeff > m_ground_static_friction_coeff)
   {
      cerr << "dmEnvironment error: kinetic > static friction coefficient.\n";
   }
}

//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundStaticFrictionCoeff() const
{
   return m_ground_static_friction_coeff;
}


//----------------------------------------------------------------------------
inline Float dmEnvironment::getGroundKineticFrictionCoeff() const
{
   return m_ground_kinetic_friction_coeff;
}

#endif
