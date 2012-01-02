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
 *     File: dmLoadfile_env.cpp
 *   Author: Scott McMillan
 *  Created: 17 October 1997
 *  Summary: load environment files and piece together a dmEnvironment
 *****************************************************************************/

#include "../dm/dmEnvironment.hpp"
#include "dmLoadFile.h"

//----------------------------------------------------------------------------
//    Summary: Constructor for dmEnvironment class loads in all sorts of stuff
// Parameters: cfg_ptr - ifstream containing the necessary parameters to
//                initialize this class
//    Returns: none
//----------------------------------------------------------------------------
void setEnvironmentParameters(dmEnvironment *env, ifstream &cfg_ptr)
{
   char flname[FILENAME_SIZE];            // filename w/ gridded terrain data

   CartesianVector gravity;
   readConfigParameterLabel(cfg_ptr, "Gravity_Vector");
   cfg_ptr >> gravity[0] >> gravity[1] >> gravity[2];
   env->setGravity(gravity);

   // get terrain model.
   readConfigParameterLabel(cfg_ptr, "Terrain_Data_Filename");
   readFilename(cfg_ptr, flname);
#ifdef DEBUG
   cout << "Terrain data file: " << flname << endl << flush;
#endif
   env->loadTerrainData(flname);

   // get ground characteristics.
   Float constant;
   readConfigParameterLabel(cfg_ptr, "Ground_Planar_Spring_Constant");
   cfg_ptr >> constant;
   env->setGroundPlanarSpringConstant(constant);

   readConfigParameterLabel(cfg_ptr, "Ground_Normal_Spring_Constant");
   cfg_ptr >> constant;
   env->setGroundNormalSpringConstant(constant);

   readConfigParameterLabel(cfg_ptr, "Ground_Planar_Damper_Constant");
   cfg_ptr >> constant;
   env->setGroundPlanarDamperConstant(constant);

   readConfigParameterLabel(cfg_ptr, "Ground_Normal_Damper_Constant");
   cfg_ptr >> constant;
   env->setGroundNormalDamperConstant(constant);

   float u_s, u_k;
   readConfigParameterLabel(cfg_ptr, "Ground_Static_Friction_Coeff");
   cfg_ptr >> u_s;
   readConfigParameterLabel(cfg_ptr, "Ground_Kinetic_Friction_Coeff");
   cfg_ptr >> u_k;

   if (u_k > u_s)
   {
      cerr << "dmEnvironment warning: u_k > u_s friction coefficient.\n";
   }
   env->setFrictionCoeffs(u_s, u_k);

#ifdef DM_HYDRODYNAMICS
   setEnvironmentParameters((dmEnvironment*)env, cfg_ptr);

   Float fluid_density;
   readConfigParameterLabel(cfg_ptr, "Fluid_Density");
   cfg_ptr >> fluid_density;

   env->setFluidDensity(fluid_density);
#endif
}

//----------------------------------------------------------------------------
dmEnvironment *dmuLoadFile_env(char *filename)
{
   ifstream env_ptr(filename);
   if (!env_ptr)
   {
      cerr << "Unable to open dmEnvironment configuration file: "
           << filename << endl;
      exit(1);
   }

   dmEnvironment *env = new dmEnvironment;
   setEnvironmentParameters(env, env_ptr);
   env_ptr.close();
   return env;
}
