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
 *     File: dmEnvironment.cpp
 *   Author: Scott McMillan
 *  Summary: Class implementation of dmEnvironment
 *****************************************************************************/

#include "dm.h"
#include "dmEnvironment.hpp"

// static variable...there can only be one environment
dmEnvironment* dmEnvironment::m_env = NULL;

//============================================================================
// class dmEnvironment : public dmObject
//============================================================================

//----------------------------------------------------------------------------
//    Summary: Constructor for dmEnvironment class loads in all sorts of stuff
// Parameters: cfg_ptr - ifstream containing the necessary parameters to
//                initialize this class
//    Returns: none
//----------------------------------------------------------------------------
dmEnvironment::dmEnvironment()
      : dmSystem(),
        m_x_dim(0),
        m_y_dim(0),
        m_grid_resolution(0),
        m_depth(NULL),
        m_terrain_filename(NULL),
        m_terrain_model_index(-1),
        m_ground_planar_spring_constant(0.0),
        m_ground_normal_spring_constant(0.0),
        m_ground_planar_damper_constant(0.0),
        m_ground_normal_damper_constant(0.0),
        m_ground_static_friction_coeff(0.0),
        m_ground_kinetic_friction_coeff(0.0)
{
   m_gravity[0] = m_gravity[1] = m_gravity[2] = 0.0;

#ifdef DM_HYDRODYNAMICS
   m_fluid_density = 0;

   for (int i = 0; i < 3; i++)
   {
      m_fluid_vel[i] = 0.0;
      m_fluid_acc[i] = 0.0;
   }
#endif
}

//----------------------------------------------------------------------------
dmEnvironment::~dmEnvironment()
{
   if (m_terrain_filename)
   {
      free(m_terrain_filename);
   }
}

#ifdef DM_HYDRODYNAMICS
//----------------------------------------------------------------------------
unsigned int dmEnvironment::getNumDOFs() const
{
   return 3;
}

//----------------------------------------------------------------------------
void dmEnvironment::setState(Float fluid_vel[], Float fluid_acc[])
{
   for (unsigned int ix = 0; ix<3; ix++)
   {
      m_fluid_vel[ix] = fluid_vel[ix];
      m_fluid_acc[ix] = fluid_acc[ix];
   }
}


//----------------------------------------------------------------------------
void dmEnvironment::getState(Float fluid_vel[], Float fluid_acc[]) const
{
   for (unsigned int ix=0; ix<3; ix++)
   {
      fluid_vel[ix] = m_fluid_vel[ix];
      fluid_acc[ix] = m_fluid_acc[ix];
   }
}

// ---------------------------------------------------------------------
void dmEnvironment::dynamics(Float *qy, Float *qdy)
{
   for (unsigned int ix = 0; ix < 3; ix++)
   {
      m_fluid_vel[ix] = qy[ix];
      qdy[ix] = m_fluid_acc[ix] = qy[ix + getNumDOFs()];
      qdy[ix + getNumDOFs()] = 0;  // derivative of acceleration
   }
}

#else

//----------------------------------------------------------------------------
unsigned int dmEnvironment::getNumDOFs() const
{
   return 0;
}

//----------------------------------------------------------------------------
void dmEnvironment::setState(Float [], Float [])
{
}

//----------------------------------------------------------------------------
void dmEnvironment::getState(Float [], Float []) const
{
}

//----------------------------------------------------------------------------
void dmEnvironment::dynamics(Float *, Float *)
{
}
#endif

// ---------------------------------------------------------------------------
// Function : loadTerrainData
// Purpose  : Function called from constructor to load prismatic terrain data
// Inputs   : filename containing data
// Outputs  :
// ---------------------------------------------------------------------------
void dmEnvironment::loadTerrainData(const char *filename)
{
   register int i, j;

   //m_terrain_filename = strdup(filename);
   m_terrain_filename = (char *) malloc(strlen(filename)+1);
   memcpy(m_terrain_filename, filename, strlen(filename)+1);

   ifstream data_ptr(filename);
   if (!data_ptr)
   {
      cerr << "Unable to open terrain data file: " << filename << endl;
      exit(3);
   }

   // Read the elevation/depth data in meters.
   data_ptr >> m_x_dim >> m_y_dim >> m_grid_resolution;

#ifdef DEBUG
   cout << "Terrain data: (" << m_x_dim << ", " << m_y_dim << ").\n" << flush;
#endif

   // allocate space for and read in depth data;
   m_depth = new Float *[m_x_dim];

   for (i = 0; i < m_x_dim; i++) {
      m_depth[i] = new Float[m_y_dim];

      for (j = 0; j < m_y_dim; j++) {
         data_ptr >> m_depth[i][j];
      }
   }

   data_ptr.close();
}

// ---------------------------------------------------------------------
// Function : getGroundDepth
// Purpose  : Compute ground location and normal from gridded depth data.
// Inputs   : Current contact position wrt ICS
// Outputs  : Ground depth along z-axis, and normal at this point.
// ---------------------------------------------------------------------
Float dmEnvironment::getGroundDepth(CartesianVector contact_pos,
                                    CartesianVector ground_normal) const
{
   register int i;
   Float t, u, depth_var;
   Float norm;
   CartesianVector v1, v2;

   int xindex = (int) (contact_pos[0]/m_grid_resolution);
   int yindex = (int) (contact_pos[1]/m_grid_resolution);

   if (contact_pos[0] < 0.0) {
      xindex--;
   }
   if (contact_pos[1] < 0.0) {
      yindex--;
   }

   t = (contact_pos[0] -
        ((float) xindex*m_grid_resolution))/m_grid_resolution;
   u = (contact_pos[1] -
        ((float) yindex*m_grid_resolution))/m_grid_resolution;

// Walked off the terrain - compute depth equal to closest edge.
   if ((xindex < 0) || (xindex > (m_x_dim - 2)) ||
       (yindex < 0) || (yindex > (m_y_dim - 2)))
   {
      // bogus normal.
      //ground_normal[0] = 0.0;
      //ground_normal[1] = 0.0;
      //ground_normal[2] = -1.0;

      if (yindex < 0) {
         yindex = 0;
         u = 0.0;
      }
      else if (yindex > (m_y_dim - 2)) {
         yindex = m_y_dim - 2;
         u = 1.0;
      }

      if (xindex < 0) {
         xindex = 0;
         t = 0.0;
      }
      else if (xindex > (m_x_dim - 2)) {
         xindex = m_x_dim - 2;
         t = 1.0;
      }
   }
/*
      if (t > u) {
         depth_var = m_depth[xindex][yindex] +
            t*(m_depth[xindex+1][yindex] - m_depth[xindex][yindex]) +
            u*(m_depth[xindex+1][yindex+1] - m_depth[xindex+1][yindex]);
      }
      else {
         depth_var = m_depth[xindex][yindex] +
            u*(m_depth[xindex][yindex+1] - m_depth[xindex][yindex]) +
            t*(m_depth[xindex+1][yindex+1] - m_depth[xindex][yindex+1]);
      }

      return (depth_var);
   }
*/

// On the terrain - compute a valid depth and normal.
   if (t > u)                   // upper-left triangle
   {
      // compute depth of terrain at this planar (x,y) coordinate.
      depth_var = m_depth[xindex][yindex] +
         t*(m_depth[xindex+1][yindex] - m_depth[xindex][yindex]) +
         u*(m_depth[xindex+1][yindex+1] - m_depth[xindex+1][yindex]);

      // compute normal to face.
      v1[0] = -m_grid_resolution;
      v1[1] = 0.0;
      v1[2] = m_depth[xindex][yindex] - m_depth[xindex+1][yindex];

      v2[0] = 0.0;
      v2[1] = m_grid_resolution;
      v2[2] = m_depth[xindex+1][yindex+1] - m_depth[xindex+1][yindex];

      crossproduct(v1, v2, ground_normal);
      norm = sqrt(ground_normal[0]*ground_normal[0] +
                  ground_normal[1]*ground_normal[1] +
                  ground_normal[2]*ground_normal[2]);
      for (i = 0; i < 3; i++) {
         ground_normal[i] /= norm;
      }
   }
   else                         // lower-right triangle
   {
      depth_var = m_depth[xindex][yindex] +
         u*(m_depth[xindex][yindex+1] - m_depth[xindex][yindex]) +
         t*(m_depth[xindex+1][yindex+1] - m_depth[xindex][yindex+1]);

      // compute normal to face.
      v1[0] = 0.0;
      v1[1] = -m_grid_resolution;
      v1[2] = m_depth[xindex][yindex] - m_depth[xindex][yindex+1];

      v2[0] = m_grid_resolution;
      v2[1] = 0.0;
      v2[2] = m_depth[xindex+1][yindex+1] - m_depth[xindex][yindex+1];

      crossproduct(v2, v1, ground_normal);
      norm = sqrt(ground_normal[0]*ground_normal[0] +
                  ground_normal[1]*ground_normal[1] +
                  ground_normal[2]*ground_normal[2]);
      for (i = 0; i < 3; i++) {
         ground_normal[i] /= norm;
      }
   }

   return depth_var;
}

// ---------------------------------------------------------------------
// Function : getGroundElevation
// Purpose  : Compute ground location and normal from gridded elevation data.
// Inputs   : Current contact position wrt ICS
// Outputs  : Ground elevation along z-axis, and normal at this point.
// ---------------------------------------------------------------------
Float dmEnvironment::getGroundElevation(CartesianVector contact_pos,
                                        CartesianVector ground_normal) const
{
   register int i;
   Float t, u, elevation_var;
   Float norm;
   CartesianVector v1, v2;

   int xindex = (int) (contact_pos[0]/m_grid_resolution);
   int yindex = (int) (contact_pos[1]/m_grid_resolution);

   if (contact_pos[0] < 0.0) {
      xindex--;
   }
   if (contact_pos[1] < 0.0) {
      yindex--;
   }

   t = (contact_pos[0] -
        ((float) xindex*m_grid_resolution))/m_grid_resolution;
   u = (contact_pos[1] -
        ((float) yindex*m_grid_resolution))/m_grid_resolution;

// Walked off the terrain - compute elevation equal to closest edge.
   if ((xindex < 0) || (xindex > (m_x_dim - 2)) ||
       (yindex < 0) || (yindex > (m_y_dim - 2)))
   {
      // bogus normal.
      //ground_normal[0] = 0.0;
      //ground_normal[1] = 0.0;
      //ground_normal[2] = -1.0;

      if (yindex < 0) {
         yindex = 0;
         u = 0.0;
      }
      else if (yindex > (m_y_dim - 2)) {
         yindex = m_y_dim - 2;
         u = 1.0;
      }

      if (xindex < 0) {
         xindex = 0;
         t = 0.0;
      }
      else if (xindex > (m_x_dim - 2)) {
         xindex = m_x_dim - 2;
         t = 1.0;
      }
   }

// On the terrain - compute a valid elevation and normal.
   if (t > u)                   // upper-left triangle
   {
      // compute elevation of terrain at this planar (x,y) coordinate.
      elevation_var = m_depth[xindex][yindex] +
         t*(m_depth[xindex+1][yindex] - m_depth[xindex][yindex]) +
         u*(m_depth[xindex+1][yindex+1] - m_depth[xindex+1][yindex]);

      // compute normal to face.
      v1[0] = -m_grid_resolution;
      v1[1] = 0.0;
      v1[2] = m_depth[xindex][yindex] - m_depth[xindex+1][yindex];

      v2[0] = 0.0;
      v2[1] = m_grid_resolution;
      v2[2] = m_depth[xindex+1][yindex+1] - m_depth[xindex+1][yindex];

      crossproduct(v2, v1, ground_normal);
      norm = sqrt(ground_normal[0]*ground_normal[0] +
                  ground_normal[1]*ground_normal[1] +
                  ground_normal[2]*ground_normal[2]);
      for (i = 0; i < 3; i++) {
         ground_normal[i] /= norm;
      }
   }
   else                         // lower-right triangle
   {
      elevation_var = m_depth[xindex][yindex] +
         u*(m_depth[xindex][yindex+1] - m_depth[xindex][yindex]) +
         t*(m_depth[xindex+1][yindex+1] - m_depth[xindex][yindex+1]);

      // compute normal to face.
      v1[0] = 0.0;
      v1[1] = -m_grid_resolution;
      v1[2] = m_depth[xindex][yindex] - m_depth[xindex][yindex+1];

      v2[0] = m_grid_resolution;
      v2[1] = 0.0;
      v2[2] = m_depth[xindex+1][yindex+1] - m_depth[xindex][yindex+1];

      crossproduct(v1, v2, ground_normal);
      norm = sqrt(ground_normal[0]*ground_normal[0] +
                  ground_normal[1]*ground_normal[1] +
                  ground_normal[2]*ground_normal[2]);
      for (i = 0; i < 3; i++) {
         ground_normal[i] /= norm;
      }
   }

   return elevation_var;
}
