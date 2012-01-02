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
 *     File: gldraw.cpp
 *   Author: Scott McMillan
 *  Created: 24 March 1997
 *  Summary:
 *****************************************************************************/

#include "dm.h"
#include "dmArticulation.hpp"
#include "dmLink.hpp"
#include "dmZScrewTxLink.hpp"
#include "dmStaticRootLink.hpp"
#include "dmMobileBaseLink.hpp"
#include "dmSphericalLink.hpp"
#include "dmQuaternionLink.hpp"
#include "dmMDHLink.hpp"
#include "dmRevoluteLink.hpp"
#include "dmPrismaticLink.hpp"
#include "dmContactModel.hpp"
#include "dmEnvironment.hpp"
#include "dmTreadmill.hpp"

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

// yeah I know this is in other places but I really want to limit its scope
const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h

//----------------------------------------------------------------------------
inline void compute_face_normal(float v0[3], float v1[3], float v2[3],
                                float normal[3])
{
   float a[3], b[3];
   register int i;

   for (i=0; i<3; i++)
   {
      a[i] = v1[i] - v0[i];
      b[i] = v2[i] - v0[i];
   }

   //cross(a, b, normal);
   normal[0] = a[1]*b[2] - a[2]*b[1];
   normal[1] = a[2]*b[0] - a[0]*b[2];
   normal[2] = a[0]*b[1] - a[1]*b[0];

   //normalize(normal);
   float norm = sqrt(normal[0]*normal[0] +
                     normal[1]*normal[1] +
                     normal[2]*normal[2]);

   if (norm > 0.0)
   {
      normal[0] /= norm;
      normal[1] /= norm;
      normal[2] /= norm;
   }
}

//============================================================================
// Initialize DynaMechs/OpenGL drawing routines
//============================================================================

//----------------------------------------------------------------------------
void dmEnvironment::drawInit()
{
   register int i, j;

   GLfloat vertex[3][3], normal[3];

   // read in and allocate depth data

   m_terrain_model_index = glGenLists(1);

   if (m_terrain_model_index == 0)
   {
      cerr << "loadModel_grid: Error unable to allocate dlist index." << endl;
   }

   glNewList(m_terrain_model_index, GL_COMPILE);
   {
      glPolygonMode(GL_FRONT, GL_LINE); //FILL);
      glPolygonMode(GL_BACK, GL_LINE);

      GLfloat color[4] = {0.5,0.5,1.0,1.0};
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);

      for (j=0; j<m_y_dim-1; j++)
      {
         glBegin(GL_TRIANGLE_STRIP);
         {
            for (i=0; i<m_x_dim; i++)
            {
               vertex[0][0] = ((GLfloat) i)*m_grid_resolution;
               vertex[0][1] = ((GLfloat) j + 1.0)*m_grid_resolution;
               vertex[0][2] = m_depth[i][j+1];

               if (i > 0)
               {
                  vertex[1][0] = ((GLfloat) i - 1.0)*m_grid_resolution;
                  vertex[1][1] = ((GLfloat) j + 1.0)*m_grid_resolution;
                  vertex[1][2] = m_depth[i-1][j+1];

                  vertex[2][0] = ((GLfloat) i - 1.0)*m_grid_resolution;
                  vertex[2][1] = ((GLfloat) j)*m_grid_resolution;
                  vertex[2][2] = m_depth[i-1][j];

                  compute_face_normal(vertex[1], vertex[2], vertex[0], normal);
                  glNormal3fv(normal);
               }
               glVertex3fv(vertex[0]);

               vertex[1][0] = ((GLfloat) i)*m_grid_resolution;
               vertex[1][1] = ((GLfloat) j)*m_grid_resolution;
               vertex[1][2] = m_depth[i][j];

               if (i > 0)
               {
                  compute_face_normal(vertex[1], vertex[0], vertex[2], normal);
                  glNormal3fv(normal);
               }
               glVertex3fv(vertex[1]);
            }
         }
         glEnd();
      }
   }
   glEndList();
}

//============================================================================
// draw environment and robot
//============================================================================

//----------------------------------------------------------------------------
void dmArticulation::draw() const
{
   glPushMatrix();

   glTranslatef(m_p_ICS[0], m_p_ICS[1], m_p_ICS[2]);

   float len = sqrt(m_quat_ICS[0]*m_quat_ICS[0] +
                    m_quat_ICS[1]*m_quat_ICS[1] +
                    m_quat_ICS[2]*m_quat_ICS[2]);
   if (len > 1.0e-6)
   {
      float angle = 2.0*atan2(len, m_quat_ICS[3]);
      glRotatef(angle*RADTODEG,
                m_quat_ICS[0]/len, m_quat_ICS[1]/len, m_quat_ICS[2]/len);
   }

   // render some sort of base element
   if (getUserData())
      glCallList(*((GLuint *) getUserData()));

   for (unsigned int j=0; j<m_link_list.size(); j++)
   {
      if (m_link_list[j]->parent == NULL)
      {
         glPushMatrix();

         // draw base link
         m_link_list[j]->link->draw();

         // recurse through the children
         for (unsigned int i=0; i<m_link_list[j]->child_list.size(); i++)
         {
            glPushMatrix();
            drawTraversal(m_link_list[j]->child_list[i]);
            glPopMatrix();
         }

         glPopMatrix();
      }
   }

   glPopMatrix();
}

//----------------------------------------------------------------------------
void dmArticulation::drawTraversal(LinkInfoStruct *node) const
{
   if (node && node->parent)
   {
      node->link->draw();

      for (unsigned int i=0; i<node->child_list.size(); i++)
      {
         if (node->child_list.size() > 1)
         {
            glPushMatrix();
            drawTraversal(node->child_list[i]);
            glPopMatrix();
         }
         else
         {
            drawTraversal(node->child_list[i]);
         }
      }
   }
}


//----------------------------------------------------------------------------
void dmZScrewTxLink::draw() const
{
   glTranslatef(0.0, 0.0, m_dMDH);
   glRotatef(m_thetaMDH*RADTODEG, 0.0, 0.0, 1.0);
}

//----------------------------------------------------------------------------
void dmStaticRootLink::draw() const
{
   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmRevoluteLink::draw() const
{
   // set static portion of the MDH transformation.
   if (m_alphaMDH != 0.0)
   {
      glRotatef(m_alphaMDH*RADTODEG, 1.0, 0.0, 0.0);
   }

   if ((m_aMDH != 0.0) || (m_dMDH != 0.0))
   {
      glTranslatef(m_aMDH, 0.0, m_dMDH);
   }

   // set dynamic z-axis transformation.
   glRotatef(m_thetaMDH*RADTODEG, 0.0, 0.0, 1.0);

   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmPrismaticLink::draw() const
{
   // set static portion of the MDH transformation.
   if (m_alphaMDH != 0.0)
   {
      glRotatef(m_alphaMDH*RADTODEG, 1.0, 0.0, 0.0);
   }

   if ((m_aMDH != 0.0) || (m_dMDH != 0.0))
   {
      glTranslatef(m_aMDH, 0.0, m_dMDH);
   }

   if (m_thetaMDH != 0.0)
   {
      glRotatef(m_thetaMDH*RADTODEG, 0.0, 0.0, 1.0);
   }

   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmSphericalLink::draw() const
{
   glTranslatef(m_p[0], m_p[1], m_p[2]);
   glRotatef(m_q[2]*RADTODEG, 0.0, 0.0, 1.0);
   glRotatef(m_q[1]*RADTODEG, 0.0, 1.0, 0.0);
   glRotatef(m_q[0]*RADTODEG, 1.0, 0.0, 0.0);

   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmQuaternionLink::draw() const
{
   glTranslatef(m_p[0], m_p[1], m_p[2]);

   float len = sqrt(m_q[0]*m_q[0] + m_q[1]*m_q[1] + m_q[2]*m_q[2]);
   if (len > 1.0e-6)
   {
      float angle = 2.0*atan2(len, m_q[3]);
      glRotatef(angle*RADTODEG, m_q[0]/len, m_q[1]/len, m_q[2]/len);
   }

   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmMobileBaseLink::draw() const
{
   glTranslatef(m_p[0], m_p[1], m_p[2]);

   float len = sqrt(m_quat[0]*m_quat[0] +
                    m_quat[1]*m_quat[1] +
                    m_quat[2]*m_quat[2]);
   if (len > 1.0e-6)
   {
      float angle = 2.0*atan2(len, m_quat[3]);
      glRotatef(angle*RADTODEG, m_quat[0]/len, m_quat[1]/len, m_quat[2]/len);
   }

   glCallList(*((GLuint *) getUserData()));
}

//----------------------------------------------------------------------------
void dmContactModel::draw() const
{
}

//----------------------------------------------------------------------------
void dmEnvironment::draw() const
{
   glCallList(m_terrain_model_index);
}

//----------------------------------------------------------------------------
void dmTreadmill::draw() const
{
   dmEnvironment::draw();

   glBegin(GL_QUAD_STRIP);
   float dxl = m_half_width*m_left[0];
   float dyl = m_half_width*m_left[1];
   float dzl = m_half_width*m_left[2];

   float dxf = m_half_length*m_forward[0];
   float dyf = m_half_length*m_forward[1];
   float dzf = m_half_length*m_forward[2];

   glVertex3f(m_position[0] + dxl - dxf,
              m_position[1] + dyl - dyf,
              m_position[2] + dzl - dzf);
   glVertex3f(m_position[0] - dxl - dxf,
              m_position[1] - dyl - dyf,
              m_position[2] - dzl - dzf);

   float step = 2.0f*m_half_length/10.f;
   float offset = m_q - ((int)(m_q/step))*step;
   if (offset < 0) offset += step;

   for (unsigned int ix = 0; ix < 10; ++ix)
   {
      float distance = offset + ix*step - m_half_length;
      float dx = distance*m_forward[0];
      float dy = distance*m_forward[1];
      float dz = distance*m_forward[2];
      glVertex3f(m_position[0] + dxl + dx,
                 m_position[1] + dyl + dy,
                 m_position[2] + dzl + dz);
      glVertex3f(m_position[0] - dxl + dx,
                 m_position[1] - dyl + dy,
                 m_position[2] - dzl + dz);
   }

   glVertex3f(m_position[0] + dxl + dxf,
              m_position[1] + dyl + dyf,
              m_position[2] + dzl + dzf);
   glVertex3f(m_position[0] - dxl + dxf,
              m_position[1] - dyl + dyf,
              m_position[2] - dzl + dzf);
   glEnd();
}
