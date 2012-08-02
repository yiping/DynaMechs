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
 *     File: glLoadModels.cpp
 *   Author: Scott McMillan
 *  Created:
 *  Summary:
 *****************************************************************************/

#include "../dm/dm.h"
#include "glLoadModels.h"

#include <string>
#include <vector>

vector<pair<string, GLuint> > solid_model_list; // list of display lists

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

//----------------------------------------------------------------------------
GLuint glLoadModel(char *filename)
{
   for (unsigned int i=0; i<solid_model_list.size(); i++)
   {
      if (strcmp(filename, solid_model_list[i].first.c_str()) == 0)
      {
         return solid_model_list[i].second;
      }
   }

   GLuint dlist = 0;

   if (strstr(filename, ".scm"))
      dlist = dmGLLoadFile_scm(filename);
   else if (strstr(filename, ".xan"))
      dlist = dmGLLoadFile_xan(filename);
   else if (strstr(filename, ".cmb"))
      dlist = dmGLLoadFile_cmb(filename);
   else
      cerr << "Error: unknown model format for " << filename << endl;

   if (dlist)
   {
      solid_model_list.push_back(make_pair(string(filename), dlist));
   }

   return dlist;
}

//----------------------------------------------------------------------------
void executeGL_scm(ifstream &data_ptr)
{
   register int i,j;
   int num_faces, num_vertices, num_vertices_tot;
   GLfloat alpha, shininess;
   GLfloat color[4] = {0.0, 0.0, 0.0, 1.0};
   GLfloat vertex[3][3], normal[3];

   glPolygonMode(GL_FRONT, GL_FILL);
   glPolygonMode(GL_BACK, GL_LINE);

   data_ptr >> color[0] >> color[1] >> color[2];  // emission color
   //glMaterialfv(GL_FRONT, GL_EMISSION, color);
   data_ptr >> color[0] >> color[1] >> color[2];  // ambient color
   glMaterialfv(GL_FRONT, GL_AMBIENT, color);
   data_ptr >> color[0] >> color[1] >> color[2];  // diffuse color
   glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
   data_ptr >> color[0] >> color[1] >> color[2];  // specular color
   glMaterialfv(GL_FRONT, GL_SPECULAR, color);
   data_ptr >> shininess;
   glMaterialf(GL_FRONT, GL_SHININESS, shininess);
   data_ptr >> alpha;

   data_ptr >> num_faces;
   data_ptr >> num_vertices_tot;

   for (i=0; i<num_faces; i++)
   {
      data_ptr >> num_vertices;

      for (j=0; j<3; j++)
      {
         data_ptr >> vertex[j][0] >> vertex[j][1] >> vertex[j][2];
      }
      compute_face_normal(vertex[2], vertex[0], vertex[1], normal);

      glBegin(GL_TRIANGLE_STRIP);
      glNormal3fv(normal);
      glVertex3fv(vertex[0]);
      glVertex3fv(vertex[1]);
      glVertex3fv(vertex[2]);

      for (j=3; j<num_vertices; j++)
      {
         data_ptr >> vertex[0][0] >> vertex[0][1] >> vertex[0][2];
         glVertex3fv(vertex[0]);
      }
      glEnd();

      num_vertices_tot -= num_vertices;
   }
}

//----------------------------------------------------------------------------
GLuint dmGLLoadFile_scm(char *model_filename)
{
   ifstream data_ptr;

   data_ptr.open(model_filename);
   if (!data_ptr)
   {
      cerr << "loadModel_scm: Error unable to open data file: "
           << model_filename << endl;
      return 0;
   }

   GLuint dlist_index = glGenLists(1);
   if (dlist_index == 0)
   {
      cerr << "loadModel_scm: Error unable to allocate dlist index." << endl;
      return 0;
   }

   glNewList(dlist_index, GL_COMPILE);
   executeGL_scm(data_ptr);
   glEndList();

   return dlist_index;
}

//----------------------------------------------------------------------------
void executeGL_xan(ifstream &data_ptr)
{
   int i,j;

   // // the following two lines are commented out by Yiping, 01/11/2012
   // // to allow users to switch betwee solid face and wireframe 
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
	//glEnable(GL_BLEND);

//   GLfloat color[4] = {0.0, 0.0, 0.0, 1.0};
//   data_ptr >> color[0] >> color[1] >> color[2];  // emission color
//   //glMaterialfv(GL_FRONT, GL_EMISSION, color);
//   data_ptr >> color[0] >> color[1] >> color[2];  // ambient color
//   glMaterialfv(GL_FRONT, GL_AMBIENT, color);
//   data_ptr >> color[0] >> color[1] >> color[2];  // diffuse color
//   glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
//   data_ptr >> color[0] >> color[1] >> color[2];  // specular color
//   glMaterialfv(GL_FRONT, GL_SPECULAR, color);

//    GLfloat alpha, shininess;
//    data_ptr >> shininess;
//    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
//    data_ptr >> alpha;

//	glEnable(GL_BLEND);
//	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // should be moved to user control code 
	
    // ****
    //since I have switched to use glColorMaterial - yiping
    GLfloat color[4] = {0.0, 0.0, 0.0, .6};
	GLfloat tmpColor[4];
	
    data_ptr >> tmpColor[0] >> tmpColor[1] >> tmpColor[2];  // emission color
    //
    data_ptr >> color[0] >> color[1] >> color[2];  // ambient color
	//
    data_ptr >> tmpColor[0] >> tmpColor[1] >> tmpColor[2];  // diffuse color
    //
    data_ptr >> tmpColor[0] >> tmpColor[1] >> tmpColor[2];  // specular color
    //
    GLfloat alpha, shininess;
    data_ptr >> shininess;
    data_ptr >> alpha;






   GLfloat scale[3];
   data_ptr >> scale[0] >> scale[1] >> scale[2];


   int num_faces, num_vertices;
   data_ptr >> num_vertices >> num_faces;

   GLfloat **vertex = new GLfloat*[num_vertices];
   for (i=0; i<num_vertices; i++)
   {
      vertex[i] = new GLfloat[3];
      data_ptr >> vertex[i][0] >> vertex[i][1] >> vertex[i][2];
      vertex[i][0] *= scale[0];
      vertex[i][1] *= scale[1];
      vertex[i][2] *= scale[2];
   }

   GLint *face_size = new GLint[num_faces];
   GLint **face_index = new GLint*[num_faces];
   for (i=0; i<num_faces; i++)
   {
      data_ptr >> face_size[i];
      face_index[i] = new GLint[face_size[i]];
   }
	
	
	
	
	
	
	

   for (i=0; i<num_faces; i++)
   {
      // There is a much easier way using GL_POLYGON...this is done to
      // develop performer 1.2 loaders that did not have PFGS_POLYS

      if ((face_size[i]/2)*2 != face_size[i]) // odd number of vertices
      {
         for (j=0; j<1+face_size[i]/2; j++)
         {
            data_ptr >> face_index[i][j*2];
         }
         for (j=0; j<face_size[i]/2; j++)
         {
            data_ptr >> face_index[i][face_size[i] - 2*(j+1)];
         }
      }
      else
      {
         for (j=0; j<face_size[i]/2; j++)
         {
            data_ptr >> face_index[i][j*2];
         }
         for (j=0; j<face_size[i]/2; j++)
         {
            data_ptr >> face_index[i][face_size[i] - 2*j - 1];
         }
      }

	   if(!glIsEnabled(GL_BLEND) || 1)
	   {
		   if(face_size[i] == 4)
		   {
			   glColor4f(.1,.1, .1,.4);
			   glBegin(GL_LINE_STRIP);
			   glLineWidth(1.0f);
			   glVertex3fv(vertex[face_index[i][0]]);
			   glVertex3fv(vertex[face_index[i][2]]);
			   glVertex3fv(vertex[face_index[i][3]]);
			   glVertex3fv(vertex[face_index[i][1]]);
			   glVertex3fv(vertex[face_index[i][0]]);
			   glEnd();
		   }
	   }
	   

	   glColor4f(color[0], color[1], color[2],color[3]);
	   
      GLfloat normal[3];
      compute_face_normal(vertex[face_index[i][0]],
                          vertex[face_index[i][1]],
                          vertex[face_index[i][2]],
                          normal);
    

      glBegin(GL_TRIANGLE_STRIP);
      {
         glNormal3fv(normal);
         for (j=0; j<face_size[i]; j++)
         {
            glVertex3fv(vertex[face_index[i][j]]);
         }
      }
      glEnd();
	   
	   
	   
   }
	
	
	//glDisable(GL_BLEND);
}

//----------------------------------------------------------------------------
GLuint dmGLLoadFile_xan(char *model_filename)
{
	cout << "Loading " << model_filename << endl;
	
   ifstream data_ptr;
   data_ptr.open(model_filename);
   if (!data_ptr)
   {
      cerr << "loadModel_xan: Error unable to open data file: "
           << model_filename << endl;
      return 0;
   }

   GLuint dlist_index = glGenLists(1);
   if (dlist_index == 0)
   {
      cerr << "loadModel_xan: Error unable to allocate dlist index." << endl;
      return 0;
   }

   glNewList(dlist_index, GL_COMPILE);
   executeGL_xan(data_ptr);
   glEndList();

   return dlist_index;
}

//----------------------------------------------------------------------------
GLuint dmGLLoadFile_cmb(char *model_filename)
{
   ifstream data_ptr;
   data_ptr.open(model_filename);
   if (!data_ptr)
   {
      cerr << "loadModel_cmb: Error unable to open data file: "
           << model_filename << endl;
      return 0;
   }

   int num_files;
   data_ptr >> num_files;

   if (num_files > 0)
   {
      GLuint dlist_index = glGenLists(1);
      if (dlist_index == 0)
      {
         cerr << "loadModel_cmb: Error unable to allocate dlist index."
              << endl;
         return 0;
      }

      glNewList(dlist_index, GL_COMPILE);

      char filename[256];
      for (int i=0; i<num_files; i++)
      {
         // read a filename inside double quotes
         if (data_ptr.getline(filename, 256, '\042'))
         {
            data_ptr.getline(filename, 256, '\042');
         }

         if (strlen(filename) > 0)
         {
            //cout << "Graphics Model file #" << i << ": " << filename
            //     << endl;

            ifstream data_ptr2;
            data_ptr2.open(filename);

            if (!data_ptr2)
            {
               cerr << "loadModel_cmb: Error unable to open data file: "
                    << filename << endl;
               return 0;
            }

            if (strstr(filename, ".scm"))
               executeGL_scm(data_ptr2);
            else if (strstr(filename, ".xan"))
               executeGL_xan(data_ptr2);
            else
               cerr << "Error: unknown model format for "
                    << filename[i] << endl;

            data_ptr2.close();
         }
      }
      glEndList();

      return dlist_index;
   }
   else
   {
      return 0;
   }
}
