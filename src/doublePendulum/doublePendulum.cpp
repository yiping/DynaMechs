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
 *     File: pendulum.cpp
 *   Author: Scott McMillan
 *  Created: 29 July 1999
 *  Summary: Test the static base case for DynaMechs 3.0
 *****************************************************************************/

#include <GLUT/glut.h>

#include <dmTime.h>
#include <dmGLMouse.hpp>
#include <dmGLPolarCamera_zup.hpp>


#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmMDHLink.hpp>
#include <dmEnvironment.hpp>
#include <dmIntegRK4.hpp>

#include <dmu.h>

dmMDHLink * link1, * link2, *link3;

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;
bool  paused_flag = true;

dmSystem *G_robot;
dmIntegRK4 *G_integrator;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;
int motion_plan_rate;       // fixed rate of 100Hz
int motion_plan_count = 0;  // counter for motion planning updates

float cmd_direction = 0.0;
float cmd_speed = 0.0;

//----------------------------------------------------------------------------
void myinit (void)
{
   GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
   GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
//     light_position is NOT default value
   GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

   glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
   glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
   glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
   glLightfv (GL_LIGHT0, GL_POSITION, light_position);

   glEnable (GL_LIGHTING);
   glEnable (GL_LIGHT0);
   glDepthFunc(GL_LESS);
   glEnable(GL_DEPTH_TEST);

   glShadeModel(GL_FLAT);
   glEnable(GL_CULL_FACE);
   glCullFace(GL_BACK);
}

//----------------------------------------------------------------------------
void display (void)
{
   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode (GL_MODELVIEW);
   glPushMatrix ();

   // ===============================================================
   (dmEnvironment::getEnvironment())->draw();

   glPushAttrib(GL_ALL_ATTRIB_BITS);
   G_robot->draw();
   glPopAttrib();
   // ===============================================================

   glDisable (GL_LIGHTING);

   glBegin(GL_LINES);
   glColor3f(1.0, 0.0, 0.0);
   glVertex3f(2.0, 0.0, 0.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glBegin(GL_LINES);
   glColor3f(0.0, 1.0, 0.0);
   glVertex3f(0.0, 2.0, 0.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glBegin(GL_LINES);
   glColor3f(0.0, 0.0, 1.0);
   glVertex3f(0.0, 0.0, 2.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glEnable (GL_LIGHTING);

   glPopMatrix ();

   glFlush ();
   glutSwapBuffers();
}

//----------------------------------------------------------------------------
void myReshape(int w, int h)
{
   glViewport (0, 0, w, h);
   mouse->win_size_x = w;
   mouse->win_size_y = h;

   //if (w <= h)
   //    glOrtho (-2.5, 2.5, -2.5*(GLfloat)h/(GLfloat)w,
   //        2.5*(GLfloat)h/(GLfloat)w, -10.0, 10.0);
   //else
   //    glOrtho (-2.5*(GLfloat)w/(GLfloat)h,
   //        2.5*(GLfloat)w/(GLfloat)h, -2.5, 2.5, -10.0, 10.0);

   camera->setPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 200.0);

   //glMatrixMode (GL_MODELVIEW);
   //glLoadIdentity();
   //glTranslatef(0,0,-10.0);

   camera->setViewMat(view_mat);
   camera->applyView();
}

//----------------------------------------------------------------------------
void processKeyboard(unsigned char key, int, int)
{
   switch (key)
   {
      case 27:
         glutDestroyWindow(glutGetWindow());
         exit(1);
         break;

      case 'p':
         paused_flag = !paused_flag;
         break;
   }
}


//----------------------------------------------------------------------------
void processSpecialKeys(int key, int, int)
{
   switch (key)
   {
      case GLUT_KEY_LEFT:
         cmd_direction += 5.0;
         if (cmd_direction > 180.0) cmd_direction -= 360.0;
         break;
      case GLUT_KEY_RIGHT:
         cmd_direction -= 5.0;
         if (cmd_direction < -180.0) cmd_direction += 360.0;
         break;
      case GLUT_KEY_UP:
         cmd_speed += 0.01f;
         if (cmd_speed > 0.25f) cmd_speed = 0.25f;
         break;
      case GLUT_KEY_DOWN:
         cmd_speed -= 0.01f;
         if (cmd_speed < 0.0) cmd_speed = 0.0;
         break;
   }
}


//----------------------------------------------------------------------------
void updateSim()
{

   if (!paused_flag)
   {
      for (int i=0; i<render_rate; i++)
      {
		  Float q[1];
		  Float qd[1];
		  Float input[1];
		  
		  // Simple Joint PD
		  link1->getState(q, qd);
		  input[0]=35*4*(3.141-q[0])+7*4*(0-qd[0]);
		  link1->setJointInput(input);
		  
		  link2->getState(q, qd);
		  input[0]=15*4*(0-q[0])+3*4*(0-qd[0]);
		  link2->setJointInput(input);

		  link3->getState(q, qd);
		  input[0]=15*4*(0-q[0])+3*4*(0-qd[0]);
		  link3->setJointInput(input);
		  
         G_integrator->simulate(idt);
         sim_time += idt;
      }
   }

   camera->update(mouse);
   camera->applyView();

   display();

   // compute render rate
   timer_count++;
   dmGetSysTime(&tv);
   double elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
      (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));


	dmArticulation * art = (dmArticulation*) G_robot;
	//art.setArticulation((dmArticulation*) G_robot);
	
   if (elapsed_time > 0)
   {
	   
	   VectorXF q(3), qd(3);
	   q << 0 , 3.1415926535897932384/4, -3.1415926535897932384/2;
	   qd << .1 , .7 , 0;
	   
	   art->setState(q.data(),qd.data());
	   //art.obtainState();
	   art->computeH();
	   //art.computeCandG();
	   //art.computeQdd();
	   
	   //art.computedHdQ();
	   
	   /*
	   dmGetSysTime(&last_tv);
	   for (int j=0; j<1000; j++) {
		   art.obtainState();
	   }
	   dmGetSysTime(&tv);
	   
	   elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	   (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));
	   cout << elapsed_time << endl;
	   
	   
	   dmGetSysTime(&last_tv);
	   for (int j=0; j<1000; j++) {
		   art.computeH();
	   }
	   dmGetSysTime(&tv);
	   
	   elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	   (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));
	   cout << elapsed_time << endl;
	   
	   dmGetSysTime(&last_tv);
	   for (int j=0; j<1000; j++) {
		   art.computeCandG();
	   }
	   dmGetSysTime(&tv);
	   
	   elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	   (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));
	   cout << elapsed_time << endl;
	   */
	   /*
	   dmGetSysTime(&last_tv);
	   for (int j=0; j<1000; j++) {
		   art.computeQdd();
	   }
	   dmGetSysTime(&tv);
	   
	   elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	   (1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));
	   cout << elapsed_time << endl;*/
	   
	   exit(0);
	   
      rtime += elapsed_time;
      cout << "time/real_time: " << sim_time << '/' << rtime
           << "  frame_rate: " << (double) timer_count/elapsed_time << endl;

      timer_count = 0;
      last_tv.tv_sec = tv.tv_sec;
      last_tv.tv_nsec = tv.tv_nsec;
   }
}

//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
   int i, j;

   glutInit(&argc, argv);

   //=========================
   char *filename = "pendulum.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640, 480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("DynaMechs Example");

   myinit();
   mouse = dmGLMouse::dmInitGLMouse();

   for (i=0; i<4; i++)
   {
      for (j=0; j<4; j++)
      {
         view_mat[i][j] = 0.0;
      }
      view_mat[i][i] = 1.0;
   }
   view_mat[3][2] = -10.0;
   camera = new dmGLPolarCamera_zup();
   camera->setRadius(30.0);
   camera->setCOI(10.0, 10.0, 5.0);
   camera->setTranslationScale(0.02f);

   // load robot stuff
   ifstream cfg_ptr;
   cfg_ptr.open(filename);

   // Read simulation timing information.
   readConfigParameterLabel(cfg_ptr,"Integration_Stepsize");
   cfg_ptr >> idt;
   if (idt <= 0.0)
   {
      cerr << "main error: invalid integration stepsize: " << idt << endl;
      exit(3);
   }
   motion_plan_rate  = (int) (0.5 + 0.01/idt);  // fixed rate of 100Hz

   readConfigParameterLabel(cfg_ptr,"Display_Update_Rate");
   cfg_ptr >> render_rate;
   if (render_rate < 1) render_rate = 1;

// ===========================================================================
// Initialize DynaMechs environment - must occur before any linkage systems
   char env_flname[FILENAME_SIZE];
   readConfigParameterLabel(cfg_ptr,"Environment_Parameter_File");
   readFilename(cfg_ptr, env_flname);
   dmEnvironment *environment = dmuLoadFile_env(env_flname);
   environment->drawInit();
   dmEnvironment::setEnvironment(environment);

// ===========================================================================
// Initialize a DynaMechs linkage system
   char robot_flname[FILENAME_SIZE];
   readConfigParameterLabel(cfg_ptr,"Robot_Parameter_File");
   readFilename(cfg_ptr, robot_flname);

   G_robot = dmuLoadFile_dm(robot_flname);
	
	//G_robot->
	
	dmArticulation * artic;
	artic = (dmArticulation*) G_robot;
	
	link1 = dynamic_cast<dmMDHLink*>(dmuFindObject("link1", artic));
	link2 = dynamic_cast<dmMDHLink*>(dmuFindObject("link2", artic));
	link3 = dynamic_cast<dmMDHLink*>(dmuFindObject("link3", artic));
	
   G_integrator = new dmIntegRK4();
   G_integrator->addSystem(G_robot);

   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutSpecialFunc(processSpecialKeys);
   glutDisplayFunc(display);
   glutIdleFunc(updateSim);

   dmGetSysTime(&last_tv);

   cout << endl;
   cout << "p - toggles dynamic simulation" << endl;
   cout << "Use mouse to rotate/move/zoom the camera" << endl << endl;

   glutMainLoop();
   return 0;             /* ANSI C requires main to return int. */
}
