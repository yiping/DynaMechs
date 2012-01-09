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
 *     File: carts.cpp
 *   Author: Scott McMillan
 *  Created: 20 March 1997
 *  Summary: DynaMechs GLUT example testing multiple systems
 *****************************************************************************/

#include <GL/glut.h>

#include <dmTime.h>
#include <dmGLMouse.hpp>
#include <dmGLPolarCamera_zup.hpp>

#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmRevoluteLink.hpp>
#include <dmMobileBaseLink.hpp>
#include <dmEnvironment.hpp>
#include <dmIntegrator.hpp>
#include <dmIntegRK4.hpp>

#include <dmu.h>

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;
bool  paused_flag = true;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;
int motion_plan_rate;       // fixed rate of 100Hz
int motion_plan_count = 0;  // counter for motion planning updates

float cmd_direction = 0.0;
float cmd_speed = 0.0;

int num_robots = 1;

class CartController
{
public:
   CartController(dmArticulation *robot);
   ~CartController();

   void computeControl(float time);

private:
   dmRevoluteLink *m_wheel[4];

   Float m_desired_joint_pos[4];
};

struct System
{
   dmArticulation *robot;
   CartController *controller;
};

dmIntegrator *G_integrator;
System       *G_system;

//----------------------------------------------------------------------------
CartController::CartController(dmArticulation *robot)
{
   cerr << "initControl():" << endl;

   dmObject *obj;
   obj = dmuFindObject("wheel1", robot);
   m_wheel[0] = dynamic_cast<dmRevoluteLink*>(obj);

   obj = dmuFindObject("wheel2", robot);
   m_wheel[1] = dynamic_cast<dmRevoluteLink*>(obj);

   obj = dmuFindObject("wheel3", robot);
   m_wheel[2] = dynamic_cast<dmRevoluteLink*>(obj);

   obj = dmuFindObject("wheel4", robot);
   m_wheel[3] = dynamic_cast<dmRevoluteLink*>(obj);

   for (int i=0; i<4; i++)
   {
      Float dummy;
      m_wheel[i]->getState(&m_desired_joint_pos[i], &dummy);

      cerr << "joint " << i << ": pos = " << m_desired_joint_pos[i] << endl;
   }
}

//----------------------------------------------------------------------------
CartController::~CartController()
{
}

//----------------------------------------------------------------------------
void CartController::computeControl(float time)
{
   Float joint_input;
   Float joint_pos;
   Float joint_vel;

   Float delta_pos = 20.0*sin(0.40*time);

   for (int i=0; i<4; i++)
   {
      m_wheel[i]->getState(&joint_pos, &joint_vel);
      joint_input = 75.0*(m_desired_joint_pos[i] + delta_pos - joint_pos) -
         2.0*joint_vel;

      m_wheel[i]->setJointInput(&joint_input);
   }
}


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

   for (int i=0; i<num_robots; i++)
   {
      glPushAttrib(GL_ALL_ATTRIB_BITS);
      G_system[i].robot->draw();
      glPopAttrib();
   }

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

   camera->setPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 200.0);

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
         if (cmd_speed < 0.0f) cmd_speed = 0.0f;
         break;
   }
}


//----------------------------------------------------------------------------
void updateSim()
{

   if (!paused_flag)
   {
      for (int j=0; j<render_rate; j++)
      {
         for (int i=0; i<num_robots; i++)
         {
            G_system[i].controller->computeControl(sim_time);
         }

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

   if (elapsed_time > 2.5)
   {
      rtime += elapsed_time;
      cerr << "time/real_time: " << sim_time << '/' << rtime
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
   char *filename = "carts.cfg";
   if (argc > 1)
   {
      num_robots = atoi(argv[1]);
   }
   if (argc > 2)
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

   G_integrator = new dmIntegRK4;
   G_system = new System[num_robots];

   for (i=0; i<num_robots; i++)
   {
      dmSystem *sys = dmuLoadFile_dm(robot_flname);
      G_system[i].robot = dynamic_cast<dmArticulation*>(sys);

      dmObject *obj;
      obj = dmuFindObject("refmember", G_system[i].robot);
      dmMobileBaseLink *ref =  dynamic_cast<dmMobileBaseLink*>(obj);

      if (ref == NULL)
      {
         cerr << "main() error: couldn't find 'refmember' object." << endl;
      }

      Float q[7]; // [quat pos]
      SpatialVector vel;
      ref->getState(q, vel);
      q[4] += 5.0*i;
      ref->setState(q, vel);

      G_system[i].controller = new CartController(G_system[i].robot);
      G_integrator->addSystem(G_system[i].robot);
   }

   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutSpecialFunc(processSpecialKeys);
   glutDisplayFunc(display);
   glutIdleFunc(updateSim);

   dmGetSysTime(&last_tv);

   cout << endl;
   cout << "usage: " << argv[0] << " [num_robots [config_file]]" << endl;
   cout << "p - toggles dynamic simulation" << endl;
   cout << "Use mouse to rotate/move/zoom the camera" << endl << endl;

   glutMainLoop();
   return 0;             /* ANSI C requires main to return int. */
}
