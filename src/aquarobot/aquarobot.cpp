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
 *     File: aquarobot.cpp
 *   Author: Scott McMillan
 *  Created: 20 March 1997
 *  Summary:
 *****************************************************************************/

#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmu.h>
#include <dmGL.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <dmTime.h>
#include <dmGLMouse.hpp>
#include <dmGLPolarCamera_zup.hpp>

#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmIntegRK4.hpp>
#include <dmEnvironment.hpp>
#include <dmMDHLink.hpp>
#include <dmMobileBaseLink.hpp>


// ----------------------  Gait algorithm stuff -------------------------
#include <GaitAlgorithm.h> // Kenji Suzuki's gait algorithm
#include <Robot.hpp>

const float MAX_SPEED = 0.25;

Robot new_robot;
Float desiredJointPos[6][3];
float motion_command[3] = {0.0, 0.0, 0.0};  // v_x, v_y, omega_z

int motion_plan_rate;       // fixed rate of 100Hz
int motion_plan_count = 0;  // counter for motion planning updates

float cmd_direction = 0.0;
float cmd_speed = 0.0;
// ----------------------------------------------------------------------

const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;
bool paused_flag = true;

dmArticulation *G_robot;
dmIntegRK4 *G_integrator;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;

//-------------------- control interface stuff ---------------------
dmMobileBaseLink *ref_member;

int num_arts = 6;
int num_links[6] = {3, 3, 3, 3, 3, 3};
dmMDHLink *robot_link[6][3];
Float desired_joint_pos[6][3];

//----------------------------------------------------------------------------
void initAquaControl(dmArticulation *robot)
{
   int i,j;

   cerr << "initAquaControl():" << endl;

   ref_member = dynamic_cast<dmMobileBaseLink*>(dmuFindObject("refmember",
                                                              robot));

   for (j=0; j<num_arts; j++)
   {
      for (i=0; i<num_links[j]; i++)
      {
         char label[8];
         sprintf(label, "%d%d", j, i);
         robot_link[j][i] = dynamic_cast<dmMDHLink*>(dmuFindObject(label,
                                                                   robot));
      }
   }
}


/*****************************************************************************
 * Function : motor_control                                                  *
 * Purpose  : Compute the motor voltages from PD servo loops.                *
 * Inputs   : Desired and actual joint positions, and actual joint velocities*
 * Outputs  : Motor voltages (jointInput).                                   *
 *****************************************************************************/

#define DAMPING_CONSTANT 0.0
#define SPRING_CONSTANT  1500.0
#define MAX_VOLTAGE      75.0

void computeAquaControl(Float jointPosDesired[6][3])
{
   register int i, k;
   Float joint_input[1];
   Float joint_pos[1];
   Float joint_vel[1];

   // leg joint controllers
   for (k=0; k<6; k++)
   {
      for (i=0; i<3; i++)
      {
         robot_link[k][i]->getState(joint_pos, joint_vel);

         joint_input[0] = -DAMPING_CONSTANT*joint_vel[0] -
            SPRING_CONSTANT*(joint_pos[0] - jointPosDesired[k][i]);

         //cerr << jointPosDesired[k][i] << ' ';

         if (joint_input[0] > MAX_VOLTAGE)
            joint_input[0] = MAX_VOLTAGE;
         else if (joint_input[0] < -MAX_VOLTAGE)
            joint_input[0] = -MAX_VOLTAGE;

         robot_link[k][i]->setJointInput(joint_input);
      }
      //cerr << endl;
   }
   //cerr << endl;
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
Float q[7];
SpatialVector dummy;

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

   // output a speed/direction command vector
   ref_member->getState(q, dummy);

   glPushMatrix();
   {
      glTranslatef(q[4], q[5], q[6]);

      float len = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2]);
      if (len > 1.0e-6)
      {
         float angle = 2.0*atan2(len, q[3]);
         glRotatef(angle*RADTODEG, q[0]/len, q[1]/len, q[2]/len);
      }

      glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.8f);
      glVertex3f(cos(cmd_direction/RADTODEG)*cmd_speed*3.0,
                 sin(cmd_direction/RADTODEG)*cmd_speed*3.0,
                 0.8f);
      glEnd();
   }
   glPopMatrix();

   glEnable (GL_LIGHTING);

   glPopMatrix ();

   glFlush ();
   glutSwapBuffers();
}

//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
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
//    Summary:
// Parameters:
//    Returns:
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
//    Summary:
// Parameters:
//    Returns:
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
         if (cmd_speed > MAX_SPEED) cmd_speed = MAX_SPEED;
         break;
      case GLUT_KEY_DOWN:
         cmd_speed -= 0.01f;
         if (cmd_speed < 0.0) cmd_speed = 0.0;
         break;
   }
}


//===========================================================================
// FUNCTION: void interface_Gait2DynaMechs(Robot &robot)
// PURPOSE:  interface between "triple" array(for Dyanamics)
//                         and "Robot" class(for gait planning).
// AUTHOR:   Kenji Suzuki.
// DATE:     October 18, 1993
// COMMENT:
//===========================================================================

void interface_Gait2DynaMechs(Robot &robot,
                              EulerAngles refPose, CartesianVector refPos,
                              Float jointPos[6][3])
{
    refPose[2] = robot.body.W_body_pos.azimuth;
    refPose[1] = robot.body.W_body_pos.elevation;
    refPose[0] = robot.body.W_body_pos.roll;

    refPos[0] = robot.body.W_body_pos.x/100.0;
    refPos[1] = robot.body.W_body_pos.y/100.0;
    refPos[2] = robot.body.W_body_pos.z/100.0;

    // LEG1
    jointPos[0][0] = robot.leg1.link1.joint_angle_pos;
    jointPos[0][1] = robot.leg1.link2.joint_angle_pos;
    jointPos[0][2] = robot.leg1.link3.joint_angle_pos;
//    jointPos[0][3][2] = -M_PI/2.0 - jointPos[0][1][0] - jointPos[0][2][0];

    // LEG2
    jointPos[1][0] = robot.leg2.link1.joint_angle_pos;
    jointPos[1][1] = robot.leg2.link2.joint_angle_pos;
    jointPos[1][2] = robot.leg2.link3.joint_angle_pos;
//    jointPos[1][3][2] = -M_PI/2.0 - jointPos[1][1][0] - jointPos[1][2][0];

    // LEG3
    jointPos[2][0] = robot.leg3.link1.joint_angle_pos;
    jointPos[2][1] = robot.leg3.link2.joint_angle_pos;
    jointPos[2][2] = robot.leg3.link3.joint_angle_pos;
//    jointPos[2][3][2] = -M_PI/2.0 - jointPos[2][1][0] - jointPos[2][2][0];

    // LEG4
    jointPos[3][0] = robot.leg4.link1.joint_angle_pos;
    jointPos[3][1] = robot.leg4.link2.joint_angle_pos;
    jointPos[3][2] = robot.leg4.link3.joint_angle_pos;
//    jointPos[3][3][2] = -M_PI/2.0 - jointPos[3][1][0] - jointPos[3][2][0];

    // LEG5
    jointPos[4][0] = robot.leg5.link1.joint_angle_pos;
    jointPos[4][1] = robot.leg5.link2.joint_angle_pos;
    jointPos[4][2] = robot.leg5.link3.joint_angle_pos;
//    jointPos[4][3][2] = -M_PI/2.0 - jointPos[4][1][0] - jointPos[4][2][0];

    // LEG6
    jointPos[5][0] = robot.leg6.link1.joint_angle_pos;
    jointPos[5][1] = robot.leg6.link2.joint_angle_pos;
    jointPos[5][2] = robot.leg6.link3.joint_angle_pos;
//    jointPos[5][3][2] = -M_PI/2.0 - jointPos[5][1][0] - jointPos[5][2][0];
}

//----------------------------------------------------------------------------
void updateSim()
{
   if (!paused_flag)
   {
      for (int i=0; i<render_rate; i++)
      {
         // Kenji Suzuki/Kan Yoneda's motion planning algorithm
         //              for dynamic sim.
         // Can run more often than rendering and less often than
         //              the dynamic simulation.
         if (motion_plan_count%motion_plan_rate == 0)
         {
            // Note motion_planning requires cm/sec velocity commands
            // commands are run through a simple low pass filter.
            motion_command[0] = cos(cmd_direction*DEGTORAD)*cmd_speed*100.0;
            motion_command[1] = sin(cmd_direction*DEGTORAD)*cmd_speed*100.0;
            motion_command[2] = 0.0;

            // Gait algorithm
            gait_algorithm(sim_time, idt*motion_plan_rate,
                           new_robot, motion_command);

            // Interface between gait algorithm and DynaMechs.  This
            // returns desiredJointPos with the next set of desired
            // joint positions.
            static CartesianVector refPos;
            static EulerAngles refPose;
            interface_Gait2DynaMechs(new_robot, refPose, refPos,
                                     desiredJointPos);

            motion_plan_count = 0;
         }

         computeAquaControl(desiredJointPos);

         G_integrator->simulate(idt);

         sim_time += idt;
         motion_plan_count++;
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
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
   int i, j;

   glutInit(&argc, argv);

   //=========================
   char *filename = "simulation.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640,480);
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
   //view_mat[3][2] = -10.0;
   camera = new dmGLPolarCamera_zup();
   camera->setRadius(8.0);
   camera->setCOI(8.0, 10.0, 2.0);
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
   G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));

   G_integrator = new dmIntegRK4();
   G_integrator->addSystem(G_robot);

   initAquaControl(G_robot);

   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutSpecialFunc(processSpecialKeys);
   glutDisplayFunc(display);
   glutIdleFunc(updateSim);

   dmGetSysTime(&last_tv);

   cout << endl;
   cout << "               p - toggles dynamic simulation" << endl;
   cout << "   UP/DOWN ARROW - increases/decreases velocity command" << endl;
   cout << "RIGHT/LEFT ARROW - changes heading command" << endl << endl;

   glutMainLoop();
   return 0;             /* ANSI C requires main to return int. */
}
