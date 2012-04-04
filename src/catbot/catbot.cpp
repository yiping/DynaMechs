/*****************************************************************************
 * DynaMechs: A Multibody Dynamic Simulation Library
 *****************************************************************************
 *     File: kurmet.cpp
 *   Author: Yiping Liu
 *  Created: 20 Jan 2012
 *  Summary: 
 *****************************************************************************/

#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmu.h>
#include <dmGL.h>
#define PI 3.14159265358979323846264338327950288419
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
#include <dmIntegEuler.hpp>
#include <dmEnvironment.hpp>
#include <dmMDHLink.hpp>
#include <dmMobileBaseLink.hpp>

#include "functions.h"
#include "global_typedef.h"

//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO

// ----------------------------------------------------------------------

const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;

bool IsWireframe = false;
bool paused_flag = true;

dmArticulation *G_robot;
dmIntegEuler *G_integrator;

vector<LinkInfoStruct*> G_robot_linkinfo_list;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;


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
   glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode (GL_MODELVIEW);
   glPushMatrix ();

   // ===============================================================
   (dmEnvironment::getEnvironment())->draw();

   glPushAttrib(GL_ALL_ATTRIB_BITS);
   if (IsWireframe == false )
   {
      glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
   }
   else
   {   
      glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
   }
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



   //glEnable (GL_LIGHTING);

   glPopMatrix ();

 //// Write some information on the viewport
    // we are currently in MODEL_VIEW
	glPushMatrix ();
	glLoadIdentity ();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix ();
	glLoadIdentity();

	GLint viewport [4];
	glGetIntegerv (GL_VIEWPORT, viewport);
	gluOrtho2D (0,viewport[2], viewport[3], 0);
	// build a orthographic projection matrix using width and height of view port

	glDepthFunc (GL_ALWAYS);
	glColor3f (0,0,0);
	glRasterPos2f(10, 20);
	char * displaytext = "Catbot Model";
	int len = (int) strlen(displaytext);
	for (int i = 0; i<len; ++i)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, displaytext[i]);
	glColor3f (1,1,1);
	glRasterPos2f(10, 40);
	char buffer [50];
    sprintf (buffer, "%f", sim_time);
    len = (int) strlen(buffer);
	for (int i = 0; i<len; ++i)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);
	glDepthFunc (GL_LESS);
	glPopMatrix ();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix ();



    //  When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	glEnable (GL_LIGHTING);



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
 
         break;
      case GLUT_KEY_RIGHT:

         break;
      case GLUT_KEY_UP:

         break;
      case GLUT_KEY_DOWN:

         break;
   }
}


//----------------------------------------------------------------------------
void updateSim()
{
	
	
	
   if (!paused_flag)
   {
      for (int i=0; i<render_rate && sim_time <10.000; i++)
      {
		  dmArticulation * art = (dmArticulation*) G_robot;
		  VectorXF jointQdd;
		  jointQdd = VectorXF::Zero(6);
		  G_robot->m_link_list[0]->link_val2.qdd = jointQdd;
		  jointQdd = VectorXF::Zero(1);

		  G_robot->m_link_list[1]->link_val2.qdd = jointQdd;
		  G_robot->m_link_list[3]->link_val2.qdd = jointQdd;
		  G_robot->inverseDynamics(false);

		  //for (int k=0; k<G_robot->getNumLinks(); k++) {
			//  cout << "tau " << k << "(" << G_robot->m_link_list[k]->link->getName() << ") =" << " [ " << G_robot->m_link_list[k]->link_val2.tau << "]" << endl;
		  //}

		  // Compose CandG vector from ID results
		  VectorXF CandG;
		  CandG.resize(8);
		  CandG.head(6) = G_robot->m_link_list[0]->link_val2.tau;
		  CandG.segment(6,1) = G_robot->m_link_list[1]->link_val2.tau;
		  CandG.segment(7,1) = G_robot->m_link_list[3]->link_val2.tau;

		  art->computeH();
		  Matrix6F Hbb = art->H.block(0,0,6,6);
		  MatrixXF Hjj = art->H.block(6,6,2,2);
		  MatrixXF Hbj = art->H.block(0,6,6,2);
		  MatrixXF Hjb = art->H.block(6,0,2,6);
		  Matrix6F invHbb = Hbb.inverse();
		  MatrixXF SqBar = MatrixXF::Zero(8,2);

		  SqBar.block(0,0,6,2) = -invHbb * Hbj;
		  SqBar.block(6,0,2,2) = MatrixXF::Identity(2,2);

		  // A and candgA provide colocated PFL with tau = A qdd + candgA
		  MatrixXF A = Hjj - Hjb * invHbb * Hbj;
		  VectorXF candgA = SqBar.transpose() * CandG;
		  		  
		  Vector2F vDes,pDes,aDes,aCom,vAct,pAct;
		  
		  // Populate Actual position and velocity
		  Float q[1],qd[1], t = sim_time;
		  G_robot->m_link_list[1]->link->getState(q,qd);
		  pAct(0) = q[0]; vAct(0) = qd[0];
		  
		  
		  G_robot->m_link_list[3]->link->getState(q,qd);
		  pAct(1) = q[0]; vAct(1) = qd[0];

		  // Compute Desired Quantities
		  vDes << 4.*sin(2*PI*t) , 4.*cos(2.*PI*t);
		  aDes << 8.*PI*cos(2*PI*t) , -8.*PI*sin(2.*PI*t);
		  pDes << 4.*pow(sin(PI*t),2)/PI , 2.*sin(2*PI*t)/PI;
		  
		  // Compute Commanded Quantity
		  aCom = aDes + 1000*(vDes - vAct) + 5000*(pDes-pAct);
		  
		  // Comput PFL torque
		  Vector2F tau = A*aCom + candgA;
		  
		  // Assign torque as joint inputs
		  Float ftau[1]; ftau[0]=tau(0);
		  G_robot->m_link_list[1]->link->setJointInput(ftau);
		  
		  ftau[0] = tau(1);
		  G_robot->m_link_list[3]->link->setJointInput(ftau);
		  
		  //exit(-1);
		  
		  // Simulate dt
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

	   MatrixXF q, qd;
	   q.resize(1,9);
	   qd.resize(1,9);
	   G_robot->getState(q.data(),qd.data());
	   cout << "Q = " <<  q << endl << "Qd " << qd << endl;
	   
      timer_count = 0;
      last_tv.tv_sec = tv.tv_sec;
      last_tv.tv_nsec = tv.tv_nsec;
   }


   //cout<<"--- --- --- "<<endl;
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

   //
   char *filename = "catbot.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640,480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("catbot");

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
   camera = new dmGLPolarCamera_zup(); // dig into
   camera->setRadius(8.0);
   //camera->setCOI(8.0, 10.0, 2.0);
   camera->setCOI(3.0, 3.0, 0.0);
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

   // control_stepsize =%lf\t

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

   G_integrator = new dmIntegEuler();
   G_integrator->addSystem(G_robot);

	
   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutSpecialFunc(processSpecialKeys);
   glutDisplayFunc(display);
   glutIdleFunc(updateSim);

   dmGetSysTime(&last_tv);

   cout<<"kurmet has "<<G_robot->getNumLinks()<<" links."<<endl;
   cout<<"link 0 has "<<G_robot->getLink(0)->getNumDOFs()<<" DOFs."<<endl;
   cout<<"link 1 has "<<G_robot->getLink(1)->getNumDOFs()<<" DOFs."<<endl;
	cout<<"link 2 has "<<G_robot->getLink(2)->getNumDOFs()<<" DOFs."<<endl;
	cout<<"link 3 has "<<G_robot->getLink(3)->getNumDOFs()<<" DOFs."<<endl;
	cout << "               p - toggles dynamic simulation" << endl;

   glutMainLoop();


   return 0;             /* ANSI C requires main to return int. */
}