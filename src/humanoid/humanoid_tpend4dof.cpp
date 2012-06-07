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

//#include "mosek.h" /* Include the MOSEK definition file. */
#include "TaskSpaceController.h"

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
   //glLightfv (GL_LIGHT0, GL_POSITION, light_position);

//	glLightfv (GL_LIGHT1, GL_AMBIENT, light_ambient);
//	glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);
//	glLightfv (GL_LIGHT1, GL_SPECULAR, light_specular);
	//glLightfv (GL_LIGHT1, GL_POSITION, light_position);
	
	glLightfv (GL_LIGHT2, GL_AMBIENT, light_ambient);
	glLightfv (GL_LIGHT2, GL_DIFFUSE, light_diffuse);
	glLightfv (GL_LIGHT2, GL_SPECULAR, light_specular);
	//glLightfv (GL_LIGHT2, GL_POSITION, light_position);
	
//	glLightfv (GL_LIGHT3, GL_AMBIENT, light_ambient);
//	glLightfv (GL_LIGHT3, GL_DIFFUSE, light_diffuse);
//	glLightfv (GL_LIGHT3, GL_SPECULAR, light_specular);
	//glLightfv (GL_LIGHT3, GL_POSITION, light_position);
	
   glEnable (GL_LIGHTING);
   glEnable (GL_LIGHT0);
//	   glEnable (GL_LIGHT1);
	   glEnable (GL_LIGHT2);
//	   glEnable (GL_LIGHT3);
	
   glDepthFunc(GL_LESS);
   glEnable(GL_DEPTH_TEST);

   /*glShadeModel(GL_FLAT);
   glEnable(GL_CULL_FACE);
   glCullFace(GL_BACK);*/
	
	// ****
	//glShadeModel(GL_FLAT);
	glShadeModel(GL_SMOOTH);
	
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	
	// ****
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);//!!
	// The above two lines mean that glMaterial will control the polygon's specular and emission colors
	// and the ambient and diffuse will both be set using glColor. - yiping
	
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

	
	glDisable (GL_LIGHTING);// ****
	glDepthFunc (GL_ALWAYS);
	glColor3f (0,0,0);
	glRasterPos2f(10, 20);
	char * displaytext = "Humanoid Model";
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
	glEnable (GL_LIGHTING);// ****


    //  When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);



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
      for (int i=0; i<render_rate; i++)
      {
		G_integrator->simulate(idt);
    	sim_time += idt;
      }
   }
	
	VectorXF q, qd;
	q.resize(11);
	qd.resize(11);
	q << 0,0,0,1, 1,2,3, .2,-.4,.1,.3;
	qd << 0,1,2,  -3,-5,4,  0, 2.3,.9,.1,-.2;
	
	G_robot->setState(q.data(),qd.data());
	
	
	
	IntVector SupportIndicies(1);
	SupportIndicies[0] = 2;
	
	// Spatial Velocity Transform from body 2 to support frame
	XformVector Xforms(1);
	Xforms[0].resize(6,6);
	Xforms[0] << 0,0,-1,0,0,0, 
				 0,1, 0,0,0,0,
				 1,0, 0,0,0,0,
				 0,3, 0,0,0,-1,
			     0,0,3,0,1,0,
				 0,0,0,1,0,0;


	TaskSpaceController tsc(G_robot);
	tsc.SupportIndices =  SupportIndicies;
	tsc.SupportXforms = Xforms;
	
	tsc.ObtainArticulationData();
	
	MatrixXF Jac;
	// Calculate Task Jacobian
	G_robot->calculateJacobian(2, Xforms[0], Jac);
	
	//tsc.TaskJacobian.resize(3,8);
	tsc.TaskJacobian = Jac.block(3,0,3,10);
	
	// Compute Task Bias from RNEA Fw Kin
	tsc.TaskBias.resize(6);
	Vector6F SpatBias, ClassBias,SpatV;
	
	SpatBias = Xforms[0]*G_robot->m_link_list[2]->link_val2.a;
	SpatV = Xforms[0]*G_robot->m_link_list[2]->link_val2.v;
	
	ClassicAcceleration(SpatBias,SpatV,ClassBias);
	tsc.TaskBias = -ClassBias.segment(3,3);
	
	tsc.TaskBias(0) += 1;
	tsc.TaskBias(1) +=-2;
	tsc.TaskBias(2) +=6;
	
	
	
	
	
	tsc.InitializeProblem();
	tsc.Optimize();
	VectorXF tau = tsc.xx.segment(0,4);
	VectorXF qdd = tsc.xx.segment(2+2,8+2);
	VectorXF fs = tsc.xx.segment(10+4,6);
	VectorXF lambda = tsc.xx.segment(16+4,4);
	
	cout << "tau = " << tau.transpose() << endl;
	cout << "qdd = " << qdd.transpose() << endl;
	cout << "fs = "  << fs.transpose()  << endl;
	cout << "lambda = "  << lambda.transpose()  << endl;
	
	
	VectorXF a = tsc.TaskJacobian * qdd;
	cout << "a" << endl;
	
	VectorXF e = tsc.TaskJacobian * qdd - tsc.TaskBias;
	cout << "e = " << e.transpose() << endl;
	
	MatrixXF H = G_robot->H;
	VectorXF CandG = G_robot->CandG;
	
	MatrixXF S = MatrixXF::Zero(2+2,8+2);
	S.block(0,6,2+2,2+2) = MatrixXF::Identity(2+2,2+2);
	
	VectorXF qdd2 = H.inverse()*(S.transpose() * tau + tsc.SupportJacobians[0].transpose()*fs- CandG);
	cout << "qdd2 = " << qdd2.transpose() << endl;
	
	VectorXF qdd3 = H.inverse()*(S.transpose() * tau - CandG);
	cout << "qdd3 = " << qdd3.transpose() << endl;
	
	VectorXF state = VectorXF::Zero(18+4);
	state.segment(0,9+2) = q;
	state.segment(9+2,9+2) = qd;
	
	VectorXF stateDot = VectorXF::Zero(18+4);
	
	// Form Joint Input and simulate
	VectorXF jointInput = VectorXF::Zero(9+2);
	jointInput.segment(7,2+2) = tau;
	G_robot->setJointInput(jointInput.data());
	

	// Process qdds
	G_robot->dynamics(state.data(),stateDot.data());

	
	VectorXF qdds = VectorXF::Zero(8+2);
	qdds.segment(0,6) = stateDot.segment(9+2,6);
	
	//cout << "w x v " << cr3(qd.segment(0,3))*qd.segment(3,3) << endl;
	
	qdds.segment(3,3) -= cr3(qd.segment(0,3))*qd.segment(3,3);
	qdds.segment(6,2+2) = stateDot.segment(16+2,2+2);
	qdds(5)+=9.81;
	
	cout << "qdds = " << qdds.transpose()  << endl;
	
	
	exit(-1);
    camera->update(mouse);
    camera->applyView();

	// ****
	// if you want the GL light to move with the camera, comment the following two lines - yiping
	const GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	//const GLfloat light_position1[] = { -1.0, 1.0, 1.0, 0.0 };
	const GLfloat light_position2[] = { -1.0, -1.0, 1.0, 0.0 };
	//const GLfloat light_position3[] = { 1.0, -1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position0);
	//glLightfv (GL_LIGHT0, GL_POSITION, light_position1);
	glLightfv (GL_LIGHT0, GL_POSITION, light_position2);
	//glLightfv (GL_LIGHT0, GL_POSITION, light_position3);
	
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
   char *filename = "pendulum.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640,480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("Humanoid");

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
   cout << "               p - toggles dynamic simulation" << endl;

   glutMainLoop();


   return 0;             /* ANSI C requires main to return int. */
}