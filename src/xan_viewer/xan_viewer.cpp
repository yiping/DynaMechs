/*
 *****************************************************************************
 *     File: xan_viewer.cpp
 *   Author: Yiping Liu	
 *  Created: 2012
 *  Summary: A utility that allows you to preview 
 *           the object specified in .xan file
 *****************************************************************************/

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <dmTime.h>
#include <dmGLMouse.hpp>
#include <dmGLPolarCamera_zup.hpp>

#include <dm.h>              // DynaMechs typedefs, globals, ...
#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmEnvironment.hpp>
#include <dmIntegRK4.hpp>

#include <dmu.h>

#include <dmSecondaryPrismaticJoint.hpp>
#include <glLoadModels.h>

#include <regex.h>
#include <glob.h>

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;
bool IsWireframe = false;

dmSystem *G_robot;
dmIntegRK4 *G_integrator;

dmTimespec tv, last_tv;

int render_rate;
int timer_count = 0;


//----------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------

dmEnvironment *Establish_Dummy_Env_Model(void)
{
   dmEnvironment *env = new dmEnvironment;
   // set Environment Parameters (env, env_ptr);

   // set gravity
   CartesianVector gravity;
   gravity[0]=0; gravity[1]=0; gravity[2];
   env->setGravity(gravity);

   // set terrain model.
   //env->loadTerrainData("terrain.dat");
   // Read the elevation/depth data in meters.


   // set ground characteristics.
   env->setGroundPlanarSpringConstant(0);
   env->setGroundNormalSpringConstant(0);
   env->setGroundPlanarDamperConstant(0);
   env->setGroundNormalDamperConstant(0);
   env->setFrictionCoeffs(0.2, 0.1);
   //env->setFluidDensity(0);

   return env;
}

//----------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------

dmArticulation *Establish_Dummy_Dm_Model(char * xanname )
{
   dmArticulation *robot = new dmArticulation();
      
   robot->setName("object");
   GLuint *dlist = new GLuint;

   
   *dlist = glLoadModel(xanname);
   robot->setUserData((void *) dlist);

   CartesianVector pos;
   Quaternion quat;

   pos[0]=0; pos[1]=0;  pos[2]=0;
   quat[0]=0; quat[1]=0; quat[2]=0; quat[3]=1.0;
   robot->setRefSystem(quat, pos);

   return robot;
}






//----------------------------------------------------------------------------
//    Summary: Initialize material property and light source.
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void myinit (void)
{
   GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
   GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
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
//    Summary:
// Parameters:
//    Returns:
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
   glVertex3f(1.0, 0.0, 0.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glBegin(GL_LINES);
   glColor3f(0.0, 1.0, 0.0);
   glVertex3f(0.0, 1.0, 0.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glBegin(GL_LINES);
   glColor3f(0.0, 0.0, 1.0);
   glVertex3f(0.0, 0.0, 1.0);
   glVertex3f(0.0, 0.0, 0.0);
   glEnd();

   glEnable (GL_LIGHTING);

   glPopMatrix ();

   glPushMatrix();
   glColor3f(1.0, 0.0, 0.0);
   glTranslatef(1.01, 0, 0);
   glRotatef(90.0, 1, 0, 0);
   glScalef(0.0005, 0.0005, 0.0005);
   glLineWidth (2.0);
   char *p = "x";
   glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
   glPopMatrix();

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
      case 'w':
         IsWireframe = !IsWireframe;
         break;
   }
}


//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
void updateFunc()
{

   camera->update(mouse);
   camera->applyView();

   display();


}



//----------------------------------------------------------------------------
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
   glutInit(&argc, argv);

   //
   char *xanname = "";
   if (argc > 1)
   {
      xanname = argv[1];
      printf("The .xan file you are viewing is: %s \r\n", xanname);
   }
   else
   {
	printf("\r\nWARNING! - You need to specify which .xan file you want to preview\r\n\r\n");
	return 0;
   }

   ifstream xan_ptr(xanname);
   if (!xan_ptr)
   {
      cerr << "Unable to open the specified xan file: "
           << xanname << endl;
      exit(1);
   }
   xan_ptr.close();



   glutInitWindowSize(640, 480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("Xan_viewer");

   myinit();
   mouse = dmGLMouse::dmInitGLMouse();

   camera = new dmGLPolarCamera_zup();
   camera->setRadius(2.0);
   camera->setCOI(0.0, 0.0, 0.0);
   camera->setTranslationScale(0.02f);

   //simulation timing information.
   idt = 0.001; // global variable

   //
   render_rate = 20; //global variable

// ===========================================================================
// Initialize DUMMY DynaMechs environment - must occur before any linkage systems
   dmEnvironment *environment = Establish_Dummy_Env_Model();
   environment->drawInit();
   dmEnvironment::setEnvironment(environment);

// ===========================================================================
// Initialize a DUMMY DynaMechs linkage system
   G_robot = Establish_Dummy_Dm_Model(xanname); //global variable

// ===========================================================================
   G_integrator = new dmIntegRK4();
   G_integrator->addSystem(G_robot);


   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutDisplayFunc(display);
   glutIdleFunc(updateFunc);

   dmGetSysTime(&last_tv);

   cout << endl;
   
   cout << "Use mouse to rotate/move/zoom the camera" << endl << endl;

   glutMainLoop();
   return 0;             /* ANSI C requires main to return int. */
}
