/*
 *****************************************************************************
 *     File: functions_A.cpp
 *   Author: Yiping Liu
 *  Created: 17 Apr 2012
 *  Summary: OpenGL rendering functions / UI functions  
 *            
 *****************************************************************************/
#include "global.h"   

//----------------------------------------------------------------------------
void myinit (void)
{
	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.7, 0.7, 0.7, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	//     light_position is NOT default value
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

	glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
	//glLightfv (GL_LIGHT0, GL_POSITION, light_position);

	glLightfv (GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv (GL_LIGHT1, GL_SPECULAR, light_specular);


	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
	glEnable (GL_LIGHT1);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);

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
	//glClearColor (1.0, 1.0, 1.0,1.0);
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

	//glDisable (GL_LIGHTING);

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

	// // output a speed/direction command vector
	/*   
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
	glPopMatrix();*/




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
	char * displaytext = "Flywheel Biped Test";
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


	glFlush ();//!
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
 
         break;
      case GLUT_KEY_RIGHT:

         break;
      case GLUT_KEY_UP:

         break;
      case GLUT_KEY_DOWN:

         break;
   }
}
