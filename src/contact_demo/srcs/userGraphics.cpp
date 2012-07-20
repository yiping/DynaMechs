
//  userGraphics.h
//  July 7, 2012
//  YL

#include "globalVariables.h"
#include "globalFunctions.h"
#include "BasicGLPane.h"
#include <stdio.h>
#include "dmRigidBody.hpp"
#include "dmDynamicContactModel.hpp"
#include "dmContactModel.hpp"

void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength);
const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);


void BasicGLPane::userGraphics2DPrimary()
{
	// for customized text rendering on primary viewport

	// vector<Float> c1u(G_contact->getDynamicContacts()[0]->u[0]);
	//cout<<"c1u - "<<c1u[0]<<endl;

	SpatialVector f;
	dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->getExternalForce(f);

	if (f[3] != 0)
	{
		RotationMatrix  R;
		CartesianVector p;	
		G_robot->getLink(0)->getPose(R,p);
		GLdouble fvp[3];
		gluProject(p[0]+f[3]/100+0.05, p[1], p[2], modelview, projection, vp, &fvp[0], &fvp[1], &fvp[2]);

	
		glColor3f (1,1,1);
		glRasterPos2f(fvp[0], vp[3]-fvp[1]);
		//glRasterPos2f(10, 60);//
		char disp_text[100]= "";
		sprintf(disp_text, "%2.2f", f[3]);
		int len = (int) strlen(disp_text);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, disp_text[i]);
	}
}



void BasicGLPane::userGraphics()
{
	// Plot User Stuff
	{
		// draw ground contact force
		Vector3F box_pos;
		Vector3F cforce;
		RotationMatrix  R;
		CartesianVector p;	
		G_robot->getLink(0)->getPose(R,p);
		box_pos= Map<Vector3F>(p);
		SpatialVector f;

		#ifdef USE_DYNAMIC_CONTACT_MODEL
		dynamic_cast<dmDynamicContactModel *>( dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->getForce(0))->getLastComputedValue(f);
		#else
		dynamic_cast<dmContactModel *>( dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->getForce(0))->getLastComputedValue(f);
		#endif

		cforce[0]=f[3]; cforce[1]=f[4]; cforce[2]=f[5]; 
		cforce = cforce/100;
		glColor3f(1.0, 0.0, 0.0);
		drawArrow(box_pos, cforce, .005, .01, .03);		
		glColor3f(1.0, 1.0, 1.0);

		dynamic_cast<dmRigidBody *>(G_robot->getLink(0))->getExternalForce(f);
		Vector3F ext_f;
		ext_f[0]=f[3]; ext_f[1]=f[4]; ext_f[2]=f[5]; ext_f=ext_f/100;
		glColor3f(0.0, 0.0, 1.0);
		drawArrow(box_pos, ext_f, .005, .01, .03);		
		glColor3f(1.0, 1.0, 1.0);

		if (frame->showCoM->IsChecked()) 
		{

		}
		
		
		if (frame->showGRF->IsChecked()) 
		{

		}
		
		if (frame->showNetForceAtGround->IsChecked()) 
		{

		}
		
		if (simThread->sim_time>500) {
			glPushMatrix();
			glColor4f(1.0, 0.0, 0.0,0.75);
			glTranslatef(2.18, 2, .15);
			glRotated(-90, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			glRotated(180, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			
			glPopMatrix();
		}
		frame->realTimeRatioDisplay->SetLabel(wxString::Format(wxT("RT Ratio: %.2lf"), real_time_ratio));
		
	}
	
}



void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength) {
	Vector3F zup; zup << 0,0,1;
	
	Vector3F normedDirection = direction.normalized();
	
	// Note: We need to create a rotation such that the z-axis points in the direction of the 'direction' argument
	//       Thus, we can realize the rotation with a pure x-y axial rotation
	//       The rotation matrix of the rotated frame 'r' to the current frame 'c' (c_R_r) has special form.
	//       It's 3-rd column in particular has form: [ wy*sin(theta) -wx*sin(theta) (1- cos(theta))]'
	//       where theta , [wx wy 0] is the angle, axis of rotation
	
	// Find the rotation angle by comparing the z-axis of the current and rotated frame
	const double cosTheta = zup.dot(normedDirection);
	const double theta = acos(cosTheta);
	const double sinTheta = sin(theta);
	
	
	// Exploit the special form of the rotation matrix (explained above) for find the axis of rotation
	double rX, rY, rZ;
	if(theta > 0) {	
		rX = - normedDirection(1)/sinTheta;
		rY =   normedDirection(0)/sinTheta;
		rZ = 0;
	}
	else {
		rX = 0;
		rY = 0;
		rZ = 1;
	}
	
	// before calling this function, be sure glMatrixMode is GL_MODELVIEW and camera->applyView is called.
	glPushMatrix();
	glTranslatef(location(0), location(1), location(2));
	glRotated(RADTODEG * theta, rX, rY, rZ);
	
	
	double cylinderLength = direction.norm();
	if (cylinderLength > headLength) {
		cylinderLength -= headLength;
	}
	else {
		headLength = cylinderLength ;
		cylinderLength = 0;
	}

	const int detail = 16;
	// Draw Cylinder [along the z axis]
    // gluCylinder(GLUquadricObj *qobj, GLdouble baseRadius, GLdouble topRadius, GLdouble height, GLint slices, GLint stacks)
	//gluCylinder(frame->glPane->quadratic,lineWidth,lineWidth,cylinderLength,detail,detail);
		glBegin(GL_LINES);
		glVertex3f(0.0, 0.0,cylinderLength );
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();	

	//Draw Cylinder Base
	//gluDisk(frame->glPane->quadratic,0,lineWidth,detail,detail);
	
	glTranslatef(0, 0, cylinderLength);
	//Draw Arrowhead
	gluCylinder(frame->glPane->quadratic,headWidth,0.0f,headLength,detail,detail);
	
	//Draw Arrowhead Base
	gluDisk(frame->glPane->quadratic,lineWidth,headWidth,detail,detail);
	
	glEnd();
	glPopMatrix();
	
}

