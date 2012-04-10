/*
 *****************************************************************************
 *     File: windmill.cpp
 *   Author: Yiping Liu
 *  Created: 21 Mar 2012
 *  Summary: Try to regain balance with windmilling arm
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
#include <dmContactModel.hpp>

#include "functions.h"
#include "global_typedef.h"

//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO
//#define JOINT_POSITION_PD_ONLY

// ----------------------------------------------------------------------

const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;

DataRecVector MyVec;
Vector6F err = Vector6F::Zero();
Vector6F desired_torso_acc = Vector6F::Zero();
Float tr[9][1];
Vector3F p_ZMP_ICS = Vector3F::Zero();
Vector3F actual_p_ZMP_ICS = Vector3F::Zero();
Vector3F p_RF_ICS = Vector3F::Zero();
Vector3F p_LF_ICS = Vector3F::Zero();
Float q1ss,q2ss,q0ss;
Float qd1ss, qd2ss, qd0ss;
bool outputOnce = true;
bool pc_ini = true;

bool IsWireframe = false;

bool paused_flag = true;
dmArticulation *G_robot;
//dmIntegRK4 *G_integrator;
dmIntegEuler *G_integrator;

vector<LinkInfoStruct*> G_robot_linkinfo_list;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;

int control_count = 0;

void SaveToDataRecord(DataRecord *Rec);


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
   glLightfv (GL_LIGHT0, GL_POSITION, light_position);

   glEnable (GL_LIGHTING);
   glEnable (GL_LIGHT0);
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
/*   glPushMatrix();
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

	glDepthFunc (GL_ALWAYS);
	glColor3f (0,0,0);
	glRasterPos2f(10, 20);
	char * displaytext = "Windmill Test";
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
	//glEnable (GL_LIGHTING);



   glFlush ();//!
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

//




//----------------------------------------------------------------------------
void updateSim()
{
	if (!paused_flag)
	{
		for (int i=0; i<render_rate; i++)
		{

			///
			G_integrator->simulate(idt);
			sim_time += idt;

			control_count ++ ;
			if (control_count == 10)
			{
				#ifndef JOINT_POSITION_PD_ONLY
				if (sim_time > 5.0)
				{

					if (pc_ini == true) // set steady state torso state - will be used as reference
					{
						Float Q[1], Qd[1];
						G_robot->getLink(2)->getState(Q,Qd); q2ss = Q[0]; qd2ss = Qd[0];
						G_robot->getLink(1)->getState(Q,Qd); q1ss = Q[0]; qd1ss = Qd[0];
						G_robot->getLink(0)->getState(Q,Qd); q0ss = Q[0]; qd0ss = Qd[0];
						pc_ini = false;
					}

					//begin postural control

					/// 1. Resolved Acceleration Control
					Matrix3F Rot_rf_ICS; // right foot
					Matrix3F Rot_lf_ICS; // left foot
					Matrix3F Rot_rk_ICS; // right knee
					Matrix3F Rot_lk_ICS; // left knee
					Matrix3F Rot_t_ICS;  // torso

					Vector3F p_rk_ICS;
					Vector3F p_lk_ICS;
					Vector3F p_rf_ICS;
					Vector3F p_lf_ICS;
					Vector3F p_t_ICS;

					Vector3F prf;	prf<< 0.25,0, 0;
					Vector3F plf;	plf<< 0.25,0, 0;

					G_robot->computeSpatialVelAndICSPose(6);
					G_robot->computeSpatialVelAndICSPose(8);

					for (int m = 0; m< 3; m++)
					{
						p_rk_ICS(m) = G_robot_linkinfo_list[6]->link_val2.p_ICS[m];
						p_lk_ICS(m) = G_robot_linkinfo_list[8]->link_val2.p_ICS[m];
						p_t_ICS(m) = G_robot_linkinfo_list[2]->link_val2.p_ICS[m];
						for (int n = 0; n< 3; n++)
						{
							Rot_rk_ICS(m,n) = G_robot_linkinfo_list[6]->link_val2.R_ICS[m][n];
							Rot_lk_ICS(m,n) = G_robot_linkinfo_list[8]->link_val2.R_ICS[m][n];
							Rot_t_ICS(m,n)  = G_robot_linkinfo_list[2]->link_val2.R_ICS[m][n];
						}
					}

					// knee and foot have the same orientation
					Rot_rf_ICS = Rot_rk_ICS;
					Rot_lf_ICS = Rot_lk_ICS;

					p_rf_ICS = Rot_rk_ICS * prf + p_rk_ICS;
					p_lf_ICS = Rot_lk_ICS * plf + p_lk_ICS;



					//

					SpatialVector f_torso_ext; // in torso coordinate
					if ( (sim_time > 5.50) && (sim_time < 5.56 ) )
					{
						// apply a distrubance
						Vector3F disturbance_f_ICS;
						Vector3F disturbance_n_ICS;
						disturbance_f_ICS << 80, 0, 0;
						disturbance_n_ICS <<  0, 0, 0;
						Vector3F disturbance_f_t;
						Vector3F disturbance_n_t;
						disturbance_f_t = Rot_t_ICS.inverse() * disturbance_f_ICS;
						disturbance_n_t = Rot_t_ICS.inverse() * disturbance_n_ICS;

						for (int k = 0; k<3; k++)
						{
							f_torso_ext[k] = disturbance_n_t[k];
							f_torso_ext[k+3] = disturbance_f_t[k];
						}

						dynamic_cast<dmRigidBody*>(G_robot->getLink(2))->setExternalForce(f_torso_ext);
					}
					else
					{
						for (int k = 0; k<6; k++)
						{
							f_torso_ext[k] = 0;
						}
						dynamic_cast<dmRigidBody*>(G_robot->getLink(2))->setExternalForce(f_torso_ext);
					}




					Vector6F TorsoVel_curr = G_robot_linkinfo_list[2]->link_val2.v;
					Vector6F RkneeVel_curr = G_robot_linkinfo_list[6]->link_val2.v;
					Vector6F LkneeVel_curr = G_robot_linkinfo_list[8]->link_val2.v;



					// Desired torso position and orientation (ICS)
					Vector3F ref_torso_p_ICS;
					ref_torso_p_ICS<< 3.0, 5.08, 0.46;
					Vector3F ref_torso_R_ICS_c1, ref_torso_R_ICS_c2, ref_torso_R_ICS_c3;
					ref_torso_R_ICS_c1 << 0, 0, 1;
					ref_torso_R_ICS_c2 << 1, 0, 0;
					ref_torso_R_ICS_c3 << 0, 1, 0;

					Vector6F poseError;

					poseError(3) = ref_torso_p_ICS[0] - G_robot_linkinfo_list[2]->link_val2.p_ICS[0];
					poseError(4) = ref_torso_p_ICS[1] - G_robot_linkinfo_list[2]->link_val2.p_ICS[1];
					poseError(5) = ref_torso_p_ICS[2] - G_robot_linkinfo_list[2]->link_val2.p_ICS[2];

					Vector3F c1,c2,c3; // columns of ref_torso_R_ICS

					for (int g = 0; g<3;g++)
					{
						c1(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][0];
						c2(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][1];
						c3(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][2];

					}

					Matrix6F XI2 = G_robot->computeSpatialTransformation(2);

					poseError.head(3) = XI2.block(0,0,3,3) * (0.5*(cr3(c1)* ref_torso_R_ICS_c1 + cr3(c2)* ref_torso_R_ICS_c2 + cr3(c3)* ref_torso_R_ICS_c3)); // there is a bias... lwp
					poseError.tail(3) = XI2.block(0,0,3,3)* poseError.tail(3); //in torso coordinate

					// PD gain
					Vector6F desired_torso_acc = Vector6F::Zero();

					Float kp, kd;
					kd = 15;
					kp = 100;  // if I use kd = 20, kp = 200, everything works, but the ZMPx's behavior is kinda weird... how to explain?
					desired_torso_acc(2) = 0.02*kp * poseError(2) + 0.02*kd * ( - TorsoVel_curr(2));
					desired_torso_acc(3) = 0.8*kp * poseError(3) +  0.8*kd * ( - TorsoVel_curr(3));
					desired_torso_acc(4) = 0.5*kp * poseError(4) + 0.5*kd * ( - TorsoVel_curr(4));

					#ifdef KURMET_DEBUG
					cout<<"desired spatial torso acceleration is: "<<endl<<desired_torso_acc<<endl<<endl;
					#endif

					///2. express the spatial torso acceleration in joint space
					G_robot_linkinfo_list[2]->link_val2.qdd(0) = desired_torso_acc(2);
					Float torsoTheta[1],torsoOmega[1];
					G_robot->getLink(2)->getState(torsoTheta, torsoOmega);
					G_robot_linkinfo_list[1]->link_val2.qdd(0) =
						(desired_torso_acc(3)*cos( torsoTheta[0])  + desired_torso_acc(4)*sin(torsoTheta[0]))/2.08;
					G_robot_linkinfo_list[0]->link_val2.qdd(0) =
						-(desired_torso_acc(3)*sin(torsoTheta[0]) + desired_torso_acc(4)*cos(torsoTheta[0]))/2.08 ;






					/// 3.calculate the desired qdd for the leg joints
					//
					// right leg:
					Matrix6F X_rf;
					MatrixXF S(2,6);
					S<< 0,0,0,1,0,0,
					0,0,0,0,1,0;

					X_rf.block<3,3>(0,0) = Matrix3F::Identity();
					X_rf.block<3,3>(0,3) = Matrix3F::Zero();
					X_rf.block<3,3>(3,0) = -cr3(prf);
					X_rf.block<3,3>(3,3) = Matrix3F::Identity();

					Vector6F RfVel = X_rf*RkneeVel_curr;

					Vector6F accBiasR;
					Matrix6XF JR;
					Matrix6F X_TR; // spatial transformation from torso to right foot
					accBiasR = G_robot->computeAccelerationBias(6,X_rf,3); //double check...
					JR = G_robot->calculateJacobian(6,X_rf,3); //double check...
					X_TR = X_rf*G_robot->computeSpatialTransformation(6,3); //double check...
					Vector2F qdd_R;
					Vector3F omega_rf;
					omega_rf = RfVel.head(3);
					Vector6F b_term_R = Vector6F::Zero();
					b_term_R.tail(3) = cr3(omega_rf)*RfVel.tail(3);
					Vector2F rhs1 = S*(- X_TR * desired_torso_acc - accBiasR - b_term_R );

					//qdd_R = solveJacobianPseudoInverse(S*JR, rhs1);
					qdd_R = (S*JR).inverse()*rhs1;

					#ifdef KURMET_DEBUG
					cout<<"JR is: "<<endl<<JR<<endl<<endl;
					cout<<"rhs1 is: "<<endl<<rhs1<<endl<<endl;
					cout<<"qdd_R: "<<endl<<qdd_R<<endl<<endl;
					#endif

					G_robot_linkinfo_list[5]->link_val2.qdd(0) = qdd_R(0);
					G_robot_linkinfo_list[6]->link_val2.qdd(0) = qdd_R(1);

					// left leg:
					Matrix6F X_lf;

					X_lf.block<3,3>(0,0) = Matrix3F::Identity();
					X_lf.block<3,3>(0,3) = Matrix3F::Zero();
					X_lf.block<3,3>(3,0) = -cr3(plf);
					X_lf.block<3,3>(3,3) = Matrix3F::Identity();

					Vector6F LfVel = X_lf * LkneeVel_curr;

					Vector6F accBiasL;
					Matrix6XF JL;
					Matrix6F X_TL; // spatial transformation from torso to left foot
					accBiasL = G_robot->computeAccelerationBias(8,X_lf,3);
					JL = G_robot->calculateJacobian(8,X_lf,3);
					X_TL = X_lf*G_robot->computeSpatialTransformation(8,3);
					Vector2F qdd_L;
					Vector3F omega_lf;
					omega_lf = LfVel.head(3);
					Vector6F b_term_L = Vector6F::Zero();
					b_term_L.tail(3) = cr3(omega_lf)*LfVel.tail(3);
					Vector2F rhs2 = S*(- X_TL * desired_torso_acc - accBiasL  - b_term_L );

					//qdd_L = solveJacobianPseudoInverse(JL, rhs2);
					qdd_L = (S*JL).inverse() * rhs2;

					#ifdef KURMET_DEBUG
					cout<<"JL is: "<<endl<<JL<<endl<<endl;
					cout<<"rhs2 is: "<<endl<<rhs2<<endl<<endl;
					cout<<"qdd_L: "<<endl<<qdd_L<<endl<<endl;
					#endif

					G_robot_linkinfo_list[7]->link_val2.qdd(0) = qdd_L(0);
					G_robot_linkinfo_list[8]->link_val2.qdd(0) = qdd_L(1);



					#ifdef KURMET_DEBUG
					for (int g = 0; g<9; g++)
					{
						if ((g != 3) && (g!= 4))
							cout<<"qdd["<<g<<"]= "<<endl<<G_robot_linkinfo_list[g]->link_val2.qdd(0)<<endl;
					}
					cout<<endl;
					#endif

					/// 4.free space inverse dynamics, to figure out the spatial force on torso
					//
					G_robot->inverseDynamics(false); //no external forces




					/// 5. calculate feet and ZMP locations (in ICS)

					//Matrix6F XI2 = G_robot->computeSpatialTransformation(2);

					G_robot->computeSpatialVelAndICSPose(6);
					G_robot->computeSpatialVelAndICSPose(8);
					CartesianVector p_RK_ICS;
					CartesianVector p_LK_ICS;
					RotationMatrix R_RK_ICS;
					RotationMatrix R_LK_ICS;
					for (int i = 0;  i<3; i++)
					{
						p_RK_ICS[i] = G_robot_linkinfo_list[6]->link_val2.p_ICS[i]; //right knee position ICS
						p_LK_ICS[i] = G_robot_linkinfo_list[8]->link_val2.p_ICS[i]; //left knee position ICS
						for (int j = 0; j <3; j++)
						{
							R_RK_ICS[i][j] = G_robot_linkinfo_list[6]->link_val2.R_ICS[i][j]; //right knee Rotation ICS
							R_LK_ICS[i][j] = G_robot_linkinfo_list[8]->link_val2.R_ICS[i][j]; //left knee Rotation ICS
						}
					}
					// foot location in ICS
					// Vector3F p_RF_ICS;
					// Vector3F p_LF_ICS;Rot_ZMP_rf
					for (int i = 0; i < 3; i++)
					{
						p_RF_ICS(i) = p_RK_ICS[i] + 0.25*R_RK_ICS[i][0];
						p_LF_ICS(i) = p_LK_ICS[i] + 0.25*R_LK_ICS[i][0];
					}

					Float sink_depth;
					sink_depth = 0.5* (p_RF_ICS(2) + p_LF_ICS(2));

					Vector6F netForce_ICS;
					Matrix6F X2IF = XI2.transpose();
					Vector6F nf = G_robot_linkinfo_list[2]->link_val2.f;
					//nf(0) = 0; nf(1) = 0; nf(5) = 0;
					// comment: don't zero things out here, otherwise the ZMP location will not come out right.
					netForce_ICS = X2IF * (nf);





					// Computed ZMP location in ICS
					// Vector3F p_ZMP_ICS;
					p_ZMP_ICS(0) = - netForce_ICS(1)/netForce_ICS(5);
					p_ZMP_ICS(1) = netForce_ICS(0)/netForce_ICS(5);
					p_ZMP_ICS(2) = 0;


					// Construct X_Zrf (X transformation from ZMP to right foot)
					Vector3F p_rf_ZMP;
					p_rf_ZMP = p_rf_ICS - p_ZMP_ICS;
					Matrix3F Rot_ZMP_rf;
					Rot_ZMP_rf = Rot_rf_ICS.inverse();
					Matrix6F X_Zrf; X_Zrf << Rot_ZMP_rf, Matrix3F::Zero(),
										-Rot_ZMP_rf*cr3(p_rf_ZMP), Rot_ZMP_rf;
					Matrix6F X_rfZF;
					X_rfZF = X_Zrf.transpose();

					// Construct X_Zlf (X transformation from ZMP to left foot)
					Vector3F p_lf_ZMP;
					p_lf_ZMP = p_lf_ICS - p_ZMP_ICS;
					Matrix3F Rot_ZMP_lf;
					Rot_ZMP_lf = Rot_lf_ICS.inverse();
					Matrix6F X_Zlf; X_Zlf << Rot_ZMP_lf, Matrix3F::Zero(),
										-Rot_ZMP_lf*cr3(p_lf_ZMP), Rot_ZMP_lf;
					Matrix6F X_lfZF;
					X_lfZF = X_Zlf.transpose();



					#ifdef KURMET_DEBUG
					//cout<<"Torso force w.r.t Torso coordinate"<<endl<<G_robot_linkinfo_list[2]->link_val2.f<<endl<<endl;
					//cout<<"Net force ICS"<<endl<<netForce_ICS<<endl<<endl;
					cout<<"Right foot position ICS is: "<<endl<<p_RF_ICS<<endl<<endl;
					cout<<"left foot  position ICS is: "<<endl<<p_LF_ICS<<endl<<endl;
					cout<<"ZMP        position ICS is: "<<endl<<p_ZMP_ICS<<endl<<endl;
					Float theta1[1],thetad1[1];
					G_robot->getLink(1)->getState(theta1,thetad1);
					cout<<"Boom pitch (q1) is: "<<endl<<theta1[0]<<endl<<endl;
					#endif

					Vector6F netForce_ZMP;
					Matrix6F XIZF;
					XIZF << Matrix3F::Identity(), -cr3(p_ZMP_ICS),
						Matrix3F::Zero(), Matrix3F::Identity();
					netForce_ZMP = XIZF * netForce_ICS;

					#ifdef KURMET_DEBUG
					cout<<"netForce_ICS = ["<< netForce_ICS.transpose()<<"]"<<endl<<endl;
					cout<<"netForce_ZMP = ["<< netForce_ZMP.transpose()<<"]"<<endl<<endl;
					#endif


					Float a = abs(p_RF_ICS[0] - p_ZMP_ICS(0) );
					Float b = abs(p_LF_ICS[0] - p_ZMP_ICS(0) );

					Float c = abs(p_RF_ICS[1] - p_ZMP_ICS(1) );
					Float d = abs(p_LF_ICS[1] - p_ZMP_ICS(1) );

					// ZMP coordinate has the same orientation as ICS,
					// Right foot coordinate has the same orientation as right knee coordinate

					// There is also the normal moment at the ZMP...
					// equal magnitude, opposite sign.





					// If the computed ZMP is within the support (both feet on the ground)
					if (( max(p_RF_ICS[0], p_LF_ICS[0]) >p_ZMP_ICS(0) ) && ( p_ZMP_ICS(0)>min(p_RF_ICS[0], p_LF_ICS[0]) ) && (p_RF_ICS[2]<-0.00) && (p_LF_ICS[2]<-0.00))
					{

						//// Force distribution
						Vector6F GRF_rf = Vector6F::Zero();
						Vector6F GRF_lf = Vector6F::Zero();

						Vector3F tempF;
						tempF = Rot_t_ICS.inverse() * (netForce_ZMP.tail(3) );
						//tempF(2) = 0;
						GRF_rf.tail(3) = (Rot_rf_ICS.inverse()*Rot_t_ICS) * ((b/(a+b))*tempF);
						GRF_lf.tail(3) = (Rot_lf_ICS.inverse()*Rot_t_ICS) * ((a/(a+b))*tempF);

						#ifdef KURMET_DEBUG
						Vector6F testVec;
						testVec = X_rfZF * GRF_rf + X_lfZF * GRF_lf;

						cout<<"X_rfZF * GRF_rf + X_lfZF * GRF_lf = "<<testVec.transpose()<<endl<<endl;
						cout<<"X_rfZF * GRF_rf = "<<(X_rfZF * GRF_rf).transpose()<<endl<<endl;
						cout<<"X_lfZF * GRF_lf = "<<(X_lfZF * GRF_lf).transpose()<<endl<<endl;
						#endif


						//				//// Force distribution: an alternative way
						//				Matrix6F X26 = G_robot->computeSpatialTransformation(6,3);
						//				Matrix6F X28 = G_robot->computeSpatialTransformation(8,3);
						//
						//				Vector6F netForce_Rk;
						//				Matrix6F X62F = X26.transpose();
						//				netForce_Rk = solveInverse(X62F, (b/(a+b))*nf);
						//
						//				// -rf
						//				Matrix6F X2rf = X_rf * X26; Matrix6F Xrf2F = X2rf.transpose();
						//				Vector6F netForce_rf;
						//				netForce_rf = solveInverse(Xrf2F, (b/(a+b))*nf);
						//
						//				Vector6F netForce_Lk;
						//				Matrix6F X82F = X28.transpose();
						//				netForce_Lk = solveInverse(X82F, (a/(a+b))*nf);
						//
						//				// -lf
						//				Matrix6F X2lf = X_lf * X26; Matrix6F Xlf2F = X2lf.transpose();
						//				Vector6F netForce_lf;
						//				netForce_lf = solveInverse(Xlf2F, (a/(a+b))*nf);




						//// Inverse dynamics again with the distributed force on each foot

						// travel back to torso, adding in the additional force to each joint.

						//G_robot_linkinfo_list[6]->link_val2.f -= netForce_Rk;
						Matrix6F X_rf6F = X_rf.transpose();
						G_robot_linkinfo_list[6]->link_val2.f -= X_rf6F * GRF_rf;
						G_robot_linkinfo_list[6]->link_val2.tau(0) = G_robot_linkinfo_list[6]->link_val2.f(2);
						Matrix6F X56 = G_robot->computeSpatialTransformation(6,6);
						Matrix6F X65F = X56.transpose();
						//G_robot_linkinfo_list[5]->link_val2.f -= X65F * netForce_Rk;
						G_robot_linkinfo_list[5]->link_val2.f -= X65F * X_rf6F * GRF_rf;
						G_robot_linkinfo_list[5]->link_val2.tau(0) = G_robot_linkinfo_list[5]->link_val2.f(2);

						Matrix6F X25 = G_robot->computeSpatialTransformation(5,3);
						Matrix6F X52F = X25.transpose();

						//G_robot_linkinfo_list[8]->link_val2.f -= netForce_Lk;
						Matrix6F X_lf8F = X_lf.transpose();
						G_robot_linkinfo_list[8]->link_val2.f -= X_lf8F * GRF_lf;
						G_robot_linkinfo_list[8]->link_val2.tau(0) = G_robot_linkinfo_list[8]->link_val2.f(2);
						Matrix6F X78 = G_robot->computeSpatialTransformation(8,8);
						Matrix6F X87F = X78.transpose();
						//G_robot_linkinfo_list[7]->link_val2.f -= X87F * netForce_Lk;
						G_robot_linkinfo_list[7]->link_val2.f -= X87F * X_lf8F * GRF_lf;
						G_robot_linkinfo_list[7]->link_val2.tau(0) = G_robot_linkinfo_list[7]->link_val2.f(2);

						Matrix6F X27 = G_robot->computeSpatialTransformation(7,3);
						Matrix6F X72F = X27.transpose();

						#ifdef KURMET_DEBUG
							cout<<"X28 "<<endl<<X78*X27<<endl<<endl;
							cout<<"G_robot->computeSpatialTransformation(8,3): "<<endl<<G_robot->computeSpatialTransformation(8,3)<<endl<<endl;
						#endif

						#ifdef KURMET_DEBUG
						cout<<"b/(a+b) = "<<(b/(a+b))<<"|| a/(a+b) = "<<(a/(a+b))<<endl<<endl;
						cout<<"a b = ["<<a<<" "<<b<<"]"<<endl<<endl;
						#endif

						#ifdef KURMET_DEBUG
						cout<<"GRF_lf = ["<< GRF_lf.transpose()<<"]"<<endl<<endl;
						cout<<"GRF_rf = ["<< GRF_rf.transpose()<<"]"<<endl<<endl;

						cout<<"nf = ["<<nf.transpose()<<"]"<<endl<<endl;

						cout<<"calculated free space torso force: "<<endl<< (G_robot_linkinfo_list[2]->link_val2.f).transpose()<<endl<<endl;
						Vector6F normal_moment_ZMP;
						normal_moment_ZMP << 0,0,netForce_ZMP(2),0,0,0;
						Vector6F TorsoForce_canceled = G_robot_linkinfo_list[2]->link_val2.f
													- X52F* X65F * X_rf6F * GRF_rf
													- X72F* X87F * X_lf8F * GRF_lf;
						cout<<"check torso force after 2nd round ID: "<<endl<< TorsoForce_canceled.transpose()<<endl<<endl;
						Vector6F TorsoForce_canceled2 = G_robot_linkinfo_list[2]->link_val2.f
													  - solveInverse(XIZF*X2IF, netForce_ZMP);
						cout<<"check torso force after 2nd round ID Another: "<<endl<< TorsoForce_canceled2.transpose()<<endl<<endl;
						Vector6F TorsoForce_canceled3 = G_robot_linkinfo_list[2]->link_val2.f
																  - solveInverse(XIZF*X2IF, testVec);
						cout<<"check torso force after 2nd round ID Third check: TestVec: "<<endl<< TorsoForce_canceled3.transpose()<<endl<<endl;
						#endif

						//// Set joint torques
						Float JointTorque[1];
						JointTorque[0] = G_robot_linkinfo_list[6]->link_val2.tau(0);
						G_robot->getLink(6)->setJointInput(JointTorque);
						tr[6][0] = JointTorque[0];

						JointTorque[0] = G_robot_linkinfo_list[5]->link_val2.tau(0);
						G_robot->getLink(5)->setJointInput(JointTorque);
						tr[5][0] = JointTorque[0];

						JointTorque[0] = G_robot_linkinfo_list[8]->link_val2.tau(0);
						G_robot->getLink(8)->setJointInput(JointTorque);
						tr[8][0] = JointTorque[0];

						JointTorque[0] = G_robot_linkinfo_list[7]->link_val2.tau(0);
						G_robot->getLink(7)->setJointInput(JointTorque);
						tr[7][0] = JointTorque[0];

						//make sure there are no torques on joint 0,1,2
						JointTorque[0] = 0;
						G_robot->getLink(0)->setJointInput(JointTorque);
						tr[0][0] = JointTorque[0];
						G_robot->getLink(1)->setJointInput(JointTorque);
						tr[1][0] = JointTorque[0];
						G_robot->getLink(2)->setJointInput(JointTorque);
						tr[2][0] = JointTorque[0];


					}
					// if the computed ZMP is located outside the support
					else //if (p_ZMP_ICS(0) >= max(p_RF_ICS[0], p_LF_ICS[0]) )
					{
						cerr << " ZMP is outside support polygon!  " << endl;

						// Lock the joints of the left leg


					   //simDataOutput(MyVec);
					   //exit(1);
					}








					//			if ( (sim_time > 5.4) && (sim_time < 5.405)  )
					//			{
					//				Float Q[1], Qd[1];
					//				G_robot->getLink(5)->getState(Q,Qd);
					//				cout<<"q5 = ["<< Q[0]<<"]"<<endl;
					//				G_robot->getLink(6)->getState(Q,Qd);
					//				cout<<"q6 = ["<< Q[0]<<"]"<<endl;
					//				G_robot->getLink(7)->getState(Q,Qd);
					//				cout<<"q7 = ["<< Q[0]<<"]"<<endl;
					//				G_robot->getLink(8)->getState(Q,Qd);
					//				cout<<"q8 = ["<< Q[0]<<"]"<<endl<<endl;
					//			}






					//			/// Calculate the actual ZMP location
					//
					//			// - get the actual ground contact forces
					//			SpatialVector rf_actual_contact_force;
					//			SpatialVector lf_actual_contact_force;
					//			dynamic_cast <dmRigidBody*>(G_robot->getLink(6))->getForce(0)->computeForce(G_robot_linkinfo_list[6]->link_val2, rf_actual_contact_force);
					//			dynamic_cast <dmRigidBody*>(G_robot->getLink(8))->getForce(0)->computeForce(G_robot_linkinfo_list[8]->link_val2, lf_actual_contact_force);
					//
					//			// - get the actual contact states
					//			bool rf_in_contact, lf_in_contact;
					//			rf_in_contact = dynamic_cast <dmContactModel*>(dynamic_cast <dmRigidBody*>(G_robot->getLink(6))->getForce(0))->getContactState(0);
					//			lf_in_contact = dynamic_cast <dmContactModel*>(dynamic_cast <dmRigidBody*>(G_robot->getLink(8))->getForce(0))->getContactState(0);
					//
					//			Matrix6F XI6 = G_robot->computeSpatialTransformation(6);
					//			Matrix6F XI8 = G_robot->computeSpatialTransformation(8);
					//
					//			Matrix6F X6IF = XI6.transpose();
					//			Matrix6F X8IF = XI8.transpose();
					//
					//			Vector6F actual_rf_f;
					//			Vector6F actual_lf_f;
					//
					//			for (int i =0; i<6; i++)
					//			{
					//				actual_rf_f(i) = rf_actual_contact_force[i];
					//				actual_lf_f(i) = lf_actual_contact_force[i];
					//			}
					//
					//			Vector6F actual_netForce_ICS;
					//			actual_netForce_ICS = X6IF * actual_rf_f + X8IF * actual_lf_f;
					//
					//			// actual ZMP location in ICS
					//
					//			actual_p_ZMP_ICS(0) = - actual_netForce_ICS(1)/actual_netForce_ICS(5);
					//			actual_p_ZMP_ICS(1) = actual_netForce_ICS(0)/actual_netForce_ICS(5);
					//			actual_p_ZMP_ICS(2) = 0;





					#ifdef KURMET_DEBUG
					cout<<"Joint Torques:  "<<endl<<" |rh| "<<G_robot_linkinfo_list[5]->link_val2.tau(0)
								  <<" |rk| "<<G_robot_linkinfo_list[6]->link_val2.tau(0)
								  <<" |lh| "<<G_robot_linkinfo_list[7]->link_val2.tau(0)
								  <<" |lk| "<<G_robot_linkinfo_list[8]->link_val2.tau(0)<<endl<<endl;
					#endif


					// some debug info
					#ifdef KURMET_DEBUG
					cout<< "a0? :"<<endl<<G_robot_linkinfo_list[0]->link_val2.a<<endl<<endl;
					cout<< "a1? :"<<endl<<G_robot_linkinfo_list[1]->link_val2.a<<endl<<endl;
					cout<< "a2? :"<<endl<<G_robot_linkinfo_list[2]->link_val2.a<<endl<<endl;
					#endif

				}
				else
				{


					// The following code use PD controller on leg joints, to give a feel
					// of what the desired ICS position/orientation should be. Only for testing.
					// torso
					Float q[1],qd[1];
					// Float tr[9][1];
					Float pGain, dGain;
					pGain = 30;
					dGain = 5;

					// right hip
					G_robot->getLink(5)->getState(q,qd);
					tr[5][0]= pGain * (-0.2 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(5)->setJointInput(tr[5]);
					// right knee
					G_robot->getLink(6)->getState(q,qd);
					tr[6][0]= pGain * (0.1 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(6)->setJointInput(tr[6]);

					// left hip
					G_robot->getLink(7)->getState(q,qd);
					tr[7][0]= pGain * (0.1 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(7)->setJointInput(tr[7]);
					// left knee
					G_robot->getLink(8)->getState(q,qd);
					tr[8][0]= pGain * (0.1 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(8)->setJointInput(tr[8]);

					G_robot->computeSpatialVelAndICSPose(2);
				}
				#endif

				#ifdef JOINT_POSITION_PD_ONLY
				Float q[1],qd[1];
				// Float tr[9][1];

				Float q5d, q6d, q7d, q8d;
				q5d = -0.423;
				q6d = 0.5538;
				q7d = 0.158;
				q8d = 0.5858;

				// right hip
				G_robot->getLink(5)->getState(q,qd);
				tr[5][0]= 30* (q5d - q[0])  - 5*(qd[0]);
				G_robot->getLink(5)->setJointInput(tr[5]);
				// right knee
				G_robot->getLink(6)->getState(q,qd);
				tr[6][0]= 30* (q6d - q[0])  - 5*(qd[0]);
				G_robot->getLink(6)->setJointInput(tr[6]);

				// left hip
				G_robot->getLink(7)->getState(q,qd);
				tr[7][0]= 30* (q7d - q[0])  - 5*(qd[0]);
				G_robot->getLink(7)->setJointInput(tr[7]);
				// left knee
				G_robot->getLink(8)->getState(q,qd);
				tr[8][0]= 30* (q8d - q[0])  - 5*(qd[0]);
				G_robot->getLink(8)->setJointInput(tr[8]);

				G_robot->computeSpatialVelAndICSPose(2);
				Matrix3F Rot_t_ICS;
				for (int m = 0; m< 3; m++)
				{
					for (int n = 0; n< 3; n++)
					{
						Rot_t_ICS(m,n)  = G_robot_linkinfo_list[2]->link_val2.R_ICS[m][n];
					}
				}

				SpatialVector f_torso_ext; // in torso coordinate
				if ( (sim_time > 5.50) && (sim_time < 5.56 ) )
				{
					// apply a distrubance
					Vector3F disturbance_f_ICS;
					Vector3F disturbance_n_ICS;
					disturbance_f_ICS << 80, 0, 0;
					disturbance_n_ICS <<  0, 0, 0;
					Vector3F disturbance_f_t;
					Vector3F disturbance_n_t;
					disturbance_f_t = Rot_t_ICS.inverse() * disturbance_f_ICS;
					disturbance_n_t = Rot_t_ICS.inverse() * disturbance_n_ICS;

					for (int k = 0; k<3; k++)
					{
						f_torso_ext[k] = disturbance_n_t[k];
						f_torso_ext[k+3] = disturbance_f_t[k];
					}

					dynamic_cast<dmRigidBody*>(G_robot->getLink(2))->setExternalForce(f_torso_ext);
				}
				else
				{
					for (int k = 0; k<6; k++)
					{
						f_torso_ext[k] = 0;
					}
					dynamic_cast<dmRigidBody*>(G_robot->getLink(2))->setExternalForce(f_torso_ext);
				}
				#endif

				control_count = 0;
			}


			// Data recording (at each integration step)
			if (sim_time>0)
			{
				  DataRecord * RecThisStep = new DataRecord;
				  SaveToDataRecord(RecThisStep);
				  MyVec.push_back(RecThisStep);
			}
		}

	}


	/// Graphics Rendering
	camera->update(mouse);
	camera->applyView();

	   // ****
	   // if you want the GL light to move with the camera, comment the following two lines - yiping
	   GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	   glLightfv (GL_LIGHT0, GL_POSITION, light_position);

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

	if (sim_time >6.0 && outputOnce == true)
	{
		simDataOutput(MyVec);
		outputOnce = false;
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
   char *filename = "windmill.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640,480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("Windmilling Experiment");

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

   //G_integrator = new dmIntegRK4();
   G_integrator = new dmIntegEuler();
   G_integrator->addSystem(G_robot);

   //initAquaControl(G_robot);

   for( int i =0;i<9; i++)
	   tr[i][0]=0;
   // Use with extra caution!!
   G_robot_linkinfo_list = G_robot->m_link_list;

    G_robot_linkinfo_list[6]->link_val2.tau = VectorXF::Zero(1);
	G_robot_linkinfo_list[5]->link_val2.tau = VectorXF::Zero(1);
	G_robot_linkinfo_list[8]->link_val2.tau = VectorXF::Zero(1);
	G_robot_linkinfo_list[7]->link_val2.tau = VectorXF::Zero(1);

	G_robot_linkinfo_list[5]->link_val2.qdd = VectorXF::Zero(1);
	G_robot_linkinfo_list[6]->link_val2.qdd = VectorXF::Zero(1);

	G_robot_linkinfo_list[7]->link_val2.qdd = VectorXF::Zero(1);
	G_robot_linkinfo_list[8]->link_val2.qdd = VectorXF::Zero(1);

	G_robot_linkinfo_list[2]->link_val2.qdd = VectorXF::Zero(1);
	G_robot_linkinfo_list[1]->link_val2.qdd = VectorXF::Zero(1);
	G_robot_linkinfo_list[0]->link_val2.qdd = VectorXF::Zero(1);

   glutReshapeFunc(myReshape);
   glutKeyboardFunc(processKeyboard);
   glutSpecialFunc(processSpecialKeys);
   glutDisplayFunc(display);
   glutIdleFunc(updateSim);

   dmGetSysTime(&last_tv);

   cout<<"kurmet has "<<G_robot->getNumLinks()<<" links."<<endl;
   cout<<"link 0 has "<<G_robot->getLink(0)->getNumDOFs()<<" DOFs."<<endl;

   // cout<<"link 1 's X is "<<endl<<G_robot->getLink(1)->get_X_FromParent_Motion()<<endl<<endl;
   // cout<<"link 0 's X is "<<endl<<G_robot->getLink(0)->get_X_FromParent_Motion()<<endl<<endl;
   // if (G_robot->getLinkParent(0) )
   // {
   //    cout<<"link 0's parent index is "<<G_robot->getLinkIndex(G_robot->getLinkParent(0) ) <<"."<<endl;
   // }
   // else
   // {
   //    cout<<"link 0's parent link is NULL"<<endl;
   // }
   
   cout<<"Torso Inertia M: "<<endl<< dynamic_cast <dmRigidBody*>(G_robot_linkinfo_list[2]->link)->getSpatialInertiaMatrix()<<endl<<endl;

   cout << endl;
   cout << "               p - toggles dynamic simulation" << endl;

   glutMainLoop();


   return 0;             /* ANSI C requires main to return int. */
}











//-----------------------------------------------------------------------------------------------------
// I am lazy here... should put this function to a separate file
void SaveToDataRecord(DataRecord *Rec)
{
	Rec->sim_time = sim_time;
	Float q[1],qd[1];

	for (int i=0; i<9;i++)
	{
		if ((i != 3) && (i != 4))
		{
			G_robot->getLink(i)->getState(q,qd);
			Rec->q[i] = q[0];
			Rec->qd[i] = qd[0];
		}
		if (i>4)
		{
			Rec->JointTorque[i] = tr[i][0];
		}
	}
	Rec->actual_torso_p_ICS[0] = G_robot_linkinfo_list[2]->link_val2.p_ICS[0];
	Rec->actual_torso_p_ICS[1] = G_robot_linkinfo_list[2]->link_val2.p_ICS[1];
	Rec->actual_torso_p_ICS[2] = G_robot_linkinfo_list[2]->link_val2.p_ICS[2];

	Rec->ref_torso_p_ICS[0] = 3.0;
	Rec->ref_torso_p_ICS[1] = 5.08;
	Rec->ref_torso_p_ICS[2] = 0.4935;

	for (int j=0; j<6;j++)
	{
		//Rec->error[j] = err(j);
		Rec->desired_torso_acc[j] = desired_torso_acc(j);
	}

	Rec->ZMPx_ICS = p_ZMP_ICS(0);
	Rec->actual_ZMPx_ICS = actual_p_ZMP_ICS(0);
	Rec->Rfx_ICS = p_RF_ICS[0];
	Rec->Lfx_ICS = p_LF_ICS[0];

	Rec->q0ss = q0ss; Rec->qd0ss = qd0ss;
	Rec->q1ss = q1ss; Rec->qd1ss = qd1ss;
	Rec->q2ss = q2ss; Rec->qd2ss = qd2ss;

	Rec->computed_tq2 = G_robot_linkinfo_list[2]->link_val2.f(2);

    Rec->tr[2][0] = tr[2][0];


}
