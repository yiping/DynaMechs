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



//#define KURMET_DEBUG

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
//dmIntegRK4 *G_integrator;
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
	char * displaytext = "Kurmet Postural Control";
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

//


//
void calculate_JdotQdot()
{

}

//----------------------------------------------------------------------------
void updateSim()
{
   if (!paused_flag)
   {
      for (int i=0; i<render_rate; i++)
      {



    	if (sim_time > 5.0)
    	{
			//postural control


			/// 1.calculate Kurmet torso spatial velocity
			//
			// expressed in torso coordinate
			G_robot->computeSpatialVelAndICSPose(2);
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
					R_RK_ICS[i][j] = G_robot_linkinfo_list[6]->link_val2.R_ICS[i][j]; //right knee Rostation ICS
					R_LK_ICS[i][j] = G_robot_linkinfo_list[8]->link_val2.R_ICS[i][j]; //leftt knee Rostation ICS
				}
			}
			#ifdef KURMET_DEBUG
			cout<<"p_RK_ICS = "<<endl<<p_RK_ICS[0]<<"  " <<p_RK_ICS[1]<<"  " <<p_RK_ICS[2] <<endl<<endl;

			#endif



			Vector6F TorsoVel_curr = G_robot_linkinfo_list[2]->link_val2.v;

			#ifdef KURMET_DEBUG
			cout<<"current torso spatial vel: "<<endl<<TorsoVel_curr<<endl<<endl;
			cout<<"current torso ICS position: "<<endl<<G_robot_linkinfo_list[2]->link_val2.p_ICS[0]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.p_ICS[1]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.p_ICS[2]<<endl<<endl;
			cout<<"current torso ICS orientation: "<<endl
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[0][0]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[0][1]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[0][2]<<endl
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[1][0]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[1][1]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[1][2]<<endl
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[2][0]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[2][1]<<" "
				<<G_robot_linkinfo_list[2]->link_val2.R_ICS[2][2]<<endl
				<<endl;
			#endif

			/// 2.figure out the desired acceleration
			//

			// 2.a Reference (desired) torso position and orientation (ICS)
			Vector3F ref_torso_p_ICS;
			ref_torso_p_ICS<< 3.0, 5.08, 0.5;  // 3.00347 5.07999 0.493448
			Vector3F ref_c1, ref_c2, ref_c3;
			ref_c1 << 0,
					  0,
					  1;
			ref_c2 << 1,
					  0,
					  0;
			ref_c3 << 0,
					  1,
					  0;


			// 2.b position and orientation error
			Vector6F err;
			err(3) = ref_torso_p_ICS[0] - G_robot_linkinfo_list[2]->link_val2.p_ICS[0];
			err(4) = ref_torso_p_ICS[1] - G_robot_linkinfo_list[2]->link_val2.p_ICS[1];
			err(5) = ref_torso_p_ICS[2] - G_robot_linkinfo_list[2]->link_val2.p_ICS[2];
			#ifdef KURMET_DEBUG
			cout<<"before = "<<endl<<err<<endl<<endl;
			#endif

			Vector3F c1,c2,c3; // columns of ref_torso_R_ICS
			c1 << G_robot_linkinfo_list[2]->link_val2.R_ICS[0][0],
					G_robot_linkinfo_list[2]->link_val2.R_ICS[1][0],
					G_robot_linkinfo_list[2]->link_val2.R_ICS[2][0];
			c2 << G_robot_linkinfo_list[2]->link_val2.R_ICS[0][1],
				  G_robot_linkinfo_list[2]->link_val2.R_ICS[1][1],
				  G_robot_linkinfo_list[2]->link_val2.R_ICS[2][1];
			c3 << G_robot_linkinfo_list[2]->link_val2.R_ICS[0][2],
				  G_robot_linkinfo_list[2]->link_val2.R_ICS[1][2],
				  G_robot_linkinfo_list[2]->link_val2.R_ICS[2][2];
			Matrix6F XI2 = G_robot->computeSpatialTransformation(2);
			err.head(3) = XI2.block(0,0,3,3) * (cr3(c1)* ref_c1 + cr3(c2)* ref_c2 + cr3(c3)* ref_c3); // there is a bias... lwp
			Vector3F temp1 = XI2.block(0,0,3,3)* err.tail(3);
			err.tail(3) = temp1;

			#ifdef KURMET_DEBUG
			cout<<"XI2 = "<<endl<<XI2<<endl<<endl;
			cout<<"orienation/pos error is: "<<endl<<err<<endl<<endl;
			#endif
			// 2.c PD gain
			Vector6F desired_torso_acc;
			Matrix6F Kd = Matrix6F::Zero();
			Matrix6F Kp = Matrix6F::Zero();
			Float k_wz, k_vx, k_vy;
			Float k_theta, k_px, k_py;
			k_wz = k_vx = k_vy = 20;
			k_theta = k_px = k_py = 100; // should be close to critically damped...
			Kd(2,2) = k_wz;
			Kd(3,3) = k_vx;
			Kd(4,4) = k_vy; // velocity error in torso coordinate
			Kp(2,2) = k_theta;
			Kp(3,3) = k_px;
			Kp(4,4) = k_py; // pos/ori error in ICS
			desired_torso_acc = Kd * ( - TorsoVel_curr) + Kp * (err);
			// the desired spatial acceleration of torso is expressed in local coordinate (a earth-fixed frame coinciding with...)
			//#ifdef KURMET_DEBUG
			cout<<"desired torso acceleration is: "<<endl<<desired_torso_acc<<endl<<endl;
			//#endif



			/// 3.calculate the desired qdd for the leg joints
			//
			// right leg:
			Matrix6F X_rf;
			Vector3F prf;
			prf<< 0.25,0, 0;
			X_rf.block<3,3>(0,0) = Matrix3F::Identity();
			X_rf.block<3,3>(0,3) = Matrix3F::Zero();
			X_rf.block<3,3>(3,0) = -cr3(prf);
			X_rf.block<3,3>(3,3) = Matrix3F::Identity();;
			Vector6F accBiasR;
			Matrix6XF JR;
			Matrix6F X_Rt; // spatial transformation from torso to right foot
			accBiasR = G_robot->computeAccelerationBias(6,X_rf,5);
			JR = G_robot->calculateJacobian(6,X_rf,5);
			X_Rt = X_rf*G_robot->computeSpatialTransformation(6,5);
			Vector2F temp2;
			Vector6F rhs1 = (- X_Rt * desired_torso_acc - accBiasR);
			#ifdef KURMET_DEBUG
			cout<<"JR is: "<<endl<<JR<<endl<<endl;
			cout<<"rhs1 is: "<<endl<<rhs1<<endl<<endl;
			#endif

			temp2 = JR.jacobiSvd(ComputeThinU | ComputeThinV).solve(rhs1);
			#ifdef KURMET_DEBUG
			cout<<"temp2a: "<<endl<<temp2<<endl<<endl;
			#endif

			G_robot_linkinfo_list[5]->link_val2.qdd(0) = temp2(0);
			G_robot_linkinfo_list[6]->link_val2.qdd(0) = temp2(1);
	//
			// left leg:
			Matrix6F X_lf;
			Vector3F plf;
			prf<< 0.25,0, 0;
			X_lf.block<3,3>(0,0) = Matrix3F::Identity();
			X_lf.block<3,3>(0,3) = Matrix3F::Zero();
			X_lf.block<3,3>(3,0) = -cr3(plf);
			X_lf.block<3,3>(3,3) = Matrix3F::Identity();;
			Vector6F accBiasL;
			Matrix6XF JL;
			Matrix6F X_Lt; // spatial transformation from torso to left foot
			accBiasL = G_robot->computeAccelerationBias(8,X_lf,7);
			JL = G_robot->calculateJacobian(8,X_lf,7);
			X_Lt = X_lf*G_robot->computeSpatialTransformation(8,7);
			Vector6F rhs2 = (- X_Lt * desired_torso_acc - accBiasL);
			#ifdef KURMET_DEBUG
			cout<<"JL is: "<<endl<<JR<<endl<<endl;
			cout<<"rhs2 is: "<<endl<<rhs2<<endl<<endl;
			#endif

			temp2 = JL.jacobiSvd(ComputeThinU | ComputeThinV).solve(rhs2);
			#ifdef KURMET_DEBUG
			cout<<"temp2b: "<<endl<<temp2<<endl<<endl;
			#endif

			G_robot_linkinfo_list[7]->link_val2.qdd(0) = temp2(0);
			G_robot_linkinfo_list[8]->link_val2.qdd(0) = temp2(1);

			// torso:
			//  from desired torso spatial acceleration to joint angle accelerations | any approximations??

			G_robot_linkinfo_list[2]->link_val2.qdd(0) = desired_torso_acc(2);
			G_robot_linkinfo_list[1]->link_val2.qdd(0) = desired_torso_acc(3)/2.08 ;
			G_robot_linkinfo_list[0]->link_val2.qdd(0) = -desired_torso_acc(5)/2.08 ;



			/// 4.free space inverse dynamics, to figure out the spatial force on torso
			//
			G_robot->inverseDynamics(); //no external forces



			/// 5.force distribution to feet
			//
			Vector6F netForce_ICS;
			Matrix6F X2IF = XI2.transpose();
			netForce_ICS = X2IF * G_robot_linkinfo_list[2]->link_val2.f;
			// calculate ZMP location in ICS
			Vector3F ZMP_ICS;
			ZMP_ICS(0) = - netForce_ICS(1)/netForce_ICS(5);
			ZMP_ICS(1) = netForce_ICS(0)/netForce_ICS(5);
			ZMP_ICS(2) = 0;
			// foot location in ICS
			// right:
			Vector3F p_RF_ICS;
			Vector3F p_LF_ICS;

			for (int i = 0; i < 3; i++)
			{
				p_RF_ICS(i) = p_RK_ICS[i] + 0.25*R_RK_ICS[i][0];
				p_LF_ICS(i) = p_LK_ICS[i] + 0.25*R_LK_ICS[i][0];
			}
			//#ifdef KURMET_DEBUG
			cout<<"Torso force w.r.t Torso coordinate"<<endl<<G_robot_linkinfo_list[2]->link_val2.f<<endl<<endl;
			cout<<"Torso force w.r.t ICS"<<endl<<netForce_ICS<<endl<<endl;
			cout<<"Right foot position ICS is: "<<endl<<p_RF_ICS<<endl<<endl;
			cout<<"left foot  position ICS is: "<<endl<<p_LF_ICS<<endl<<endl;
			cout<<"ZMP        position ICS is: "<<endl<<ZMP_ICS<<endl<<endl;
			//#endif

			//    	Vector6F netForce_ZMP;
			//    	Matrix6F XIZF;
			//    	XIZF << Matrix3F::Identity(), -crm(ZMP_ICS),
			//    			Matrix3F::Zero(), Matrix3F::Identity();
			//    	netForce_ZMP = XIZF * netForce_ICS;

			Vector6F netForce_Rf;
			Vector6F netForce_Lf;
			Matrix6F XIRfF;
			Matrix6F XILfF;

			XIRfF << Matrix3F::Identity(), -cr3(p_RF_ICS),
					Matrix3F::Zero(), Matrix3F::Identity();
			XILfF << Matrix3F::Identity(), -cr3(p_LF_ICS),
					Matrix3F::Zero(), Matrix3F::Identity();



			Float a = abs(p_RF_ICS[0] - ZMP_ICS(0) );
			Float b = abs(p_LF_ICS[0] - ZMP_ICS(0) );
			if (( max(p_RF_ICS[0], p_LF_ICS[0]) >ZMP_ICS(0) ) && ( ZMP_ICS(0)>min(p_RF_ICS[0], p_LF_ICS[0]) ) )
			{
				netForce_Rf = XIRfF * (b/(a+b)) * netForce_ICS; //
				netForce_Lf = XILfF * (a/(a+b)) * netForce_ICS; // No need though...
			}
			else
			{
				   cerr << " ZMP is outside support polygon!  " << endl;
				   exit(1);
			}

			/// 6.inverse dynamics again with the distributed force on each foot
			//
			// calculated external force for each foot needs to transfer to the corresponding knee joint frame.
			Matrix6F XILk = G_robot->computeSpatialTransformation(8);
			Matrix6F XIRk = G_robot->computeSpatialTransformation(6);


			Matrix6F XRkIF = XIRk.transpose();
			Vector6F netForce_Rk;
			netForce_Rk = XRkIF.partialPivLu().solve( (b/(a+b))*netForce_ICS);

			Matrix6F XLkIF = XILk.transpose();
			Vector6F netForce_Lk;
			netForce_Lk = XLkIF.partialPivLu().solve( (a/(a+b))*netForce_ICS );

			// travel back to torso, adding in the additional force to each joint.
			G_robot_linkinfo_list[6]->link_val2.f += netForce_Rk;
			G_robot_linkinfo_list[6]->link_val2.tau(0) = G_robot_linkinfo_list[6]->link_val2.f(2);
			Matrix6F X56 = G_robot->computeSpatialTransformation(6,6);
			Matrix6F X65F = X56.transpose();
			G_robot_linkinfo_list[5]->link_val2.f += X65F * G_robot_linkinfo_list[6]->link_val2.f;
			G_robot_linkinfo_list[5]->link_val2.tau(0) = G_robot_linkinfo_list[5]->link_val2.f(2);

			Matrix6F X25 = G_robot->computeSpatialTransformation(5,3);
			Matrix6F X52F = X25.transpose();

			G_robot_linkinfo_list[8]->link_val2.f += netForce_Lk;
			G_robot_linkinfo_list[8]->link_val2.tau(0) = G_robot_linkinfo_list[8]->link_val2.f(2);
			Matrix6F X78 = G_robot->computeSpatialTransformation(8,8);
			Matrix6F X87F = X78.transpose();
			G_robot_linkinfo_list[7]->link_val2.f += X87F * G_robot_linkinfo_list[8]->link_val2.f;
			G_robot_linkinfo_list[7]->link_val2.tau(0) = G_robot_linkinfo_list[7]->link_val2.f(2);

			Matrix6F X27 = G_robot->computeSpatialTransformation(7,3);
			Matrix6F X72F = X27.transpose();

			//verification!!
			G_robot_linkinfo_list[2]->link_val2.f;
			cout<<"calculated free space torso force: "<<endl<< (G_robot_linkinfo_list[2]->link_val2.f).transpose()<<endl<<endl;
			Vector6F expectedTorsoForce = G_robot_linkinfo_list[2]->link_val2.f - X52F*G_robot_linkinfo_list[5]->link_val2.f
												  - X72F*G_robot_linkinfo_list[7]->link_val2.f;
			cout<<"calculated torso force with contact: "<<endl<< expectedTorsoForce.transpose()<<endl<<endl;

			//set joint torques
			Float JointTorque[1];
			JointTorque[0] = G_robot_linkinfo_list[5]->link_val2.tau(0);
			G_robot->getLink(5)->setJointInput(JointTorque);
			JointTorque[0] = G_robot_linkinfo_list[6]->link_val2.tau(0);
			G_robot->getLink(6)->setJointInput(JointTorque);
			JointTorque[0] = G_robot_linkinfo_list[7]->link_val2.tau(0);
			G_robot->getLink(7)->setJointInput(JointTorque);
			JointTorque[0] = G_robot_linkinfo_list[8]->link_val2.tau(0);
			G_robot->getLink(8)->setJointInput(JointTorque);

			//    	for (int i = 0; i<3; i++) //link indices
			//    	{
			//    		Float q[1],qd[1];
			//    		Float tr[1];
			//    		G_robot->getLink(i)->getState(q,qd);
			//    	}


			///// A proposed test ///////////////////////////////////////////////////////
			// Calculate the torso spatial force for a 2D (planar), STATIC KURMET.
			// Assume qdd = 0 and qd = 0 for all joints.
			/////////////////////////////////////////////////////////////////////////////

			// some debug info
			#ifdef KURMET_DEBUG
			cout<< "a0? :"<<endl<<G_robot_linkinfo_list[0]->link_val2.a<<endl<<endl;
			cout<< "a1? :"<<endl<<G_robot_linkinfo_list[1]->link_val2.a<<endl<<endl;
			cout<< "a2? :"<<endl<<G_robot_linkinfo_list[2]->link_val2.a<<endl<<endl;
			#endif

    	}
    	else
    	{

    		//		Float q[1],qd[1];
    		//		Float tr[1];
    		//		G_robot->getLink(1)->getState(q,qd);
    		//		tr[0]= 30000* (1.5708 - q[0])  - 2000*(qd[0]);
    		//		G_robot->getLink(1)->setJointInput(tr);


    		//		G_robot->getLink(0)->getState(q,qd);
    		//		tr[0]= 10000* (- q[0])  - 2000*(qd[0]);
    		//		G_robot->getLink(0)->setJointInput(tr);

			// The following code use PD controller on torso and leg joints, to give a feel
			// of what the desired ICS position/orientation should be. Only for testing.
			// torso
			Float q[1],qd[1];
			Float tr[1];
			G_robot->getLink(2)->getState(q,qd);
			tr[0]= 0* (0 - q[0])  - 0*(qd[0]);
			G_robot->getLink(2)->setJointInput(tr);
			// right hip
			G_robot->getLink(5)->getState(q,qd);
			tr[0]= 100* (-0.2 - q[0])  - 20*(qd[0]);
			G_robot->getLink(5)->setJointInput(tr);
			// right knee
			G_robot->getLink(6)->getState(q,qd);
			tr[0]= 100* (0.1 - q[0])  - 20*(qd[0]);
			G_robot->getLink(6)->setJointInput(tr);

			// left hip
			G_robot->getLink(7)->getState(q,qd);
			tr[0]= 100* (0.1 - q[0])  - 20*(qd[0]);
			G_robot->getLink(7)->setJointInput(tr);
			// left knee
			G_robot->getLink(8)->getState(q,qd);
			tr[0]= 100* (0.1 - q[0])  - 20*(qd[0]);
			G_robot->getLink(8)->setJointInput(tr);
    	}

    	///
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
//    Summary:
// Parameters:
//    Returns:
//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
   int i, j;

   glutInit(&argc, argv);

   //
   char *filename = "kurmet.cfg";
   if (argc > 1)
   {
      filename = argv[1];
   }

   glutInitWindowSize(640,480);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutCreateWindow("KURMET");

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

   // control_stepsize =

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
   cout<<"link 1 's X is "<<endl<<G_robot->getLink(1)->get_X_FromParent_Motion()<<endl<<endl;
   cout<<"link 0 's X is "<<endl<<G_robot->getLink(0)->get_X_FromParent_Motion()<<endl<<endl;
   if (G_robot->getLinkParent(0) )
   {
      cout<<"link 0's parent index is "<<G_robot->getLinkIndex(G_robot->getLinkParent(0) ) <<"."<<endl;
   }
   else 
   {
      cout<<"link 0's parent link is NULL"<<endl;
   }
   
   cout << endl;
   cout << "               p - toggles dynamic simulation" << endl;

   glutMainLoop();
   return 0;             /* ANSI C requires main to return int. */
}
