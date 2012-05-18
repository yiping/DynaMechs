/*
 *****************************************************************************
 *     File: flywheel_biped.cpp
 *   Author: Yiping Liu
 *  Created: 16 Apr 2012
 *  Summary: Try to regain balance of a planar biped with a 
*            torso-mounted flywheel 
 *****************************************************************************/

#include "global.h"            
#include "function_prototypes.h"   

//#define BIPED_DEBUG
#define OUTPUT_DEBUG_INFO
//#define JOINT_POSITION_PD_ONLY



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

			    Rfoot = dynamic_cast<dmRigidBody*>(G_robot->getLink(6));
			    Rcontact = dynamic_cast<dmContactModel*>(Rfoot->getForce(0));
			    // force object 0 is instantiated to be the dmContactModel object.

				#ifndef JOINT_POSITION_PD_ONLY
				if (Rcontact->getContactState(0))//sim_time > 5.0
				{
					//begin postural control

					//// 0. Update variables

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

					RkneeVel_curr = G_robot_linkinfo_list[6]->link_val2.v;
					LkneeVel_curr = G_robot_linkinfo_list[8]->link_val2.v;
					TorsoVel_curr = G_robot_linkinfo_list[2]->link_val2.v;



					//// 1. Resolve torso Acceleration

					desired_torso_acc = resolveTorsoAcceleration();


					//// 2. compute Qdd
					// 2-a. reflect the spatial torso acceleration in joint space - joint 0,1,2
					G_robot_linkinfo_list[2]->link_val2.qdd(0) = desired_torso_acc(2);
					Float tq[1],tqd[1];
					G_robot->getLink(2)->getState(tq, tqd);
					G_robot_linkinfo_list[1]->link_val2.qdd(0) =
						(desired_torso_acc(3)*cos( tq[0])  + desired_torso_acc(4)*sin(tq[0]))/2.08;
					G_robot_linkinfo_list[0]->link_val2.qdd(0) =
						-(desired_torso_acc(3)*sin(tq[0]) + desired_torso_acc(4)*cos(tq[0]))/2.08 ;


					// 2-b. compute the desired qdd for the leg joints
					// right leg: joint 5,6
					computeBipedRightLegDesiredQdd();

					// left leg: joint 7,8
					G_robot_linkinfo_list[7]->link_val2.qdd(0) = 0;
					G_robot_linkinfo_list[8]->link_val2.qdd(0) = 0;

					// flywheel joint
					G_robot_linkinfo_list[9]->link_val2.qdd(0) = 0;

					#ifdef BIPED_DEBUG
					for (int g = 0; g<9; g++)
					{
						if ((g != 3) && (g!= 4))
							cout<<"qdd["<<g<<"]= "<<endl<<G_robot_linkinfo_list[g]->link_val2.qdd(0)<<endl;
					}
					cout<<endl;
					#endif

					//// 3.free space inverse dynamics, to figure out the spatial force on torso

					G_robot->inverseDynamics(false); //assuming no external forces

					Matrix6F XI2 = G_robot->computeSpatialTransformation(2);
					Matrix6F X2IF = XI2.transpose();
					f_torso_ICS = X2IF * (G_robot_linkinfo_list[2]->link_val2.f);
					f_torso = G_robot_linkinfo_list[2]->link_val2.f;


					// Computed ZMP location in ICS
					p_ZMP_ICS(0) = - f_torso_ICS(1)/f_torso_ICS(5);
					p_ZMP_ICS(1) =   f_torso_ICS(0)/f_torso_ICS(5);
					p_ZMP_ICS(2) = 0;


					// ZMP coordinate has the same orientation as ICS,
					// Right foot coordinate has the same orientation as right knee coordinate

					//
					Matrix3F Rot_t_rf = Rot_rf_ICS.inverse() * Rot_t_ICS;
					GRF_rf.tail(3) = Rot_t_rf * f_torso.tail(3);



					//// 4. Inverse dynamics again (inward recursion only), with GRF_rf applied at right foot.

					// travel back to torso, adding in the additional torque to each joint.

					Matrix6F X_rf;
					X_rf << Matrix3F::Identity(), Matrix3F::Zero(),
								      -cr3(prf), Matrix3F::Identity();
					Matrix6F X_rf6F = X_rf.transpose();
					G_robot_linkinfo_list[6]->link_val2.f -= X_rf6F * GRF_rf;
					G_robot_linkinfo_list[6]->link_val2.tau(0) = G_robot_linkinfo_list[6]->link_val2.f(2);
					Matrix6F X56 = G_robot->computeSpatialTransformation(6,6);
					Matrix6F X65F = X56.transpose();
					G_robot_linkinfo_list[5]->link_val2.f -= X65F * X_rf6F * GRF_rf;
					G_robot_linkinfo_list[5]->link_val2.tau(0) = G_robot_linkinfo_list[5]->link_val2.f(2);

					Matrix6F X25 = G_robot->computeSpatialTransformation(5,3);
					Matrix6F X52F = X25.transpose();

					Vector6F f_remain;
					f_remain = f_torso - X52F* X65F * X_rf6F * GRF_rf;
					cout<<" ["<< f_remain.transpose()<<"] "<<endl;

					#ifdef BIPED_DEBUG
					cout<<"GRF_rf = ["<< GRF_rf.transpose()<<"]"<<endl<<endl;
					cout<<"calculated free space torso force: "<<endl<< (G_robot_linkinfo_list[2]->link_val2.f).transpose()<<endl<<endl;
					Vector6F TorsoForce_canceled = G_robot_linkinfo_list[2]->link_val2.f
												- X52F* X65F * X_rf6F * GRF_rf;
					cout<<"check torso force after 2nd round ID: "<<endl<< TorsoForce_canceled.transpose()<<endl<<endl;
					#endif

					//// 5. Set joint torques
					tr[6][0] = G_robot_linkinfo_list[6]->link_val2.tau(0);
					G_robot->getLink(6)->setJointInput(tr[6]);
					tr[5][0] = G_robot_linkinfo_list[5]->link_val2.tau(0);
					G_robot->getLink(5)->setJointInput(tr[5]);

					// left leg
					Float q[1],qd[1];
					G_robot->getLink(7)->getState(q,qd);
					tr[7][0]= pGain * (0.7 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(7)->setJointInput(tr[7]);
					G_robot->getLink(8)->getState(q,qd);
					tr[8][0]= pGain * (0.3 - q[0])  - dGain *(qd[0]);
					G_robot->getLink(8)->setJointInput(tr[8]);

					// flywheel joint
					tr[9][0] = - f_remain(2);
					G_robot->getLink(9)->setJointInput(tr[9]);

					//make sure there are no torques on joint 0,1,2
					tr[0][0] = 0;	tr[1][0] = 0;	tr[2][0] = 0;
					G_robot->getLink(0)->setJointInput(tr[0]);
					G_robot->getLink(1)->setJointInput(tr[1]);
					G_robot->getLink(2)->setJointInput(tr[2]);

					#ifdef BIPED_DEBUG
					cout<<"Joint Torques:  "<<endl<<" |rh| "<<G_robot_linkinfo_list[5]->link_val2.tau(0)
								  <<" |rk| "<<G_robot_linkinfo_list[6]->link_val2.tau(0)
								  <<" |lh| "<<G_robot_linkinfo_list[7]->link_val2.tau(0)
								  <<" |lk| "<<G_robot_linkinfo_list[8]->link_val2.tau(0)<<endl<<endl;
					#endif



				}
				else
				{
					// PD
					Float q[1],qd[1];
					Float q5d, q6d, q7d, q8d;
					q5d = -0.1; q6d = 0.6; q7d = 0.7; q8d = 0.3;

					// right hip
					G_robot->getLink(5)->getState(q,qd);
					tr[5][0]= pGain * (q5d - q[0])  - dGain *(qd[0]);
					G_robot->getLink(5)->setJointInput(tr[5]);
					// right knee
					G_robot->getLink(6)->getState(q,qd);
					tr[6][0]= pGain * (q6d - q[0])  - dGain *(qd[0]);
					G_robot->getLink(6)->setJointInput(tr[6]);

					// left hip
					G_robot->getLink(7)->getState(q,qd);
					tr[7][0]= pGain * (q7d - q[0])  - dGain *(qd[0]);
					G_robot->getLink(7)->setJointInput(tr[7]);
					// left knee
					G_robot->getLink(8)->getState(q,qd);
					tr[8][0]= pGain * (q8d - q[0])  - dGain *(qd[0]);
					G_robot->getLink(8)->setJointInput(tr[8]);

					G_robot->computeSpatialVelAndICSPose(2);
				}
				#endif

				#ifdef JOINT_POSITION_PD_ONLY

				Float q[1],qd[1];

				Float q5d, q6d, q7d, q8d;
				q5d = -0.1; q6d = 0.6; q7d = 0.7; q8d = 0.3;
				//q5d = -0.6; q6d = 0.55; q7d = 0.2; q8d = 0.3;

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


				#endif

				control_count = 0;
			}

			CoM_pos_ICS = G_robot->computeCoM_ICS();

			// Data recording (at each integration step)
			if (sim_time>0)
			{
				  DataRecord * RecThisStep = new DataRecord;
				  SaveToDataRecord(RecThisStep);
				  MyVec.push_back(RecThisStep);
			}
		}

	}

	//CoM_pos_ICS = G_robot->computeCoM_ICS();
	
	/// Graphics Rendering
	camera->update(mouse);
	camera->applyView();

	   // ****
	   // if you want the GL light to move with the camera, comment the following two lines - yiping
	   GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	   glLightfv (GL_LIGHT0, GL_POSITION, light_position0);

	   GLfloat light_position1[] = { -1.0, -1.0, 1.0, 0.0 };
	   glLightfv (GL_LIGHT1, GL_POSITION, light_position1);

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

	if (sim_time >15.0 && outputOnce == true)
	{
		simDataOutput(MyVec);
		outputOnce = false;
	}

	//cout<<"--- --- --- "<<endl;
}







//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
	int i, j;

	glutInit(&argc, argv);

	//
	char *filename = "flywheel_biped.cfg";
	if (argc > 1)
	{
		filename = argv[1];
	}

	glutInitWindowSize(640,480);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("Flywheel Biped Experiment");

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

	

	readConfigParameterLabel(cfg_ptr,"Display_Update_Rate");
	cfg_ptr >> render_rate;
	if (render_rate < 1) render_rate = 1;

	// ------------------------------------------------------------------
	// Initialize DynaMechs environment - must occur before any linkage systems
	char env_flname[FILENAME_SIZE];
	readConfigParameterLabel(cfg_ptr,"Environment_Parameter_File");
	readFilename(cfg_ptr, env_flname);
	dmEnvironment *environment = dmuLoadFile_env(env_flname);
	environment->drawInit();
	dmEnvironment::setEnvironment(environment);

	// ------------------------------------------------------------------
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

	for(int i = 0; i<G_robot->getNumLinks(); i++)
	{	
		tr[i][0]=0;
		G_robot_linkinfo_list[i]->link_val2.tau = VectorXF::Zero(1);
		G_robot_linkinfo_list[i]->link_val2.qdd = VectorXF::Zero(1);
	}

	adjustBipedLegConfig(-0.1,0.6,0.7,0.3);
	//adjustBipedLegConfig(-0.6,0.55,0.2,0.3);

	readTorsoPoseSetpoints();

	glutReshapeFunc(myReshape);
	glutKeyboardFunc(processKeyboard);
	glutSpecialFunc(processSpecialKeys);
	glutDisplayFunc(display);
	glutIdleFunc(updateSim);

   	dmGetSysTime(&last_tv);

	cout<<"Biped has "<<G_robot->getNumLinks()<<" links."<<endl;
	cout<<"link 0 has "<<G_robot->getLink(0)->getNumDOFs()<<" DOFs."<<endl;
   
	cout<<"Torso Inertia M: "<<endl<< dynamic_cast <dmRigidBody*>(G_robot_linkinfo_list[2]->link)->getSpatialInertiaMatrix()<<endl<<endl;

	cout << endl;
	cout << "               p - toggles dynamic simulation" << endl;

	glutMainLoop();


	return 0;             /* ANSI C requires main to return int. */
}




