/*
 *****************************************************************************
 *     File: functions_D.cpp
 *   Author: Yiping Liu
 *  Created: 18 Apr 2012
 *  Summary: Biped control related functions   
 *            
 *****************************************************************************/

// ALERT:
// All functions in this file are specific to the biped model
 
#include "global.h" 


// ---------------------------------------------------------------------------
/// Calculate the actual ZMP location
void calculateActualZMP(void)
{
	// - get the actual ground contact forces
	SpatialVector rf_actual_contact_force;
	SpatialVector lf_actual_contact_force;
	dynamic_cast <dmRigidBody*>(G_robot->getLink(6))->getForce(0)->computeForce(G_robot_linkinfo_list[6]->link_val2, rf_actual_contact_force);
	dynamic_cast <dmRigidBody*>(G_robot->getLink(8))->getForce(0)->computeForce(G_robot_linkinfo_list[8]->link_val2, lf_actual_contact_force);

	// - get the actual contact states
	bool rf_in_contact, lf_in_contact;
	rf_in_contact = dynamic_cast <dmContactModel*>(dynamic_cast <dmRigidBody*>(G_robot->getLink(6))->getForce(0))->getContactState(0);
	lf_in_contact = dynamic_cast <dmContactModel*>(dynamic_cast <dmRigidBody*>(G_robot->getLink(8))->getForce(0))->getContactState(0);

	Matrix6F XI6 = G_robot->computeSpatialTransformation(6);
	Matrix6F XI8 = G_robot->computeSpatialTransformation(8);

	Matrix6F X6IF = XI6.transpose();
	Matrix6F X8IF = XI8.transpose();

	Vector6F actual_rf_f;
	Vector6F actual_lf_f;

	for (int i =0; i<6; i++)
	{
		actual_rf_f(i) = rf_actual_contact_force[i];
		actual_lf_f(i) = lf_actual_contact_force[i];
	}

	Vector6F actual_netForce_ICS;
	actual_netForce_ICS = X6IF * actual_rf_f + X8IF * actual_lf_f;

	// actual ZMP location in ICS

	actual_p_ZMP_ICS(0) = - actual_netForce_ICS(1)/actual_netForce_ICS(5);
	actual_p_ZMP_ICS(1) = actual_netForce_ICS(0)/actual_netForce_ICS(5);
	actual_p_ZMP_ICS(2) = 0;

}


// ---------------------------------------------------------------------------
// used at initialization stage to overide the theta angle [read from dm file]
void adjustBipedLegConfig(Float rh, Float rk, Float lh, Float lk)
{
	Float qd = 0;
	G_robot->getLink(5)->setState(&rh, &qd);
	G_robot->getLink(6)->setState(&rk, &qd);
	G_robot->getLink(7)->setState(&lh, &qd);
	G_robot->getLink(8)->setState(&lk, &qd);

	G_integrator->synchronizeState();
}


// ---------------------------------------------------------------------------
// apply torso disturbance
void applyTorsoDisturbance(Float tA, Float tB, Vector3F disturbance_f_ICS, Vector3F disturbance_n_ICS)
{
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
	if ( (sim_time > tA) && (sim_time < tB ) )  //eg. tA = 5.50, tB = 5.56
	{
		// apply a distrubance

		//Vector3F disturbance_f_ICS;
		//Vector3F disturbance_n_ICS;
		//disturbance_f_ICS << 80, 0, 0;
		//disturbance_n_ICS <<  0, 0, 0;
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
}



// ---------
void computeBipedRightLegDesiredQdd()
{
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
	accBiasR = G_robot->computeAccelerationBias(6,X_rf,3);
	JR = G_robot->calculateJacobian(6,X_rf,3);
	X_TR = X_rf*G_robot->computeSpatialTransformation(6,3);
	Vector2F qdd_R;
	Vector3F omega_rf;
	omega_rf = RfVel.head(3);
	Vector6F b_term_R = Vector6F::Zero();
	b_term_R.tail(3) = cr3(omega_rf)*RfVel.tail(3);
	Vector2F rhs1 = S*(- X_TR * desired_torso_acc - accBiasR - b_term_R );

	//qdd_R = solveJacobianPseudoInverse(S*JR, rhs1);
	qdd_R = (S*JR).inverse()*rhs1;

	#ifdef BIPED_DEBUG
	cout<<"JR is: "<<endl<<JR<<endl<<endl;
	cout<<"rhs1 is: "<<endl<<rhs1<<endl<<endl;
	cout<<"qdd_R: "<<endl<<qdd_R<<endl<<endl;
	#endif

	G_robot_linkinfo_list[5]->link_val2.qdd(0) = qdd_R(0);
	G_robot_linkinfo_list[6]->link_val2.qdd(0) = qdd_R(1);
}



// -----------
void computeBipedLeftLegDesiredQdd()
{
	Matrix6F X_lf;
	MatrixXF S(2,6);
	S<< 0,0,0,1,0,0,
	    0,0,0,0,1,0;

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

	#ifdef BIPED_DEBUG
	cout<<"JL is: "<<endl<<JL<<endl<<endl;
	cout<<"rhs2 is: "<<endl<<rhs2<<endl<<endl;
	cout<<"qdd_L: "<<endl<<qdd_L<<endl<<endl;
	#endif

	G_robot_linkinfo_list[7]->link_val2.qdd(0) = qdd_L(0);
	G_robot_linkinfo_list[8]->link_val2.qdd(0) = qdd_L(1);
}


Vector6F resolveTorsoAcceleration()
{
	Vector6F desired_torso_acceleration = Vector6F::Zero();
	// Desired torso position and orientation (ICS)
	Vector6F poseError;

	Vector3F ref_torso_p_ICS;
	Vector3F ref_torso_R_ICS_c1, ref_torso_R_ICS_c2, ref_torso_R_ICS_c3;// columns of ref_torso_R_ICS
	Vector3F c1,c2,c3;

	ref_torso_p_ICS<< 3.0, 5.08, 0.46;

	ref_torso_R_ICS_c1 << 0, 0, 1;
	ref_torso_R_ICS_c2 << 1, 0, 0;
	ref_torso_R_ICS_c3 << 0, 1, 0; //upright

	poseError(3) = ref_torso_p_ICS[0] - G_robot_linkinfo_list[2]->link_val2.p_ICS[0];
	poseError(4) = ref_torso_p_ICS[1] - G_robot_linkinfo_list[2]->link_val2.p_ICS[1];
	poseError(5) = ref_torso_p_ICS[2] - G_robot_linkinfo_list[2]->link_val2.p_ICS[2];

	for (int g = 0; g<3;g++)
	{
		c1(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][0];
		c2(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][1];
		c3(g) = G_robot_linkinfo_list[2]->link_val2.R_ICS[g][2];
	}

	Matrix6F XI2 = G_robot->computeSpatialTransformation(2);
	poseError.head(3) = XI2.block(0,0,3,3) * (0.5*(cr3(c1)* ref_torso_R_ICS_c1 + cr3(c2)* ref_torso_R_ICS_c2 + cr3(c3)* ref_torso_R_ICS_c3)); // there is a bias... lwp
	poseError.tail(3) = XI2.block(0,0,3,3)* poseError.tail(3); //in torso coordinate

	Float kp, kd;// PD gain
	kd = 15;
	kp = 100;  //
	desired_torso_acceleration(2) = 0.02*kp * poseError(2) + 0.02*kd * ( - TorsoVel_curr(2)); // soft gain on torso pitch
	desired_torso_acceleration(3) = 0.8*kp * poseError(3) +  0.8*kd * ( - TorsoVel_curr(3));
	desired_torso_acceleration(4) = 0.5*kp * poseError(4) + 0.5*kd * ( - TorsoVel_curr(4));

	#ifdef BIPED_DEBUG
	cout<<"desired spatial torso acceleration is: "<<endl<<desired_torso_acceleration<<endl<<endl;
	#endif
	return desired_torso_acceleration;
}
