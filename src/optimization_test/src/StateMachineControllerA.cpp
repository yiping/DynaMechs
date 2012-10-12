

#include "StateMachineControllerA.h"

#define DEBUG_SMC_A
StateMachineControllerA::StateMachineControllerA(dmArticulation * robot) 
	: TaskSpaceControllerL(robot)
{
#ifdef DEBUG_SMC_A
	cout<<" -- StateMachineControllerA constructor --"<<endl;
#endif
	// initialize state machine
	cout<<"Total state machine states: "<<NUM_STATES<<endl;
	stateNames.resize(NUM_STATES); 
	stateNames[DROP] = "Drop";
	stateNames[BALANCE_MIDDLE] = "Balance Middle";	
	stateFunctions.resize(NUM_STATES);
	stateFunctions[DROP] = &StateMachineControllerA::Drop;	
	stateFunctions[BALANCE_MIDDLE] = &StateMachineControllerA::BalanceMiddle;	


	state = DROP;
	transitionFlag = true;

}

void StateMachineControllerA::Drop() 
{
	if (transitionFlag) // call once upon entering Drop state, the following settings are not used by Drop State, though
	{
		cout << setprecision(8) << endl;
		cout << "[Drop] - pCom: "<<pCom.transpose() << endl;
		
		// record the desired nominal positions for each joint - for next state use
		int numLinks = artic->getNumLinks();
		for (int i=0; i<numLinks; i++) 
		{
			LinkInfoStruct * bodyi = artic->m_link_list[i];
			int dof = bodyi->dof;	// true dof
			if (dof != 0) 
			{
				if (dof == 6) 		// FB
				{
					Vector3F om;
					om << 0,M_PI/20,0;
					matrixExpOmegaCross(om,RDesJoint[i]);	
				}
				else if (dof ==3)	//Quaternion link (stay with initial configuration)
				{
					CartesianTensor i_R_pi;
					CartesianVector pQLink;	// shoulder, hip
					Matrix3F i_R_pi_mat;
					bodyi->link->getPose(i_R_pi,pQLink);
					copyRtoMat(i_R_pi, i_R_pi_mat);
					RDesJoint[i] = i_R_pi_mat.transpose();	//RDesJoint : pi_Rd_i
				}
				else if (dof == 1)	// Revolute	link (stay with initial configuration)
				{
					Float qLink[0],qdLink[0];
					bodyi->link->getState(qLink, qdLink);
					posDesJoint[i](0) = qLink[0]; 
				}
			}
		}

		transitionFlag = false;
	}
	if (stateTime > .20) {
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}
	
}


void StateMachineControllerA::BalanceMiddle() 
{


	if (transitionFlag) 
	{
		cout <<"[BalanceMiddle] - pCom: "<< pCom.transpose() << endl;

		kpCM = 300;
		//kpCM = 30;
		kdCM = 2*sqrt(kpCM);
		kdAM = 25;
		pComDes << 2.00, 2.0, .48;	// predetermined nominal COM position
		//pComDes << 2.00, 2.0, .72;

		for (int i=0; i<NS; i++) 
		{
			kpFoot[i]=0;		// kpFoot, kdFoot are set zero when feet are in contact
			kdFoot[i]=0;
			//aDesFoot[i] << 0,0,0,0,0,0;
		
			//RDesFoot[i] <<  0, 0,  1, 
			//				0, 1, 0,
			//				-1, 0,  0;	// ankle 2 orientation in ICS
		}
		//pDesFoot[0] << 2,2,.01;			// 
		//pDesFoot[1] << 2.18,2,.01;	

		vComDes.setZero();
		aComDes.setZero();
		kDotDes.setZero();
		kComDes.setZero();
	
		//TaskSchedule.resize(1);
		//TaskSchedule<<2;
		TaskSchedule.resize(3);
		//TaskSchedule<<6,1,7;
		TaskSchedule<<6,2,3;

		transitionFlag = false;
	}

	if (stateTime > 500) 
	{
		state = BALANCE_MIDDLE ;
		transitionFlag = true;
		return;
	}


}




void StateMachineControllerA::StateControl()
{


	dmTimespec controlStart, controlEnd;
	dmGetSysTime(&controlStart);

	ControlInit();

	// Do at least one state call, and more if it transitioned out
	// 
	do 
	{
		if (transitionFlag) 
		{
			stateTime = 0;
		}
		(this->*stateFunctions[state])();	// function pointer
	} while (transitionFlag);


	if (state != DROP) 
	{
		TaskJacobians.resize(8); // currently only 8 possible objectives
		TaskBiases.resize(8);

		// Task 0 - (Centroidal) Angular momentum (AM) objective
		// Task 1 - (Centroidal) Linear momentum (LM) objective
		// Task 2 - Combined momentum objective
		{
			hDes.head(3) = kComDes;
			hDes.tail(3) = vComDes*totalMass;

			MatrixXF AmObj;
			MatrixXF AmBias;
			Vector3F angMomDotDes = kDotDes + kdAM*(kComDes-centMom.head(3)); 
			//Vector3F angMomDotDes = Vector3F::Zero(); 
			AmObj = MatrixXF::Zero(3, NVAR);
			AmObj.block(0,NJ,3,NJ+6) = CentMomMat.topRows(3);
			AmBias = -cmBias.segment(0,3);
			AmBias += angMomDotDes;

			TaskJacobians[0] = AmObj;
			TaskBiases[0] = AmBias;



			MatrixXF LmObj;
			MatrixXF LmBias;
			Vector3F linMomDotDes = totalMass*aComDes + totalMass*kpCM*(pComDes - pCom) + kdCM*(vComDes*totalMass - centMom.tail(3));
			//Vector3F linMomDotDes = Vector3F::Zero();
			LmObj = MatrixXF::Zero(3, NVAR);
			LmObj.block(0,NJ,3,NJ+6) = CentMomMat.bottomRows(3);
			LmBias = -cmBias.segment(3,3);
			LmBias += linMomDotDes;

			TaskJacobians[1] = LmObj;
			TaskBiases[1] = LmBias;

			hDotDes.head(3) = angMomDotDes;
			hDotDes.tail(3) = linMomDotDes;


			MatrixXF CentroidMomObj(6, NVAR);
			VectorXF CentroidMomBias(6);
			CentroidMomObj << AmObj, LmObj;
			CentroidMomBias << AmBias, LmBias;
			TaskJacobians[2] = CentroidMomObj;
			TaskBiases[2] = CentroidMomBias; 
		}

		// Task 3 - Nominal pose objective 
		{
			MatrixXF NpObj = MatrixXF::Zero(NJ+6, NVAR);
			VectorXF NpWeight = VectorXF::Zero(NJ+6);	// taskbias for nominal pose

			//////
			Float kpAnkle, kpKnee, kpHip, kpShoulder, kpElbow, kpTorso;
			Float wAnkle, wKnee, wHip, wShoulder, wElbow, wTorso;	// w - weight
	
			kpAnkle = 120;		
			kpKnee = 240;		
			kpHip = 120;		
			kpShoulder = 200;	
			kpElbow = 240;		
			kpTorso = 200;
			//kpTorso = 120./10.;	

			wAnkle = 1;
			wKnee = 1;
			wHip = .1;
			wShoulder = 1.5;
			wElbow = 1;
			wTorso = 1000;
			//wTorso = 1/2.;

			//////
			// torso  
			kpJoint[0] = kpTorso;	
			kdJoint[0] = 2*sqrt(kpTorso);	// critical damping
			NpWeight.segment(0,3).setConstant(wTorso);

			// right leg
			kpJoint[2] = kpHip;
			kdJoint[2] = 2*sqrt(kpHip);
			NpWeight.segment(6,3).setConstant(wHip);
					
			kpJoint[3] = kpKnee;
			kdJoint[3] = 2*sqrt(kpKnee);
			NpWeight(9) = wKnee;
					
			kpJoint[4] = kpAnkle;
			kdJoint[4] = 2*sqrt(kpAnkle);
			kpJoint[5] = kpAnkle;	
			kdJoint[5] = 2*sqrt(kpAnkle);
			NpWeight.segment(10,2).setConstant(wAnkle);

			// left leg
			kpJoint[6] = kpHip;
			kdJoint[6] = 2*sqrt(kpHip);
			NpWeight.segment(12,3).setConstant(wHip);
					
			kpJoint[7] = kpKnee;
			kdJoint[7] = 2*sqrt(kpKnee);
			NpWeight(15) = wKnee;
					
			kpJoint[8] = kpAnkle;
			kdJoint[8] = 2*sqrt(kpAnkle);
			kpJoint[9] = kpAnkle;
			kdJoint[9] = 2*sqrt(kpAnkle);
			NpWeight.segment(16,2).setConstant(wAnkle);

			// right arm
			kpJoint[10] = kpShoulder;
			kdJoint[10] = 2*sqrt(kpShoulder);
			NpWeight.segment(18,3).setConstant(wShoulder);
					
			kpJoint[11] = kpElbow;
			kdJoint[11] = 2*sqrt(kpElbow);
			NpWeight(21) = wElbow;
					
			// left arm
			kpJoint[12] = kpShoulder;
			kdJoint[12] = 2*sqrt(kpShoulder);
			NpWeight.segment(22,3).setConstant(wShoulder);
					
			kpJoint[13] = kpElbow;
			kdJoint[13] = 2*sqrt(kpElbow);
			NpWeight(25) = wElbow;



			//////

			CartesianTensor i_R_pi;
			CartesianVector pLink;
			Matrix3F i_R_pi_mat, Ra; // a -> actual
			Vector3F eOmega;
			VectorXF NpBias = VectorXF::Zero(NJ+6);

			// compute taskbias for each link 
			int jointIndexDm=0;
			for (int i=0; i<artic->getNumLinks(); i++) 	//	
			{
				LinkInfoStruct * bodyi = artic->m_link_list[i];
				int jointIndexE = bodyi->index_ext;	// extended index (ignore all the z-screws, incremented by true dof)
				int jointDof = bodyi->dof;			// true dof

				if (jointDof) 
				{
					Float Kp = kpJoint[i];
					Float Kd = kdJoint[i];
				
					if (jointDof == 1) 		// revolute links
					{
						NpBias(jointIndexE) = Kp * (posDesJoint[i](0) - q(jointIndexDm)) 
											 + Kd * (rateDesJoint[i](0) - qd(jointIndexE));
						if (i==3 || i==7) // knees
						{
							// a stub ...
						}
					}		
					else if (jointDof >= 3) 	// Quaternion or FB link
					{
						bodyi->link->getPose(i_R_pi,pLink);
						copyRtoMat(i_R_pi, i_R_pi_mat);
					
						Ra = i_R_pi_mat.transpose();	//  pi_Ra_i
					
						matrixLogRot(RDesJoint[i]*Ra.transpose(),eOmega);	// RDesJoint[i] is:  pi_Rd_i  |   Re = Rd Ra'
					
						// Note: omega is in "i" coordinates, error in p(i) coordinates
						Vector3F omega = qd.segment(jointIndexE,3);
					
						// Note: Velocity bias accel is omega cross omega  // YL ?
						NpBias.segment(jointIndexE,3) = (i_R_pi_mat*(Kp *eOmega) + Kd * (rateDesJoint[i].head(3) - omega)); // - omega.cross(omega);
					}

					// override the weight for FB link
					if (i == 0) // In DM, 0 link is floating base
					{
						NpWeight.segment(0,3).setConstant(10);	// only set weight to orientation of floating base
					}
				
				}
				jointIndexDm += bodyi->link->getNumDOFs();	// getNumDOFs() returns 7 for for FB and 4 for Quaternion links
			}

			MatrixXF NpObjTight = MatrixXF::Identity(NJ+6, NJ+6);
			NpObj.block(0,NJ,NJ+6,NJ+6) = NpWeight.asDiagonal() * NpObjTight;  
	   
			TaskJacobians[3] = NpObj;
			TaskBiases[3] = NpWeight.asDiagonal() * NpBias;
		}



		// Task 4, 5, 6 - Foot orientation/position objective
		{
			vector<MatrixXF> FObjs(NS);
			vector<VectorXF> FBiases(NS);
	
			MatrixXF X(6,6);  
			X.setZero();

			Matrix3F eR;
			Vector3F eOmega;
			int constraintRow = 0;

			Vector6F aFootCmd;
			MatrixXF footJacobian;
			Vector6F footAccBias;

			for (int i=0; i<NS; i++) 
			{
		
				int jointIndex = SupportIndices[i];		// not extended index :) 
				LinkInfoStruct * link = artic->m_link_list[jointIndex];

				// Compute the orientation Error
				eR = RDesFoot[i] * RFoot[i].transpose();
				matrixLogRot(eR,eOmega);
			
				// Use a PD to create a desired acceleration (everything here is in inertial coordinates)
				aFootCmd.head(3) = kpFoot[i] * eOmega;							// in ICS
				aFootCmd.tail(3) = kpFoot[i] * (pDesFoot[i] - pFoot[i]);		// in ICS
				aFootCmd += aDesFoot[i] + kdFoot[i] * (vDesFoot[i] - vFoot[i]);
			
				if (kdFoot[i] == 0) 		// YL ?		// when foot in contact with ground, kp and kd are set zero
				{
					aFootCmd = - 10 * vFoot[i];		
					// curent contact model requires foot motion to generate grf, but we also try to prevent foot from moving too fast, so here is the velocity damper
				}

				X.block(0,0,3,3) = RFoot[i]; 	// from Ankle frame to ICS
				X.block(3,3,3,3) = RFoot[i];
				artic->computeJacobian(jointIndex,X,footJacobian);
				computeAccBiasFromFwKin(link->link_val2,footAccBias);		// Acc bias (Jdot qdot)	[already been converted to classic acceleration]
			
				FObjs[i] = MatrixXF::Zero(6, NVAR);
				FObjs[i].block(0,NJ, 6, NJ+6) = footJacobian;
				FBiases[i] = VectorXF::Zero(6);
				FBiases[i] = aFootCmd - footAccBias;

			
				aDesFoot[i] = aFootCmd;		// YL ?
	
				// Option to Scale Linear/Angular Position Control

			}

			int cnt = 4;	// a quick workaround, need to be improved later
			for (int i=0; i<NS; i++)
			{
				TaskJacobians[cnt + i ] = FObjs[i];
				TaskBiases[cnt +i ] = FBiases[i];
			}

			TaskJacobians[6] = MatrixXF::Zero(12, NVAR);
			TaskBiases[6] = VectorXF::Zero(12);
			TaskJacobians[6].block(0,0, 6, NVAR) = TaskJacobians[4];
			TaskJacobians[6].block(6,0, 6, NVAR) = TaskJacobians[5];
			TaskBiases[6].segment(0,6) = TaskBiases[4];
			TaskBiases[6].segment(6,6) = TaskBiases[5];		
		} 

		{
			MatrixXF J7(3+NJ+6,NVAR);
			J7<< TaskJacobians[0], TaskJacobians[3];
			VectorXF b7(3+NJ+6);
			b7<< TaskBiases[0], TaskBiases[3];
			TaskJacobians[7] = J7; 
			TaskBiases[7] = b7; 
		}

		RobotControl();

		//exit(18);
		
		dmGetSysTime(&controlEnd);
		controlTime = timeDiff(controlStart, controlEnd);
	}	
	else 
	{
		//tau.setZero(26);

	}

	stateTime += sm_dt;
}
