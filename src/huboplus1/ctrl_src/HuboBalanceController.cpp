// HuboBalanceController.cpp
// Nov 27, 2012
// YL

#include "control_defs.h"
#include "HuboBalanceController.h"
#include "globalFunctions.h"
#include "globalVariables.h"




HuboBalanceController::HuboBalanceController(dmArticulation * robot) 
	: HuboController(robot, NUM_STATES)
{

	// initialize state machine
	cout<<"Total state machine states: "<<NUM_STATES<<endl;
	stateNames.resize(numStates);	//NUM_STATES
	stateNames[DROP] = "Drop";
	stateNames[BALANCE_MIDDLE] = "Balance Middle";	

	stateFunctions.resize(numStates);	//NUM_STATES
	stateFunctions[DROP] = &HuboBalanceController::Drop;	
	stateFunctions[BALANCE_MIDDLE] = &HuboBalanceController::BalanceMiddle;	

	
	state = DROP;
	transitionFlag = true;
	
}



void HuboBalanceController::Drop() 
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
					CartesianVector pQLink;
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
	}
	if (stateTime > .20) 
	{
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}
	transitionFlag = false;
}

void HuboBalanceController::BalanceMiddle() 
{
	if (transitionFlag) 
	{
		cout <<"[BalanceMiddle] - pCom: "<< pCom.transpose() << endl;

		kpCM = 300;
		kdCM = 2*sqrt(kpCM);
		kdAM = 25;
		pComDes << 2.00, 2.0, .56;	// predetermined nominal COM position
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

		transitionFlag = false;
	}


	if (stateTime > 500) 
	{
		state = BALANCE_MIDDLE;
		transitionFlag = true;
		return;
	}

}





void HuboBalanceController::StateControl()
{
	
	
	
	
	dmTimespec controlStart, controlEnd;
	dmGetSysTime(&controlStart);
	
	this->HumanoidController::ControlInit();
	
	OptimizationSchedule.setZero(6+NJ+6+12);
	OptimizationSchedule.segment(0,6).setConstant(3);		// cmm 3
	OptimizationSchedule.segment(6,NJ+6).setConstant(4);	// nominal pose 4
	OptimizationSchedule.segment(6+3,3).setConstant(-1);	// torso position: don't care
	OptimizationSchedule.tail(12).setConstant(2);			// feet positions 2

	//OptimizationSchedule.segment(0,6).setConstant(2);
	//OptimizationSchedule.segment(6,NJ+6).setConstant(2);
	//OptimizationSchedule.tail(12).setConstant(0);


	TaskWeight.setOnes(6+NJ+6+12);
	TaskWeight.tail(12).setConstant(1000);
	
	
	// Do at least one state call, and more if it transitioned out
	do 
	{
		if (transitionFlag) 
		{
			stateTime = 0;
		}
		(this->*stateFunctions[state])();
	} while (transitionFlag);
	
	if (state != DROP) 
	{
		int taskRow = 0;
		// Compute Centroidal Task Information
		{
			//Our task jacobain will be a desired pose + centroidal quantities
			TaskJacobian.block(0,0,6,NJ+6) = CentMomMat;
			
			// Compute Task Bias from RNEA Fw Kin
			TaskBias.segment(0,6) = -cmBias;
			
			hDes.head(3) = kComDes;
			hDes.tail(3) = vComDes*totalMass;
			
			Vector3F linMomDotDes = totalMass*aComDes + totalMass*kpCM*(pComDes - pCom) + kdCM*(vComDes*totalMass - centMom.tail(3));
			Vector3F angMomDotDes = kDotDes + kdAM*(kComDes-centMom.head(3));
			hDotDes.head(3) = angMomDotDes;
			hDotDes.tail(3) = linMomDotDes;
			
			// Dampen angular and PD CoM
			TaskBias.segment(0,3) += angMomDotDes;
			TaskBias.segment(3,3) += linMomDotDes;
			
			TaskWeight.segment(0,3).setConstant(100);
			TaskWeight.segment(3,3).setConstant(100);
			
	#ifdef CONTROL_DEBUG
			{
				//cout << "eCom " << (pComDes - pCom).transpose() << endl;
				//cout << "veCom " << (vComDes - centMom.tail(3) / totalMass) << endl;
				//cout << "adCom " << aComDes.transpose() << endl;
				cout << "pCoM " << pCom.transpose() << endl;
				cout << "cm " << centMom.transpose() << endl;
				cout << "vt " << qd.head(6).transpose() << endl;
				
				//cout << "Total Mass " << totalMass << endl;
				cout << "ldotdes " << linMomDotDes.transpose() << endl;
				//cout << "kdotdes " << angMomDotDes << endl;
			}
	#endif
			
			taskRow +=6;
		}
		
		//Compute Joint Task Information
		{

			TaskJacobian.block(taskRow,0,NJ+6,NJ+6) = MatrixXF::Identity(NJ+6,NJ+6);
			
			// Compute task bias based on PD on joint angles
			{
				// manually set joint PD gains and weights 
				Float kpAnkle, kpKnee, kpHip, kpShoulder, kpElbow, kpTorso, kpHead, kpPelvis, kpWrist;
				Float wAnkle, wKnee, wHip, wShoulder, wElbow, wTorso, wHead, wPelvis, wWrist;

				kpAnkle = 120;			wAnkle  = 1;
				kpKnee = 240;			wKnee   = 1;
				kpHip = 120;			wHip  = .1;
				kpShoulder = 200;		wShoulder  = 1.5;
				kpElbow = 240;			wElbow  = 1;
				kpTorso = 200;			wTorso = 1000;
				kpPelvis = 200;			wPelvis = 1.5;
				kpHead = 120;			wHead = 1;					
				kpWrist = 120;			wWrist = 1;
				{
					////TORSO///////////////////////////////////////////////+0
					kpJoint[0] = kpTorso;
					kdJoint[0] = 2*sqrt(kpTorso);
					TaskWeight.segment(taskRow,3).setConstant(wTorso);
					////PELVIS//////////////////////////////////////////////+6
					kpJoint[1] = kpPelvis;
					kdJoint[1] = 2*sqrt(kpPelvis);
					TaskWeight.segment(taskRow+6,1).setConstant(wPelvis);
					////RIGHT LEG///////////////////////////////////////////+7

					kpJoint[3] = kpHip;		kdJoint[3] = 2*sqrt(kpHip);
					kpJoint[5] = kpHip;		kdJoint[5] = 2*sqrt(kpHip);
					kpJoint[7] = kpHip;		kdJoint[7] = 2*sqrt(kpHip);
					TaskWeight.segment(taskRow+7,3).setConstant(wHip);
					
					kpJoint[8] = kpKnee;
					kdJoint[8] = 2*sqrt(kpKnee);
					TaskWeight(taskRow+10) = wKnee;
					
					kpJoint[9] = kpAnkle;	kdJoint[9] = 2*sqrt(kpAnkle);
					kpJoint[11] = kpAnkle;	kdJoint[11] = 2*sqrt(kpAnkle);
					TaskWeight.segment(taskRow+11,2).setConstant(wAnkle);
					////Left Leg////////////////////////////////////////////+13
					kpJoint[12] = kpHip;		kdJoint[12] = 2*sqrt(kpHip);
					kpJoint[14] = kpHip;		kdJoint[14] = 2*sqrt(kpHip);
					kpJoint[16] = kpHip;		kdJoint[16] = 2*sqrt(kpHip);
					TaskWeight.segment(taskRow+13,3).setConstant(wHip);
					
					kpJoint[17] = kpKnee;
					kdJoint[17] = 2*sqrt(kpKnee);
					TaskWeight(taskRow+16) = wKnee;
					
					kpJoint[18] = kpAnkle;	kdJoint[18] = 2*sqrt(kpAnkle);
					kpJoint[20] = kpAnkle;	kdJoint[20] = 2*sqrt(kpAnkle);
					TaskWeight.segment(taskRow+17,2).setConstant(wAnkle);
					////RIGHT ARM///////////////////////////////////////////+19
					kpJoint[21] = kpShoulder;	kdJoint[21] = 2*sqrt(kpShoulder);
					kpJoint[23] = kpShoulder;	kdJoint[23] = 2*sqrt(kpShoulder);
					kpJoint[25] = kpShoulder;	kdJoint[25] = 2*sqrt(kpShoulder);
					TaskWeight.segment(taskRow+19,3).setConstant(wShoulder);
					
					kpJoint[27] = kpElbow;
					kdJoint[27] = 2*sqrt(kpElbow);
					TaskWeight(taskRow+22) = wElbow;
					
					kpJoint[29] = kpWrist;	kdJoint[29] = 2*sqrt(kpWrist);
					kpJoint[31] = kpWrist;	kdJoint[31] = 2*sqrt(kpWrist);
					TaskWeight.segment(taskRow+23,2).setConstant(wWrist);
					////LEFT ARM////////////////////////////////////////////+25
					kpJoint[32] = kpShoulder;	kdJoint[32] = 2*sqrt(kpShoulder);
					kpJoint[34] = kpShoulder;	kdJoint[34] = 2*sqrt(kpShoulder);
					kpJoint[36] = kpShoulder;	kdJoint[36] = 2*sqrt(kpShoulder);
					TaskWeight.segment(taskRow+25,3).setConstant(wShoulder);
					
					kpJoint[38] = kpElbow;
					kdJoint[38] = 2*sqrt(kpElbow);
					TaskWeight(taskRow+28) = wElbow;
					
					kpJoint[40] = kpWrist;	kdJoint[40] = 2*sqrt(kpWrist);
					kpJoint[42] = kpWrist;	kdJoint[42] = 2*sqrt(kpWrist);
					TaskWeight.segment(taskRow+29,2).setConstant(wWrist);

					////HEAD////////////////////////////////////////////////+31
					kpJoint[43] = kpHead;
					kdJoint[43] = 2*sqrt(kpHead);
					TaskWeight(taskRow+31) = wHead;
					// 32-6 == 26 == NJ
				}
				
			}
			

			// Joint PDs
			int jointIndexDm=0;
			CartesianTensor i_R_pi;
			CartesianVector pLink;
			Matrix3F tmpR, i_R_pi_mat, RAct;
			Vector3F eOmega, om;
			
			for (int i=0; i<artic->getNumLinks(); i++) 	// go through all the links, include z-screws
			{
				LinkInfoStruct * bodyi = artic->m_link_list[i];
				int jointIndex = bodyi->index_ext;	// index counting true dof
				int jointDof = bodyi->dof;		// true dof
				
				if (jointDof)	// if not z-screw 
				{
					Float Kp = kpJoint[i];
					Float Kd = kdJoint[i];
					if (i == 0) 	// overwrite task weight for FB
					{
						TaskWeight.segment(taskRow,3).setConstant(10);
					}
					else if (jointDof == 1) 	// set taskbias for revolute link
					{
						TaskBias(taskRow+jointIndex) = Kp * (posDesJoint[i](0) - q(jointIndexDm)) 
															+ Kd * (rateDesJoint[i](0) - qd(jointIndex));
					}
					else if (jointIndex < 19) 	// if lower body, set task weight really low [overwrite]
					{
						TaskWeight.segment(taskRow+jointIndex,jointDof).setConstant(.1);
						//taskOptimActive.segment(taskRow+jointIndex,jointDof).setZero();
					}
					
					if (jointDof >= 3) 	// set taskbias for quat link or FB
					{
						bodyi->link->getPose(i_R_pi,pLink);
						copyRtoMat(i_R_pi, i_R_pi_mat);
						
						RAct = i_R_pi_mat.transpose();
						
						matrixLogRot(RDesJoint[i]*RAct.transpose(),eOmega);
						
						// Note: omega is in "i" coordinates, error in p(i) coordinates
						Vector3F omega = qd.segment(bodyi->index_ext,3);
						
						// Note: Velocity bias accel is omega cross omega
						TaskBias.segment(taskRow+jointIndex,3) = (i_R_pi_mat*(Kp *eOmega) + Kd * (rateDesJoint[i].head(3) - omega)) - omega.cross(omega);
					}
				}
				jointIndexDm += bodyi->link->getNumDOFs();	// fake dof
			}
			
			taskRow += NJ+6;
		}
		
		//Compute Foot Task Information
		{
			vector<MatrixXF > footJacs(NS);
			vector<Vector6F > footBias(NS);
			
			MatrixX6F X;
			X.resize(6,6);  X.block(0,3,3,3).setZero(); X.block(3,0,3,3).setZero();
			Matrix3F eR;

			Vector3F eOmega;
			Vector6F aFootCmd;
			
			for (int i=0; i<NS; i++) 
			{


				int jointIndex = SupportIndices[i];
				LinkInfoStruct * link = artic->m_link_list[jointIndex];

				// Compute the orientation Error
				eR = RDesFoot[i] * RFoot[i].transpose();
				matrixLogRot(eR,eOmega);
				
				// Use a PD to create a desired acceleration (everything here is in inertial coordinates)
				aFootCmd.head(3) = kpFoot[i] * eOmega;
				aFootCmd.tail(3) = kpFoot[i] * (pDesFoot[i] - pFoot[i]);
				aFootCmd += aDesFoot[i] + kdFoot[i] * (vDesFoot[i] - vFoot[i]);
				
				if (kdFoot[i] == 0) 
				{
					aFootCmd = - 10 * vFoot[i];
					//aFootCmd.setZero();
					// current contact model requires foot motion to generate grf, but we also try to prevent foot from moving too fast, so here is the velocity damper
				}

				
				// Compute Task Information
				X.block(0,0,3,3) = RFoot[i]; 
				X.block(3,3,3,3) = RFoot[i];
				artic->computeJacobian(jointIndex,X,footJacs[i]);
				computeAccBiasFromFwKin(link->link_val2,footBias[i]);// Acc bias (Jdot qdot)	[already been converted to classic acceleration]
				
				// Load Task Information
				TaskJacobian.block(taskRow,0,6,NJ+6) = footJacs[i];
				TaskBias.segment(taskRow,6)          = aFootCmd - footBias[i];
				
				aDesFoot[i] = aFootCmd;
				
				// Option to Scale Linear/Angular Position Control
				
				
				taskRow+=6;
			}
		}
		//cout << "Total Tasks! " << taskRow << endl;
		//cout << "Size " << TaskBias.size() << endl;
		HumanoidController::HumanoidControl();
		
		dmGetSysTime(&controlEnd);
		
		controlTime = timeDiff(controlStart, controlEnd);
		
		
		//cout << "Control Complete " << simThread->sim_time << endl;
		//exit(-1);
		
	}
	else 
	{
		tau.setZero(NJ);
		qdd.setZero(NJ+6);
		qddA.setZero(NJ+6);
		fs.setZero(6*NS);
	}

	
	if (frame->logDataCheckBox->IsChecked()) 
	{
		logData();
	}
	
	
	stateTime += simThread->cdt;
}
