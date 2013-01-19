/*
 *  humanoidDataLogging.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"
#include "HumanoidDataLogger.h"
#include "HumanoidController.h"
#include <wx/filefn.h>
#include "GlobalFunctions.h"

HumanoidDataLogger::HumanoidDataLogger(dmArticulation * robot, int stateSize) : HumanoidStateMachineController(robot,stateSize) {
	
	
	TIME	=		addItem("Time",					"t");
	STATE_CODE=		addItem("State",				"state");
	
	// Angles
	BASE_QUAT0=		addItem("Base Quaternion0",		"q(1)");
	BASE_QUAT1=		addItem("Base Quaternion1",		"q(2)");
	BASE_QUAT2=		addItem("Base Quaternion2",		"q(3)");
	BASE_QUAT3=		addItem("Base Quaternion3",		"q(4)");
	BASE_P_X=		addItem("Base Position X" ,		"q(5)");
	BASE_P_Y=		addItem("Base Position Y",		"q(6)");
	BASE_P_Z=		addItem("Base Position Z",		"q(7)");
	
	RHIP_QUAT0=		addItem("R Hip q0",				"q(8)");
	RHIP_QUAT1=		addItem("R Hip q1",				"q(9)");
	RHIP_QUAT2=		addItem("R Hip q2",				"q(10)");
	RHIP_QUAT3=		addItem("R Hip q3",				"q(11)");
	RKNEE=			addItem("R Knee Angle",			"q(12)");
	RANK1=			addItem("R Ank 1 Angle",		"q(13)");
	RANK2=			addItem("R Ank 2 Angle",		"q(14)");
	
	LHIP_QUAT0=		addItem("L Hip q0",				"q(15)");
	LHIP_QUAT1=		addItem("L Hip q1",				"q(16)");
	LHIP_QUAT2=		addItem("L Hip q2",				"q(17)");
	LHIP_QUAT3=		addItem("L Hip q3",				"q(18)");
	LKNEE=			addItem("L Knee Angle",			"q(19)");
	LANK1=			addItem("L Ank 1 Angle",		"q(20)");
	LANK2=			addItem("L Ank 2 Angle",		"q(21)");
	
	RSHOULD_QUAT0=		addItem("R Shoulder q0",		"q(22)");
	RSHOULD_QUAT1=		addItem("R Shoulder q1",		"q(23)");
	RSHOULD_QUAT2=		addItem("R Shoulder q2",		"q(24)");
	RSHOULD_QUAT3=		addItem("R Shoulder q3",		"q(25)");
	RELBOW=				addItem("R Elbow Angle",		"q(26)");
	
	LSHOULD_QUAT0=		addItem("R Shoulder Phi",		"q(27)");
	LSHOULD_QUAT1=		addItem("R Shoulder Psi",		"q(28)");
	LSHOULD_QUAT2=		addItem("R Shoulder Gamma",		"q(29)");
	LSHOULD_QUAT3=		addItem("R Shoulder Gamma",		"q(30)");
	LELBOW=				addItem("L Elbow Angle",		"q(31)");
	
	int angleItems[] = {BASE_QUAT0,BASE_QUAT1,BASE_QUAT2,BASE_QUAT3,
		BASE_P_X,BASE_P_Y,BASE_P_Z,
		RHIP_QUAT0,RHIP_QUAT1, RHIP_QUAT2,RHIP_QUAT3, RKNEE,RANK1,RANK2,
		LHIP_QUAT0,LHIP_QUAT1, LHIP_QUAT2,LHIP_QUAT3, LKNEE,LANK1,LANK2,
		RSHOULD_QUAT0,RSHOULD_QUAT1,RSHOULD_QUAT2,RSHOULD_QUAT3,RELBOW,
		LSHOULD_QUAT0,LSHOULD_QUAT1,LSHOULD_QUAT2,LSHOULD_QUAT3,LELBOW};
	IntVector angleGroup(angleItems,angleItems+sizeof(angleItems)/sizeof(int));
	JOINT_ANGLES = declareGroup("Joint Angles", angleGroup);
	
	BASE_OMEGA_X=		addItem("Base Omega X",			"qd(1)");
	BASE_OMEGA_Y=		addItem("Base Omega Y",			"qd(2)");
	BASE_OMEGA_Z=		addItem("Base Omega Z",			"qd(3)");
	BASE_V_X=			addItem("Base Vel. X",			"qd(4)");
	BASE_V_Y=			addItem("Base Vel. Y",			"qd(5)");
	BASE_V_Z=			addItem("Base Vel. Z",			"qd(6)");
	RHIP_OMEGA_X=		addItem("R Hip Omega X",		"qd(7)");
	RHIP_OMEGA_Y=		addItem("R Hip Omega Y",		"qd(8)");
	RHIP_OMEGA_Z=		addItem("R Hip Omega Z",		"qd(9)");
	RKNEE_RATE=			addItem("R Knee Rate",			"qd(10)");
	RANK1_RATE=			addItem("R Ank 1 Rate",			"qd(11)");
	RANK2_RATE=			addItem("R Ank 2 Rate",			"qd(12)");
	LHIP_OMEGA_X=		addItem("L Hip Omega X",		"qd(13)");
	LHIP_OMEGA_Y=		addItem("L Hip Omega Y",		"qd(14)");
	LHIP_OMEGA_Z=		addItem("L Hip Omega Z",		"qd(15)");
	LKNEE_RATE=		addItem("L Knee Rate",			"qd(16)");
	LANK1_RATE=		addItem("L Ank 1 Rate",			"qd(17)");
	LANK2_RATE=		addItem("L Ank 2 Rate",			"qd(18)");
	RSHOULD_OMEGA_X=		addItem("R Shoulder Omega X",	"qd(19)");
	RSHOULD_OMEGA_Y=		addItem("R Shoulder Omega Y",	"qd(20)");
	RSHOULD_OMEGA_Z=		addItem("R Shoulder Omega Z",	"qd(21)");
	RELBOW_RATE=		addItem("R Elbow Rate",			"qd(22)");
	LSHOULD_OMEGA_X=		addItem("L Shoulder Omega X",	"qd(23)");
	LSHOULD_OMEGA_Y=		addItem("L Shoulder Omega Y",	"qd(24)");
	LSHOULD_OMEGA_Z=		addItem("L Shoulder Omega Z",	"qd(25)");
	LELBOW_RATE=		addItem("L Elbow Rate",			"qd(26)");
	
	int rateItems[] = {BASE_OMEGA_X,BASE_OMEGA_Y,BASE_OMEGA_Z,BASE_V_X,BASE_V_Y,BASE_V_Z,
		RHIP_OMEGA_X,RHIP_OMEGA_Y,RHIP_OMEGA_Z,RKNEE_RATE,RANK1_RATE,RANK2_RATE,
		LHIP_OMEGA_X,LHIP_OMEGA_Y,LHIP_OMEGA_Z,LKNEE_RATE,LANK1_RATE,LANK2_RATE,
		RSHOULD_OMEGA_X,RSHOULD_OMEGA_Y,RSHOULD_OMEGA_Z,RELBOW_RATE,
		LSHOULD_OMEGA_X,LSHOULD_OMEGA_Y,LSHOULD_OMEGA_Z,LELBOW_RATE};
	
	IntVector rateGroup(rateItems,rateItems+sizeof(rateItems)/sizeof(int));
	JOINT_RATES = declareGroup("Joint Rates", rateGroup);
	
	RHIP_TAU_X=		addItem("R Hip Tau X",			"tau(1)");
	RHIP_TAU_Y=		addItem("R Hip Tau Y",			"tau(2)");
	RHIP_TAU_Z=		addItem("R Hip Tau Z",			"tau(3)");
	RKNEE_TAU=		addItem("R Knee Tau",			"tau(4)");
	RANK1_TAU=		addItem("R Ank 1 Tau",			"tau(5)");
	RANK2_TAU=		addItem("R Ank 2 Tau",			"tau(6)");
	LHIP_TAU_X=		addItem("L Hip Tau X",			"tau(7)");
	LHIP_TAU_Y=		addItem("L Hip Tau Y",			"tau(8)");
	LHIP_TAU_Z=		addItem("L Hip Tau Z",			"tau(9)");
	LKNEE_TAU=		addItem("L Knee Tau",			"tau(10)");
	LANK1_TAU=		addItem("L Ank 1 Tau",			"tau(11)");
	LANK2_TAU=		addItem("L Ank 2 Tau",			"tau(12)");
	RSHOULD_TAU_X=		addItem("R Shoulder Tau X",		"tau(13)");
	RSHOULD_TAU_Y=		addItem("R Shoulder Tau Y",		"tau(14)");
	RSHOULD_TAU_Z=		addItem("R Shoulder Tau Z",		"tau(15)");
	RELBOW_TAU=		addItem("R Elbow Tau",			"tau(16)");
	LSHOULD_TAU_X=		addItem("L Shoulder Tau X",		"tau(17)");
	LSHOULD_TAU_Y=		addItem("L Shoulder Tau Y",		"tau(18)");
	LSHOULD_TAU_Z=		addItem("L Shoulder Tau Z",		"tau(19)");
	LELBOW_TAU=		addItem("L Elbow Tau",			"tau(20)");
	
	
	int tauItems[] = {RHIP_TAU_X,RHIP_TAU_Y,RHIP_TAU_Z,RKNEE_TAU,RANK1_TAU,RANK2_TAU,
		LHIP_TAU_X,LHIP_TAU_Y,LHIP_TAU_Z,LKNEE_TAU,LANK1_TAU,LANK2_TAU,
		RSHOULD_TAU_X,RSHOULD_TAU_Y,RSHOULD_TAU_Z,RELBOW_TAU,
		LSHOULD_TAU_X,LSHOULD_TAU_Y,LSHOULD_TAU_Z,LELBOW_TAU};
	
	IntVector tauGroup(tauItems,tauItems+sizeof(tauItems)/sizeof(int));
	JOINT_TORQUES = declareGroup("Joint Torques", tauGroup);
	
	
	// Left CoP Group
	LCOP_F_X=		addItem("L CoP Force X",		"lCopForce(1)");
	LCOP_F_Y=		addItem("L CoP Force Y",		"lCopForce(2)");
	LCOP_F_Z=		addItem("L CoP Force Z",		"lCopForce(3)");
	LCOP_P_X=		addItem("L CoP Pos X",			"lCopPos(1)");
	LCOP_P_Y=		addItem("L CoP Pos Y",			"lCopPos(2)");
	LCOP_N_Z=		addItem("L CoP Mom Z",			"lCopNz");
	
	LCONT_STATE = addItem("L Contact State",			"lContState");
	LSLIDE_STATE = addItem("L Slide State",			"lSlideState");
	
	
	int leftCoPItems[] = {LCOP_F_X, LCOP_F_Y, LCOP_F_Z, LCOP_P_X, LCOP_P_Y, LCOP_N_Z};
	
	IntVector lCoPGroup(leftCoPItems,leftCoPItems+sizeof(leftCoPItems)/sizeof(int));
	LEFT_FOOT_WRENCH = declareGroup("Left Foot Wrench", lCoPGroup);
	
	
	// Right CoP Groups
	RCOP_F_X=		addItem("R CoP Force X",		"rCopForce(1)");
	RCOP_F_Y=		addItem("R CoP Force Y",		"rCopForce(2)");
	RCOP_F_Z=		addItem("R CoP Force Z",		"rCopForce(3)");
	RCOP_P_X=		addItem("R CoP Pos X",			"rCopPos(1)");
	RCOP_P_Y=		addItem("R CoP Pos Y",			"rlCopPos(2)");
	RCOP_N_Z=		addItem("R CoP Mom Z",			"rCopNz");
	
	RCONT_STATE = addItem("R Contact State",			"rContState");
	RSLIDE_STATE = addItem("R Slide State",			"rSlideState");
	
	int rightCoPItems[] = {RCOP_F_X, RCOP_F_Y, RCOP_F_Z, RCOP_P_X, RCOP_P_Y, RCOP_N_Z};
	
	IntVector rCoPGroup(rightCoPItems,rightCoPItems+sizeof(rightCoPItems)/sizeof(int));
	RIGHT_FOOT_WRENCH = declareGroup("Right Foot Wrench", rCoPGroup);
	
	// Zmp Group
	ZMP_F_X=		addItem("ZMP Force X",			"zmpForce(1)");
	ZMP_F_Y=		addItem("ZMP Force Y",			"zmpForce(2)");
	ZMP_F_Z=		addItem("ZMP Force Z",			"zmpForce(3)");
	ZMP_P_X=		addItem("ZMP Pos X",			"zmpPos(1)");
	ZMP_P_Y=		addItem("ZMP Pos Y",			"zmpPos(2)");
	ZMP_N_Z=		addItem("ZMP Mom Z",			"zmpNz");
	
	int zmpItems[] = {ZMP_F_X, ZMP_F_Y, ZMP_F_Z, ZMP_P_X, ZMP_P_Y, ZMP_N_Z};
	
	IntVector zmpGroup(zmpItems,zmpItems+sizeof(zmpItems)/sizeof(int));
	ZMP_WRENCH = declareGroup("ZMP Wrench", zmpGroup);

	CONTROL_TIME = addItem("Control Time", "controlTime");
	
	// Add Dynamic Groups
	COM_POSITION = addGroup("CoM Pos (Act)", "pCom",3);
	COM_POSITION_DES = addGroup("Com Pos (Des)", "pComDes", 3);
	
	COM_VELOCITY = addGroup("CoM Velocity", "vCom",3);
	COM_VELOCITY_DES = addGroup("CoM Velocity", "vComDes",3);
	
	LEFT_FOOT_POS = addGroup("Left Foot Pos", "lFootPos", 3);
	LEFT_FOOT_POS_DES = addGroup("Left Foot Pos (Des)", "lFootPosDes", 3);
	
	LEFT_FOOT_VEL = addGroup("Left Foot Vel", "lFootVel", 6);
	LEFT_FOOT_VEL_DES = addGroup("Left Foot Vel (Des)", "lFootVelDes", 6);
	
	LEFT_FOOT_ACC = addGroup("Left Foot Acc", "lFootAcc", 6);
	LEFT_FOOT_ACC_DES = addGroup("Left Foot Acc (Des)", "lFootAccDes", 6);
	
	RIGHT_FOOT_POS = addGroup("Right Foot Pos", "rFootPos", 3);
	RIGHT_FOOT_POS_DES = addGroup("Right Foot Pos (Des)", "rFootPosDes", 3);
	
	RIGHT_FOOT_VEL = addGroup("Right Foot Vel", "rFootVel", 6);
	RIGHT_FOOT_VEL_DES = addGroup("Right Foot Vel (Des)", "rFootVelDes", 6);
	
	RIGHT_FOOT_ACC     = addGroup("Right Foot Acc", "rFootAcc", 6);
	RIGHT_FOOT_ACC_DES = addGroup("Right Foot Acc (Des)", "rFootAccDes", 6);
	
	CENTROIDAL_MOMENTUM = addGroup("Centroidal Momentum",	"hCom",6);
	HDES				= addGroup("Cent. Mom. Des",		"hComDes",3);
	
	HDOT_DES			= addGroup("H Dot Des",				"hDotDes", 6);
	HDOT_OPT			= addGroup("H Dot Pot",				"hDotOpt",6);
	
	QDD_OPT				= addGroup("Qdd Opt",				"qddOpt", 26);
	QDD_ACT				= addGroup("Qdd Act",				"qddAct", 26);
	
	LWRENCH_OPT			= addGroup("Left Wrench Opt",		"lWrenchOpt",6);
	RWRENCH_OPT			= addGroup("Right Wrench Opt",		"rWrenchOpt",6);
	
	ZMP_WRENCH_OPT		= addGroup("Left Wrench Opt",		"zmpWrenchOpt",6);
	ZMP_POS_OPT			= addGroup("ZMP Pos Opt",		"zmpPosOpt",3);
	
	HMAT				= addMatrixGroup("H",						"H",26,26);
	
	JLF				= addMatrixGroup("Jleftfoot",			"Jlf",6,26);
	JRF             = addMatrixGroup("Jrightfoot", "Jrf",6,26);
	
	CANDG               = addGroup("CandG", "CandG", 26);
	
	LEFT_FOOT_SPATIAL_WRENCH  = addGroup("Left Foot Spatial Wrench", "lFootSpatWrench", 6);
	RIGHT_FOOT_SPATIAL_WRENCH = addGroup("Right Foot Spatial Wrench", "rFootSpatWrench", 6);
	
	TOTAL_ENERGY = addItem("Total Energy", "totalEnergy");
	KINETIC_ENERGY = addItem("Kinetic Energy", "kineticEnergy"); 
	POTENTIAL_ENERGY = addItem("Potential Energy", "potentialEnergy");
	CRB_KINETIC_ENERGY = addItem("CRB Kinetic Energy", "crbKineticEnergy");
	COM_KINETIC_ENERGY = addItem("CoM Kinetic Energy", "comKineticEnergy");
	
	
	AVG_ANGULAR_VELOCITY = addGroup("Avg. Ang. Velocity", "avgAngVel", 3);
	
	LEFT_LEG_LENGTH = addItem("Left Leg Length", "lLegLength"); 
	RIGHT_LEG_LENGTH = addItem("Right Leg Length", "rLegLength");
	
	SLIP_SPRING_ENERGY = addItem("SLIP Spring Energy","slipSpringEnergy");
	SLIP_KINETIC_ENERGY = addItem("SLIP Kinetic Energy","slipKineticEnergy");
	SLIP_POTENTIAL_ENERGY = addItem("SLIP Potential Energy","slipPotentialEnergy");
	
	STEP_NUM = addItem("Step Number" ,"stepNum");
}


void HumanoidDataLogger::logData() {
	
	dataMutex.Lock();
	newRecord();
	assignItem(TIME, simThread->sim_time);
	assignItem(STATE_CODE, state);
	assignItem(CONTROL_TIME, controlTime);
	
	assignGroup(JOINT_ANGLES, q);
	assignGroup(JOINT_RATES, qd);
	assignGroup(JOINT_TORQUES, tau);
	Vector6F force;
	
	force.head(3) = grfInfo.fCoPs[0];
	force.segment(3,2) = grfInfo.pCoPs[0].head(2);
	force(5)     = grfInfo.nCoPs[0];
	assignGroup(RIGHT_FOOT_WRENCH, force);
	assignItem(RCONT_STATE, contactState[0]);
	assignItem(RSLIDE_STATE, slidingState[0]);
	
	force.head(3) = grfInfo.fCoPs[1];
	force.segment(3,2) = grfInfo.pCoPs[1].head(2);
	force(5)     = grfInfo.nCoPs[1];
	assignGroup(LEFT_FOOT_WRENCH, force);
	assignItem(LCONT_STATE, contactState[1]);
	assignItem(LSLIDE_STATE, slidingState[1]);
	
	force.head(3) = grfInfo.fZMP;
	force.segment(3,2) = grfInfo.pZMP.head(2);
	force(5)     = grfInfo.nZMP;
	assignGroup(ZMP_WRENCH, force);
	
	assignGroup(ZMP_WRENCH_OPT, zmpWrenchOpt);
	assignGroup(ZMP_POS_OPT, zmpPosOpt);
	
	assignGroup(COM_POSITION, pCom);
	assignGroup(COM_POSITION_DES, pComDes);
	assignGroup(COM_VELOCITY, vCom);
	assignGroup(COM_VELOCITY_DES, vComDes);
	
	assignGroup(CENTROIDAL_MOMENTUM, centMom);
	assignGroup(HDES, hDes);
	
	assignGroup(HDOT_DES, hDotDes);
	assignGroup(HDOT_OPT, hDotOpt);
	
	assignGroup(QDD_OPT, qdd);
	assignGroup(QDD_ACT, qddA);
	
	
	assignGroup(RIGHT_FOOT_POS, pFoot[0]);
	assignGroup(RIGHT_FOOT_POS_DES, pDesFoot[0]);
	
	assignGroup(RIGHT_FOOT_VEL, vFoot[0]);
	assignGroup(RIGHT_FOOT_VEL_DES, vDesFoot[0]);
	
	assignGroup(RIGHT_FOOT_ACC, aFoot[0]);
	assignGroup(RIGHT_FOOT_ACC_DES, aDesFoot[0]);
	
	assignGroup(LEFT_FOOT_POS, pFoot[1]);
	assignGroup(LEFT_FOOT_POS_DES, pDesFoot[1]);
	
	assignGroup(LEFT_FOOT_VEL, vFoot[1]);
	assignGroup(LEFT_FOOT_VEL_DES, vDesFoot[1]);
	
	assignGroup(LEFT_FOOT_ACC, aFoot[1]);
	assignGroup(LEFT_FOOT_ACC_DES, aDesFoot[1]);
	
	assignGroup(RWRENCH_OPT, fs.head(6));
	assignGroup(LWRENCH_OPT, fs.tail(6));
	
	assignMatrixGroup(JRF, grfInfo.footJacs[0]);
	assignMatrixGroup(JLF, grfInfo.footJacs[1]);
	
	assignGroup(RIGHT_FOOT_SPATIAL_WRENCH, grfInfo.footWrenches[0]);
	assignGroup(LEFT_FOOT_SPATIAL_WRENCH, grfInfo.footWrenches[1]);
	
	assignGroup(CANDG, artic->CandG);
	assignMatrixGroup(HMAT,artic->H);
	
	
	assignItem(TOTAL_ENERGY, totalEnergy);
	assignItem(KINETIC_ENERGY, kineticEnergy);
	assignItem(POTENTIAL_ENERGY, potentialEnergy);
	assignItem(CRB_KINETIC_ENERGY, crbKineticEnergy);
	assignItem(COM_KINETIC_ENERGY, comKineticEnergy);
	assignGroup(AVG_ANGULAR_VELOCITY, avgAngVelocity);
	
	assignItem(LEFT_LEG_LENGTH, lLegLength);
	assignItem(RIGHT_LEG_LENGTH, rLegLength);
	
	assignItem(SLIP_SPRING_ENERGY,slipSpringEnergy);
	assignItem(SLIP_KINETIC_ENERGY,slipKineticEnergy);
	assignItem(SLIP_POTENTIAL_ENERGY,slipPotentialEnergy);
	
	assignItem(STEP_NUM,stepNum);
	
	
	dataMutex.Unlock();
}

void HumanoidDataLogger::saveData()
{
	const wxDateTime now = wxDateTime::UNow();
	
	wxString dataDirectory(dataSaveDirectory.c_str(),wxConvUTF8);
	wxString curFilePath = dataDirectory + wxT("/recentData.dat");
	
	dataDirectory += now.FormatISODate();
	wxString dataFile =  now.FormatISODate() + wxT("_") + now.FormatISOTime() + wxT(".dat");
	dataFile.Replace(wxT(":"), wxT("-"));
	
	wxString dataPath = dataDirectory + wxT("/") + dataFile;
	
	if(! wxDirExists(dataDirectory))
	{
		wxMkdir(dataDirectory);	
	}
	
	stringstream ss;
	ss << dataPath.mb_str();
	setFile(ss.str());
	writeRecords();
	
	FILE * curFile = fopen(curFilePath.mb_str(),"w");
	fprintf(curFile, "%s",ss.str().c_str());
	fclose(curFile);
}