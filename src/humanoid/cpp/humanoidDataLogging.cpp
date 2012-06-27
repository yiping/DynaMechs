/*
 *  humanoidDataLogging.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "humanoidDataLogging.h"
#include "humanoidControl.h"
#include <wx/filefn.h>
#include "GlobalFunctions.h"

DataLogger dataLog;
// Dynamic Groups
int COM_POSITION, COM_VELOCITY, COM_POSITION_DES, CENTROIDAL_MOMENTUM, QDD_OPT, HMAT, HDOT_DES, HDOT_OPT;


void initializeDataLogging() {
	
	dataLog.setMaxItems(MAX_STATIC_ITEMS);
	dataLog.setMaxGroups(MAX_STATIC_GROUPS);
	
	dataLog.setItemName(TIME,			"Time",					"t");
	
	// Angles
	dataLog.setItemName(BASE_QUAT0,		"Base Quaternion0",		"q(1)");
	dataLog.setItemName(BASE_QUAT1,		"Base Quaternion1",		"q(2)");
	dataLog.setItemName(BASE_QUAT2,		"Base Quaternion2",		"q(3)");
	dataLog.setItemName(BASE_QUAT3,		"Base Quaternion3",		"q(4)");
	dataLog.setItemName(BASE_P_X,		"Base Position X" ,		"q(5)");
	dataLog.setItemName(BASE_P_Y,		"Base Position Y",		"q(6)");
	dataLog.setItemName(BASE_P_Z,		"Base Position Z",		"q(7)");
	dataLog.setItemName(RHIP_PHI,		"R Hip Phi",			"q(8)");
	dataLog.setItemName(RHIP_PSI,		"R Hip Psi",			"q(9)");
	dataLog.setItemName(RHIP_GAMMA,		"R Hip Gamma",			"q(10)");
	dataLog.setItemName(RKNEE,			"R Knee Angle",			"q(11)");
	dataLog.setItemName(RANK1,			"R Ank 1 Angle",		"q(12)");
	dataLog.setItemName(RANK2,			"R Ank 2 Angle",		"q(13)");
	dataLog.setItemName(LHIP_PHI,		"L Hip Phi",			"q(14)");
	dataLog.setItemName(LHIP_PSI,		"L Hip Psi",			"q(15)");
	dataLog.setItemName(LHIP_GAMMA,		"L Hip Gamma",			"q(16)");
	dataLog.setItemName(LKNEE,			"L Knee Angle",			"q(17)");
	dataLog.setItemName(LANK1,			"L Ank 1 Angle",		"q(18)");
	dataLog.setItemName(LANK2,			"L Ank 2 Angle",		"q(19)");
	dataLog.setItemName(RSHOULD_PHI,	"R Shoulder Phi",		"q(20)");
	dataLog.setItemName(RSHOULD_PSI,	"R Shoulder Psi",		"q(21)");
	dataLog.setItemName(RSHOULD_GAMMA,	"R Shoulder Gamma",		"q(22)");
	dataLog.setItemName(RELBOW,			"R Elbow Angle",		"q(23)");
	dataLog.setItemName(LSHOULD_PHI,	"L Shoulder Phi",		"q(24)");
	dataLog.setItemName(LSHOULD_PSI,	"L Shoulder Psi",		"q(25)");
	dataLog.setItemName(LSHOULD_GAMMA,	"L Shoulder Gamma",		"q(26)");
	dataLog.setItemName(LELBOW,			"L Elbow Angle",		"q(27)");
	
	int angleItems[] = {BASE_QUAT0,BASE_QUAT1,BASE_QUAT2,BASE_QUAT3,
		BASE_P_X,BASE_P_Y,BASE_P_Z,RHIP_PHI,RHIP_PSI, RHIP_GAMMA,
		RKNEE,RANK1,RANK2,LHIP_PHI,LHIP_PSI,LHIP_GAMMA,LKNEE,LANK1,LANK2,
		RSHOULD_PHI,RSHOULD_PSI,RSHOULD_GAMMA,RELBOW,LSHOULD_PHI,
		LSHOULD_PSI,LSHOULD_GAMMA,LELBOW};
	IntVector angleGroup(angleItems,angleItems+sizeof(angleItems)/sizeof(int));
	dataLog.declareGroup(JOINT_ANGLES,"Joint Angles", angleGroup);
	
	dataLog.setItemName(BASE_OMEGA_X,	"Base Omega X",			"qd(1)");
	dataLog.setItemName(BASE_OMEGA_Y,	"Base Omega Y",			"qd(2)");
	dataLog.setItemName(BASE_OMEGA_Z,	"Base Omega Z",			"qd(3)");
	dataLog.setItemName(BASE_V_X,		"Base Vel. X",			"qd(4)");
	dataLog.setItemName(BASE_V_Y,		"Base Vel. Y",			"qd(5)");
	dataLog.setItemName(BASE_V_Z,		"Base Vel. Z",			"qd(6)");
	dataLog.setItemName(RHIP_OMEGA_X,	"R Hip Omega X",		"qd(7)");
	dataLog.setItemName(RHIP_OMEGA_Y,   "R Hip Omega Y",		"qd(8)");
	dataLog.setItemName(RHIP_OMEGA_Z,	"R Hip Omega Z",		"qd(9)");
	dataLog.setItemName(RKNEE_RATE,		"R Knee Rate",			"qd(10)");
	dataLog.setItemName(RANK1_RATE,		"R Ank 1 Rate",			"qd(11)");
	dataLog.setItemName(RANK2_RATE,		"R Ank 2 Rate",			"qd(12)");
	dataLog.setItemName(LHIP_OMEGA_X,	"L Hip Omega X",		"qd(13)");
	dataLog.setItemName(LHIP_OMEGA_Y,   "L Hip Omega Y",		"qd(14)");
	dataLog.setItemName(LHIP_OMEGA_Z,	"L Hip Omega Z",		"qd(15)");
	dataLog.setItemName(LKNEE_RATE,		"L Knee Rate",			"qd(16)");
	dataLog.setItemName(LANK1_RATE,		"L Ank 1 Rate",			"qd(17)");
	dataLog.setItemName(LANK2_RATE,		"L Ank 2 Rate",			"qd(18)");
	dataLog.setItemName(RSHOULD_OMEGA_X,"R Shoulder Omega X",	"qd(19)");
	dataLog.setItemName(RSHOULD_OMEGA_Y,"R Shoulder Omega Y",	"qd(20)");
	dataLog.setItemName(RSHOULD_OMEGA_Z,"R Shoulder Omega Z",	"qd(21)");
	dataLog.setItemName(RELBOW_RATE,	"R Elbow Rate",			"qd(22)");
	dataLog.setItemName(LSHOULD_OMEGA_X,"L Shoulder Omega X",	"qd(23)");
	dataLog.setItemName(LSHOULD_OMEGA_Y,"L Shoulder Omega Y",	"qd(24)");
	dataLog.setItemName(LSHOULD_OMEGA_Z,"L Shoulder Omega Z",	"qd(25)");
	dataLog.setItemName(LELBOW_RATE,	"L Elbow Rate",			"qd(26)");
	
	int rateItems[] = {BASE_OMEGA_X,BASE_OMEGA_Y,BASE_OMEGA_Z,BASE_V_X,BASE_V_Y,BASE_V_Z,
		RHIP_OMEGA_X,RHIP_OMEGA_Y,RHIP_OMEGA_Z,RKNEE_RATE,RANK1_RATE,RANK2_RATE,
		LHIP_OMEGA_X,LHIP_OMEGA_Y,LHIP_OMEGA_Z,LKNEE_RATE,LANK1_RATE,LANK2_RATE,
		RSHOULD_OMEGA_X,RSHOULD_OMEGA_Y,RSHOULD_OMEGA_Z,RELBOW_RATE,
		LSHOULD_OMEGA_X,LSHOULD_OMEGA_Y,LSHOULD_OMEGA_Z,LELBOW_RATE};
	
	IntVector rateGroup(rateItems,rateItems+sizeof(rateItems)/sizeof(int));
	dataLog.declareGroup(JOINT_RATES,"Joint Rates", rateGroup);
	
	dataLog.setItemName(RHIP_TAU_X,		"R Hip Tau X",			"tau(1)");
	dataLog.setItemName(RHIP_TAU_Y,		"R Hip Tau Y",			"tau(2)");
	dataLog.setItemName(RHIP_TAU_Z,		"R Hip Tau Z",			"tau(3)");
	dataLog.setItemName(RKNEE_TAU,		"R Knee Tau",			"tau(4)");
	dataLog.setItemName(RANK1_TAU,		"R Ank 1 Tau",			"tau(5)");
	dataLog.setItemName(RANK2_TAU,		"R Ank 2 Tau",			"tau(6)");
	dataLog.setItemName(LHIP_TAU_X,		"L Hip Tau X",			"tau(7)");
	dataLog.setItemName(LHIP_TAU_Y,		"L Hip Tau Y",			"tau(8)");
	dataLog.setItemName(LHIP_TAU_Z,		"L Hip Tau Z",			"tau(9)");
	dataLog.setItemName(LKNEE_TAU,		"L Knee Tau",			"tau(10)");
	dataLog.setItemName(LANK1_TAU,		"L Ank 1 Tau",			"tau(11)");
	dataLog.setItemName(LANK2_TAU,		"L Ank 2 Tau",			"tau(12)");
	dataLog.setItemName(RSHOULD_TAU_X,	"R Shoulder Tau X",		"tau(13)");
	dataLog.setItemName(RSHOULD_TAU_Y,	"R Shoulder Tau Y",		"tau(14)");
	dataLog.setItemName(RSHOULD_TAU_Z,	"R Shoulder Tau Z",		"tau(15)");
	dataLog.setItemName(RELBOW_TAU,		"R Elbow Tau",			"tau(16)");
	dataLog.setItemName(LSHOULD_TAU_X,	"L Shoulder Tau X",		"tau(17)");
	dataLog.setItemName(LSHOULD_TAU_Y,	"L Shoulder Tau Y",		"tau(18)");
	dataLog.setItemName(LSHOULD_TAU_Z,	"L Shoulder Tau Z",		"tau(19)");
	dataLog.setItemName(LELBOW_TAU,		"L Elbow Tau",			"tau(20)");
	
	
	int tauItems[] = {RHIP_TAU_X,RHIP_TAU_Y,RHIP_TAU_Z,RKNEE_TAU,RANK1_TAU,RANK2_TAU,
		LHIP_TAU_X,LHIP_TAU_Y,LHIP_TAU_Z,LKNEE_TAU,LANK1_TAU,LANK2_TAU,
		RSHOULD_TAU_X,RSHOULD_TAU_Y,RSHOULD_TAU_Z,RELBOW_TAU,
		LSHOULD_TAU_X,LSHOULD_TAU_Y,LSHOULD_TAU_Z,LELBOW_TAU};
	
	IntVector tauGroup(tauItems,tauItems+sizeof(tauItems)/sizeof(int));
	dataLog.declareGroup(JOINT_TORQUES,"Joint Torques", tauGroup);
	
	
	// Left CoP Group
	dataLog.setItemName(LCOP_F_X,		"L CoP Force X",		"lCopForce(1)");
	dataLog.setItemName(LCOP_F_Y,		"L CoP Force Y",		"lCopForce(2)");
	dataLog.setItemName(LCOP_F_Z,		"L CoP Force Z",		"lCopForce(3)");
	dataLog.setItemName(LCOP_P_X,		"L CoP Pos X",			"lCopPos(1)");
	dataLog.setItemName(LCOP_P_Y,		"L CoP Pos Y",			"lCopPos(2)");
	dataLog.setItemName(LCOP_N_Z,		"L CoP Mom Z",			"lCopNz");
	
	int leftCoPItems[] = {LCOP_F_X, LCOP_F_Y, LCOP_F_Z, LCOP_P_X, LCOP_P_Y, LCOP_N_Z};
	
	IntVector lCoPGroup(leftCoPItems,leftCoPItems+sizeof(leftCoPItems)/sizeof(int));
	dataLog.declareGroup(LEFT_FOOT_WRENCH,"Left Foot Wrench", lCoPGroup);
	
	
	// Right CoP Groups
	dataLog.setItemName(RCOP_F_X,		"R CoP Force X",		"lCopForce(1)");
	dataLog.setItemName(RCOP_F_Y,		"R CoP Force Y",		"lCopForce(2)");
	dataLog.setItemName(RCOP_F_Z,		"R CoP Force Z",		"lCopForce(3)");
	dataLog.setItemName(RCOP_P_X,		"R CoP Pos X",			"lCopPos(1)");
	dataLog.setItemName(RCOP_P_Y,		"R CoP Pos Y",			"lCopPos(2)");
	dataLog.setItemName(RCOP_N_Z,		"R CoP Mom Z",			"lCopNz");
	
	int rightCoPItems[] = {RCOP_F_X, RCOP_F_Y, RCOP_F_Z, RCOP_P_X, RCOP_P_Y, RCOP_N_Z};
	
	IntVector rCoPGroup(rightCoPItems,rightCoPItems+sizeof(rightCoPItems)/sizeof(int));
	dataLog.declareGroup(RIGHT_FOOT_WRENCH,"Right Foot Wrench", rCoPGroup);
	
	// Zmp Group
	dataLog.setItemName(ZMP_F_X,		"ZMP Force X",			"zmpForce(1)");
	dataLog.setItemName(ZMP_F_Y,		"ZMP Force Y",			"zmpForce(2)");
	dataLog.setItemName(ZMP_F_Z,		"ZMP Force Z",			"zmpForce(3)");
	dataLog.setItemName(ZMP_P_X,		"ZMP Pos X",			"zmpPos(1)");
	dataLog.setItemName(ZMP_P_Y,		"ZMP Pos Y",			"zmpPos(2)");
	dataLog.setItemName(ZMP_N_Z,		"ZMP Mom Z",			"zmpNz");
	
	int zmpItems[] = {ZMP_F_X, ZMP_F_Y, ZMP_F_Z, ZMP_P_X, ZMP_P_Y, ZMP_N_Z};
	
	IntVector zmpGroup(zmpItems,zmpItems+sizeof(zmpItems)/sizeof(int));
	dataLog.declareGroup(ZMP_WRENCH,"ZMP Wrench", zmpGroup);
	
	
	// Add Dynamic Groups
	COM_POSITION = dataLog.addGroup("CoM Pos (Act)", "pCom",3);
	COM_POSITION_DES = dataLog.addGroup("Com Pos (Des)", "pComDes", 3);
	COM_VELOCITY = dataLog.addGroup("CoM Velocity", "vCom",3);
	
	CENTROIDAL_MOMENTUM = dataLog.addGroup("Centroidal Momentum",	"hCom",6);
	HDOT_DES			= dataLog.addGroup("H Dot Des",				"hDotDes", 6);
	HDOT_OPT			= dataLog.addGroup("H Dot Pot",				"hDotOpt",6);
	
	QDD_OPT				= dataLog.addGroup("Qdd Opt",				"qddOpt", 26);
	//QDD_ACT				= dataLog.addGroup("Qdd Act",				"qddAct",26);
	
	HMAT				= dataLog.addMatrixGroup("H",						"H",26,26);
	
}



void logData() {
	if (!logDataCheckBox->IsChecked()) {
		return;
	}
	
	dataMutex.Lock();
	dataLog.newRecord();
	dataLog.assignItem(TIME, sim_time);
	dataLog.assignGroup(JOINT_ANGLES, q);
	dataLog.assignGroup(JOINT_RATES, qd);
	dataLog.assignGroup(JOINT_TORQUES, tau);
	Vector6F force;
	
	force.head(3) = grfInfo.fCoPs[0];
	force.segment(3,2) = grfInfo.pCoPs[0].head(2);
	force(5)     = grfInfo.nCoPs[0];
	dataLog.assignGroup(LEFT_FOOT_WRENCH, force);
	
	force.head(3) = grfInfo.fCoPs[1];
	force.segment(3,2) = grfInfo.pCoPs[1].head(2);
	force(5)     = grfInfo.nCoPs[1];
	dataLog.assignGroup(RIGHT_FOOT_WRENCH, force);
	
	force.head(3) = grfInfo.fZMP;
	force.segment(3,2) = grfInfo.pZMP.head(2);
	force(5)     = grfInfo.nZMP;
	dataLog.assignGroup(ZMP_WRENCH, force);
	
	dataLog.assignGroup(COM_POSITION, pCom);
	dataLog.assignGroup(COM_POSITION_DES, pComDes);
	dataLog.assignGroup(COM_VELOCITY, vCom);
	
	dataLog.assignGroup(CENTROIDAL_MOMENTUM, centMom);
	dataLog.assignGroup(HDOT_DES, hDotDes);
	dataLog.assignGroup(HDOT_OPT, hDotOpt);
	
	dataLog.assignGroup(QDD_OPT, qdd);
	
	dataLog.assignMatrixGroup(HMAT,G_robot->H);
	dataMutex.Unlock();
}

void saveData()
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
	dataLog.setFile(ss.str());
	dataLog.writeRecords();
	
	FILE * curFile = fopen(curFilePath.mb_str(),"w");
	fprintf(curFile, "%s",ss.str().c_str());
	fclose(curFile);
}