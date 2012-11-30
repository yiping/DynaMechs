// HuboController.cpp
// Nov 27, 2012
// YL

#include "control_defs.h"
#include "HuboController.h"
#include "HumanoidController.h"
#include <wx/filefn.h>
#include "globalFunctions.h"
#include "globalVariables.h"

HuboController::HuboController(dmArticulation * robot, int stateSize) : HumanoidControllerStateMachine(robot,stateSize) 
{
	
	SIM_TIME = addItem( "Sim Time",		"t");
	STATE_CODE = addItem("State",				"state");

	G_COM_POS				= addGroup("CoM Pos ", "pCom",3);
	G_COM_POS_DES 			= addGroup("CoM Pos Des", "pComDes", 3);

	G_ZMP_WRENCH			= addGroup("ZMP Wrench",		"zmpWrench",6);
	G_ZMP_WRENCH_OPT		= addGroup("ZMP Wrench Opt",		"zmpWrenchOpt",6);
	G_ZMP_POS				= addGroup("ZMP Pos",		"zmpPos",3);
	G_ZMP_POS_OPT			= addGroup("ZMP Pos Opt",		"zmpPosOpt",3);


	G_HCOM 					= addGroup( "Centroidal Momentum", "hCom", 6);
	G_HCOM_DES				= addGroup( "Centroidal Momentum Desired", "hComDes", 6);
	G_HCOMDOT_OPT			= addGroup("H Com Dot Opt",				"hComDotOpt",6);
	G_HCOMDOT_DES			= addGroup("H Com Dot Des",				"hComDotDes", 6);
}


void HuboController::logData() 
{
	
	dataMutex.Lock();
	newRecord();
	assignItem(SIM_TIME, simThread->sim_time);
	assignItem(STATE_CODE, state);

	assignGroup(G_COM_POS, pCom);
	assignGroup(G_COM_POS_DES, pComDes);


	Vector6F force = Vector6F::Zero();
	force.head(3) = grfInfo.fZMP;
	force(5)     = grfInfo.nZMP;
	assignGroup(G_ZMP_WRENCH, force);
	assignGroup(G_ZMP_WRENCH_OPT, zmpWrenchOpt);

	assignGroup(G_ZMP_POS, grfInfo.pZMP);
	assignGroup(G_ZMP_POS_OPT, zmpPosOpt);


	assignGroup(G_HCOM, centMom);
	assignGroup(G_HCOM_DES, hDes);
	assignGroup(G_HCOMDOT_OPT, hDotOpt);
	assignGroup(G_HCOMDOT_DES, hDotDes);

	dataMutex.Unlock();
}

void HuboController::saveData()
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
