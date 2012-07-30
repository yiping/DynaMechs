

// ContactDemoDataLogger.cpp
// July 12, 2012
// YL

#include "globalVariables.h"
#include "ContactDemoDataLogger.h"
#include <wx/filefn.h>
#include "globalFunctions.h"
#include "dmRigidBody.hpp"
#include "dmDynamicContactModel.hpp"
#include "dmContactModel.hpp"

ContactDemoDataLogger::ContactDemoDataLogger( ) 
{
	SIM_TIME          		= addItem( "Sim Time",		"t");
	NORMAL_PENETRATION		= addItem( "Normal Penetration ", "n_penetration");
	NORMAL_VELOCITY 		= addItem( "Normal Velocity ", "n_velocity");
	G_FE_TERRAIN		  			= addGroup("(Total) Force",	"fe_terrain", 3);
	G_FE_PLANAR_DAMPER_TERRAIN 		= addGroup("Planar Damper Force", "fe_pd_terrain", 3);
	G_EXT_F           		= addGroup("External Force",	"ext_f", 6);
	G_BOX_POS_ICS     		= addGroup("Box Pos in ICS", "box_pos", 3);
	G_BOX_VEL		   		= addGroup("Box Vel ", "box_vel", 6);
	G_SLIDING_FLAGS   		= addGroup("Sliding Flags", "sflags", 1);
	G_CONTACT_FLAGS   		= addGroup("Contact Flags", "cflags", 1);


	cout<<endl<<"SIM_TIME                  	"<<SIM_TIME<<endl
		<<"NORMAL_PENETRATION               "<<NORMAL_PENETRATION<<endl
		<<"NORMAL_VELOCITY               "<<NORMAL_VELOCITY<<endl
		<<"G_FE_TERRAIN	               		"<<G_FE_TERRAIN	<<endl
		<<"G_FE_PLANAR_DAMPER_TERRAIN	 	"<<G_FE_PLANAR_DAMPER_TERRAIN	<<endl
		<<"G_EXT_F                   "<<G_EXT_F<<endl
		<<"G_BOX_POS_ICS             "<<G_BOX_POS_ICS <<endl
		<<"G_BOX_VEL                 "<<G_BOX_VEL <<endl
		<<"G_SLIDING_FLAGS           "<<G_SLIDING_FLAGS <<endl
		<<"G_CONTACT_FLAGS           "<<G_CONTACT_FLAGS <<endl

		<<endl;
}	



void ContactDemoDataLogger::logData() 
{
	dataMutex.Lock();
/*	newRecord();
	assignItem(SIM_TIME, simThread->sim_time);

	Vector6F force;	
	Vector3F f3;
	SpatialVector f;
	dmRigidBody * rb = dynamic_cast<dmRigidBody *>(G_robot->getLink(0));

	#ifdef USE_DYNAMIC_CONTACT_MODEL
	dmDynamicContactModel * cm = dynamic_cast<dmDynamicContactModel *>(rb->getForce(0));
	#else
	dmContactModel * cm = dynamic_cast<dmContactModel *>(rb->getForce(0));
	#endif

	Float n_pen, n_vel;
	n_pen = cm->m_normal_penetration[0];
	n_vel = cm->m_normal_velocity[0];
	assignItem(NORMAL_PENETRATION, n_pen);
	assignItem(NORMAL_VELOCITY, n_vel);


	//cm->getLastComputedValue(f);
	//force = Map<Vector6F>(f);
	//assignGroup(G_CONTACT_F, force);

	CartesianVector fe;
	for (int j=0;j<3;j++)
		fe[j] = cm->m_fe_patch[0][j];
	f3 = Map<Vector3F>(fe);
	assignGroup(G_FE_TERRAIN, f3);

    //cm->getLastComputedPlanarDamperForce(f);
	//force = Map<Vector6F>(f);
	//assignGroup(G_CONTACT_F_PLANAR_DAMPER, force);

	for (int j=0;j<3;j++)
		fe[j] = cm->m_fe_patch_planar_damper[0][j];
	f3 = Map<Vector3F>(fe);
	assignGroup(G_FE_PLANAR_DAMPER_TERRAIN, f3);	


	rb->getExternalForce(f);
	force = Map<Vector6F>(f);	
	assignGroup(G_EXT_F, force);

	//RotationMatrix  R;
	//CartesianVector p;
	//G_robot->getLink(0)->getPose(R,p);
	Vector3F pos;
	pos = Map<Vector3F> ((Float*)G_robot->getForKinStruct(0)->p_ICS);	
	assignGroup(G_BOX_POS_ICS, pos);

	Vector6F vel;
	vel = Map<Vector6F> ((Float*)G_robot->getForKinStruct(0)->v); 
	assignGroup(G_BOX_VEL, vel);

	//

	VectorXF sflags(1);
	VectorXF cflags(1);
	for (int i=0; i<1; i++)
	{
		sflags(i) = cm->getSlidingState(i);
		cflags(i) = cm->getContactState(i);
	}
	assignGroup(G_SLIDING_FLAGS,sflags );
	assignGroup(G_CONTACT_FLAGS,cflags );

//	CartesianVector anc;
//	Vector3F a;
//	cm->getCurrentAnchorPoint(anc,0);
//	a = Map<Vector3F>(anc);
//	assignGroup(G_ANCHOR_0_ICS, a);

//	cm->getCurrentAnchorPoint(anc,1);
//	a = Map<Vector3F>(anc);
//	assignGroup(G_ANCHOR_1_ICS, a);

//	cm->getCurrentAnchorPoint(anc,2);
//	a = Map<Vector3F>(anc);
//	assignGroup(G_ANCHOR_2_ICS, a);

//	cm->getCurrentAnchorPoint(anc,3);
//	a = Map<Vector3F>(anc);
//	assignGroup(G_ANCHOR_3_ICS, a);

*/
	dataMutex.Unlock();
}




void ContactDemoDataLogger::saveToFile()
{
	const wxDateTime now = wxDateTime::UNow();
	
	wxString dataDirectory(dataSaveDirectory.c_str(),wxConvUTF8);
	wxString curFilePath = dataDirectory + wxT("/recentData.dat");
	
	dataDirectory += wxT("/") + now.FormatISODate()  ; // sort data by date
	wxString dataFile =  now.FormatISODate() + wxT("_") + now.FormatISOTime() + wxT(".dat");
	dataFile.Replace(wxT(":"), wxT("-"));
	
	wxString dataPath = dataDirectory + wxT("/") + dataFile;

	if(! wxDirExists(dataDirectory))
	{
		cout<<"making new dir..."<<endl;
		wxMkdir(dataDirectory);	
		cout<<wxDirExists(dataDirectory)<<endl;
	}


	stringstream ss;
	ss << dataPath.mb_str();

	setFile(ss.str());
	writeRecords();

	FILE * curFile = fopen(curFilePath.mb_str(),"w");

	if (curFile!= NULL)
	{
		fprintf(curFile, "%s",ss.str().c_str());
		fclose(curFile);
	}
	else
	{
		cout<<"cannot write to :  "<< curFilePath.mb_str() <<endl;
		exit(1);
	}		
	
}


