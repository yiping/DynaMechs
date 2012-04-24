/*
 *****************************************************************************
 *     File: functions_C.cpp
 *   Author: Yiping Liu
 *  Created: 18 Apr 2012
 *  Summary: Utility functions   
 *            
 *****************************************************************************/

#include "global.h"   
#include <dm.h>

void SaveToDataRecord(DataRecord *Rec)
{
	Rec->sim_time = sim_time;
	Float q[1],qd[1];

	for (int i=0; i<G_robot->getNumLinks();i++)
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

	for (int j=0; j<6;j++)
	{
		Rec->desired_torso_acc[j] = desired_torso_acc(j);
	}

	Rec->ZMPx_ICS = p_ZMP_ICS(0);
	Rec->actual_ZMPx_ICS = actual_p_ZMP_ICS(0);

	for (int j=0; j<3;j++)
	{
		Rec->p_Rf_ICS[j] = p_rf_ICS[j];
		Rec->p_Lf_ICS[j] = p_lf_ICS[j];
	}

}



void simDataOutput(const DataRecVector & MyVec)
{
	cout<<"outputting simulation data..."<<endl;
   	string OutputFileString = "biped_sim_data.txt";

	
	ofstream Writer;
	string OutputFile = "biped_sim_data.txt";
	Writer.open(OutputFile.c_str(),ios::out|ios::trunc);  
    	if( !Writer.is_open())
	{
		cerr<<"Data File not open! - Error"<<endl<<endl; 
	}
	else
	{ 
		for( int u = 0; u<MyVec.size();u++)
		{
			Writer<<setw(15)<<MyVec[u]->sim_time;
			Writer<<setw(15)<<MyVec[u]->q[0]; 
			Writer<<setw(15)<<MyVec[u]->q[1];                     
			Writer<<setw(15)<<MyVec[u]->q[2];      
			Writer<<setw(15)<<MyVec[u]->q[5];     
			Writer<<setw(15)<<MyVec[u]->q[6];      
			Writer<<setw(15)<<MyVec[u]->q[7];
			Writer<<setw(15)<<MyVec[u]->q[8];
			Writer<<setw(15)<<MyVec[u]->q[9];

			/*Writer<<setw(15)<<MyVec[u]->qd[0]; 
			Writer<<setw(15)<<MyVec[u]->qd[1];                     
			Writer<<setw(15)<<MyVec[u]->qd[2];      
			Writer<<setw(15)<<MyVec[u]->qd[5];     
			Writer<<setw(15)<<MyVec[u]->qd[6];      
			Writer<<setw(15)<<MyVec[u]->qd[7];
			Writer<<setw(15)<<MyVec[u]->qd[8];
			Writer<<setw(15)<<MyVec[u]->qd[9];*/

			Writer<<setw(15)<<MyVec[u]->JointTorque[5];     
			Writer<<setw(15)<<MyVec[u]->JointTorque[6];      
			Writer<<setw(15)<<MyVec[u]->JointTorque[7];
			Writer<<setw(15)<<MyVec[u]->JointTorque[8];
			Writer<<setw(15)<<MyVec[u]->JointTorque[9]<<endl; 
		}
		Writer.close();
		cout<<"done."<<endl<<endl;
	}

}
