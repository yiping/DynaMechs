

// OTDataLogger.cpp
// Sep 3, 2012
// Project: Optimization Test
// YL

#include "globalVariables.h"
#include "OTDataLogger.h"
#include <wx/filefn.h>
#include "globalFunctions.h"
#include "dmRigidBody.hpp"

#include "dmContactModel.hpp"

OTDataLogger::OTDataLogger( ) 
{
	SIM_TIME = addItem( "Sim Time",		"t");



	cout<<endl<<"SIM_TIME                  	"<<SIM_TIME<<endl


		<<endl;
}	



void OTDataLogger::logData() 
{
	dataMutex.Lock();
	newRecord();
	assignItem(SIM_TIME, simThread->sim_time);


	dataMutex.Unlock();
}




void OTDataLogger::saveToFile()
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


