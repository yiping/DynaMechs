/*
 *  MainFrame.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "MainFrame.h"
#include "GlobalDefines.h"


BEGIN_EVENT_TABLE(MainFrame, wxFrame)
EVT_BUTTON  (wxID_OK,   MainFrame::OnAbout)
EVT_BUTTON  (BUTTON_SaveView,   MainFrame::OnSaveView)
EVT_BUTTON  (BUTTON_ApplyView,   MainFrame::OnApplyView)
END_EVENT_TABLE()


MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
: wxFrame(NULL, -1, title, pos, size) {
    CreateStatusBar();
    SetStatusText( _("Welcome to DynaMechs wxViewr!") );
}

void MainFrame::OnSaveView(wxCommandEvent& WXUNUSED(event))
{
	float x, y, z;
	camera->getCOI(x, y, z);
	cout<<"current COI is: [ "<<x<<" "<<y<<" "<<z<<" ]"<<endl;
	float r;
	camera->getRadius(r);
	cout<<"current Radius is: " <<r<<endl;
	float elev;
	camera->getElevation(elev);
	cout<<"current elevation is: " <<elev<<endl;
	float azim;
	camera->getAzimuth(azim);
	cout<<"current azimuth is: " <<azim<<endl;	
	
	cout<<"Saving current view to file..."<<endl;
	
	ofstream Writer;
	string OutputFile = "view.txt";
	Writer.open(OutputFile.c_str(),ios::out|ios::trunc);  
    if( !Writer.is_open())
	{
		cerr<<"View file not opened! - OnSaveView Error"<<endl<<endl; 
	}
	else
	{ 
		Writer<<setw(15)<<x;
		Writer<<setw(15)<<y;
		Writer<<setw(15)<<z;
		Writer<<setw(15)<<r;
		Writer<<setw(15)<<elev;
		Writer<<setw(15)<<azim<<endl;
	}
	Writer.close();
	cout<<"Saved."<<endl<<endl;
	
}

void MainFrame::OnApplyView(wxCommandEvent& WXUNUSED(event))
{
	cout<<"read..."<<endl;
	ifstream reader;
	string inputFile = "view.txt";
	reader.open(inputFile.c_str(),ios::in);
	float x, y, z, r, elev, azim;
    if( !reader.is_open())
	{
		
		cerr<<"View file not open! - OnApplyView Error"<<endl<<endl;
	}
	else
	{
		
		while (reader>>x>>y>>z>>r>>elev>>azim)
		{
			
		}
		cout<<"View loaded from file is: [ "<<x<<" "<<y<<" "<<z<<" "<<r<<" "<<elev<<" "<<azim<<" ]"<<endl;
	}
    reader.close();
	
	cout<<"Apply view ..."<<endl;
	camera->setRadius(r);
	camera->setCOI(x, y, z);
	camera->setElevation(elev);
	camera->setAzimuth(azim);
	
	Refresh();	
}



void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event)) {
	// Note wxMessageBox might crash under Windows OS.
    wxMessageBox( _("This is a wxWidgets Hello world sample"),
				 _("About Hello World"),
				 wxOK | wxICON_INFORMATION, this );
}

