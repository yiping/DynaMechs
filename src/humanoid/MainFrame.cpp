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
#include "humanoidDataLogging.h"


BEGIN_EVENT_TABLE(MainFrame, wxFrame)
//EVT_BUTTON  (wxID_OK,   MainFrame::OnAbout)
EVT_BUTTON  (BUTTON_SaveView,   MainFrame::OnSaveView)
EVT_BUTTON  (BUTTON_ApplyView,   MainFrame::OnApplyView)
EVT_BUTTON  (BUTTON_SaveData,   MainFrame::OnSaveData)

EVT_MENU  (MENU_Apply_View,   MainFrame::OnApplyView)
EVT_MENU  (MENU_Save_View,   MainFrame::OnSaveView)


EVT_CLOSE   (MainFrame::OnClose)
EVT_MENU	(MENU_Pause_Sim, MainFrame::OnPauseSim)
EVT_MENU	(MENU_Log_Data, MainFrame::OnLogData)
EVT_MENU	(MENU_Save_Data,MainFrame::OnSaveData)
EVT_MENU    (MENU_Save_Directory, MainFrame::OnSaveDirectory)
EVT_MENU	(MENU_Control_Step, MainFrame::OnControlStep)
EVT_MENU	(MENU_Display_Freq, MainFrame::OnDisplayFreq)
EVT_MENU	(MENU_Integration_Step, MainFrame::OnIntegrationStep)
END_EVENT_TABLE()


MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
: wxFrame(NULL, -1, title, pos, size) {
    CreateStatusBar();
    SetStatusText( _("Welcome to DynaMechs wxViewr!") );
	
	wxMenu * editMenu = new wxMenu();
	
	editMenu->AppendCheckItem(MENU_Pause_Sim,_T("&Pause Simulation\tCtrl-P"));
	
	editMenu->Append(MENU_Control_Step, _T("&Control Step\tCtrl-C"),
					 _T("Change the simulation control step size."), true);
	editMenu->Append(MENU_Integration_Step, _T("&Integration Step\tCtrl-I"),
					 _T("Change the integration step size."), true);
	editMenu->AppendSeparator();
	
	wxMenu * graphicsMenu = new wxMenu;
	
	graphicsMenu->Append(MENU_Display_Freq, _T("&Display Frequency\tCtrl-D"),
					 _T("Change the display frequency."), true);
	graphicsMenu->Append(MENU_Save_View, _T("Save View\tCtrl-Shift-S"));
	graphicsMenu->Append(MENU_Apply_View, _T("Apply View\tCtrl-Shift-A"));
						 
	
	wxMenu * dataMenu = new wxMenu;
	
	dataMenu->AppendCheckItem(MENU_Log_Data,_T("&Log Data\tCtrl-L"));
	dataMenu->Append(MENU_Save_Data, _T("&Save Data\tCtrl-S"));						  
	dataMenu->Append(MENU_Save_Directory, _T("Data Directory"), _T("Modify the data save directory"));
	
	
    // now append the freshly created menu to the menu bar...
    menuBar = new wxMenuBar;
    menuBar->Append(editMenu, _T("&Simulation"));
	menuBar->Append(graphicsMenu, _T("&Graphics"));
    menuBar->Append(dataMenu, _T("&Data"));

    menuBar->Check(MENU_Pause_Sim, true);
	
	SetMenuBar(menuBar);
}

void MainFrame::OnClose(wxCloseEvent & event)
{
	simThread->requestStop();
	delete simThread;
	delete glPane;
	event.Skip();
}
void MainFrame::OnSaveData(wxCommandEvent & event)
{
	saveData();
}


void MainFrame::OnPauseSim(wxCommandEvent &event) {
	paused_flag = !paused_flag;
	menuBar->Check(MENU_Pause_Sim, paused_flag);
	if (!paused_flag) {
		simThread->unPause();
	}
	
}
void MainFrame::OnLogData(wxCommandEvent &event) {
	bool logState = logDataCheckBox->IsChecked();
	logState = !logState;
	cout << "Changed Log state to " << logState << endl;
	logDataCheckBox->SetValue(logState);
	menuBar->Check(MENU_Log_Data, logState);
}
void MainFrame::OnControlStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Control Step Size"),
							 wxT("Please enter the control step size"),wxString::Format(wxT("%.5lf"), cdt));
	if(dialog.ShowModal() == wxID_OK) {
		double newcdt = 0;
		if(dialog.GetValue().ToDouble(&newcdt)) {
			if(newcdt > 0) {
				cdt = newcdt;
			}
		}
	}
}
void MainFrame::OnDisplayFreq(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Display Frequency"),
							 wxT("Please enter the display freuency (Hz)"),wxString::Format(wxT("%.2lf"), render_rate));
	if(dialog.ShowModal() == wxID_OK) {
		double newfreq = 0;
		if(dialog.GetValue().ToDouble(&newfreq)) {
			if(newfreq > 0) {
				render_rate = newfreq;
				glPane->restartTimer(newfreq);
			}
		}
	}
}
void MainFrame::OnIntegrationStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Integration Step Size"),
							 wxT("Please enter the integration step size"),wxString::Format(wxT("%.5lf"), idt));
	
	if(dialog.ShowModal() == wxID_OK) {
		double newidt = 0;
		if(dialog.GetValue().ToDouble(&newidt)) {
			if(newidt > 0) {
				idt = newidt;
			}
		}
	}
}
void MainFrame::OnSaveDirectory(wxCommandEvent &event)
{
	wxString dir = wxDirSelector(wxT("Select the Data Save Directory"),wxString(dataSaveDirectory.c_str(),wxConvUTF8));
	dataSaveDirectory = dir.mb_str();
	cout << "Data Directory Changed to " << dataSaveDirectory << endl;
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

