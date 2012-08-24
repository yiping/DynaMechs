/*
 *  HumanoidMainFrame.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"
#include "HumanoidMainFrame.h"
#include "HumanoidDataLogger.h"
#include "wxBoxSlider.h"


BEGIN_EVENT_TABLE(HumanoidMainFrame, wxFrame)
//EVT_BUTTON  (wxID_OK,   HumanoidMainFrame::OnAbout)
EVT_BUTTON  (BUTTON_SaveView,   HumanoidMainFrame::OnSaveView)
EVT_BUTTON  (BUTTON_ApplyView,   HumanoidMainFrame::OnApplyView)
EVT_BUTTON  (BUTTON_SaveData,   HumanoidMainFrame::OnSaveData)

EVT_MENU  (MENU_Apply_View,   HumanoidMainFrame::OnApplyView)
EVT_MENU  (MENU_Save_View,   HumanoidMainFrame::OnSaveView)


EVT_CLOSE   (HumanoidMainFrame::OnClose)
EVT_MENU	(MENU_Pause_Sim, HumanoidMainFrame::OnPauseSim)
EVT_MENU	(MENU_Log_Data, HumanoidMainFrame::OnLogData)
EVT_MENU	(MENU_Save_Data,HumanoidMainFrame::OnSaveData)
EVT_MENU    (MENU_Save_Directory, HumanoidMainFrame::OnSaveDirectory)
EVT_MENU	(MENU_Control_Step, HumanoidMainFrame::OnControlStep)
EVT_MENU	(MENU_Display_Freq, HumanoidMainFrame::OnDisplayFreq)
EVT_MENU	(MENU_Integration_Step, HumanoidMainFrame::OnIntegrationStep)
EVT_MENU	(MENU_Slow_Motion, HumanoidMainFrame::OnSlowMotion)

EVT_MENU    (wxID_EXIT, HumanoidMainFrame::OnQuit)
END_EVENT_TABLE()


HumanoidMainFrame::HumanoidMainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
: wxFrame(NULL, -1, title, pos, size) {
    CreateStatusBar();
    SetStatusText( _("Welcome to DynaMechs wxViewr!") );
	
	// Create menus
	{
		wxMenu * fileMenu = new wxMenu();
		
		fileMenu->Append(wxID_EXIT, _T("Quit\tCtrl-Q"));
		
		wxMenu * editMenu = new wxMenu();
		
		editMenu->AppendCheckItem(MENU_Pause_Sim,_T("&Pause Simulation\tCtrl-P"));
		
		editMenu->Append(MENU_Control_Step, _T("&Control Step\tCtrl-C"),
						 _T("Change the simulation control step size."));
		editMenu->Append(MENU_Integration_Step, _T("&Integration Step\tCtrl-I"),
						 _T("Change the integration step size."));
		editMenu->AppendSeparator();
		
		wxMenu * graphicsMenu = new wxMenu;
		
		graphicsMenu->Append(MENU_Display_Freq, _T("&Display Frequency\tCtrl-D"),
						 _T("Change the display frequency."));
		graphicsMenu->Append(MENU_Save_View, _T("Save View\tCtrl-Shift-S"));
		graphicsMenu->Append(MENU_Apply_View, _T("Apply View\tCtrl-Shift-A"));
		
		graphicsMenu->AppendCheckItem(MENU_Slow_Motion, _T("Slow Motion\tCtrl-Shift-M"));
		
		
		wxMenu * dataMenu = new wxMenu;
		
		dataMenu->AppendCheckItem(MENU_Log_Data,_T("&Log Data\tCtrl-L"));
		dataMenu->Append(MENU_Save_Data, _T("&Save Data\tCtrl-S"));						  
		dataMenu->Append(MENU_Save_Directory, _T("Data Directory"), _T("Modify the data save directory"));
		
		
		// now append the freshly created menu to the menu bar...
		menuBar = new wxMenuBar;

#ifndef __WXMAC__
		menuBar->Append(fileMenu,_T("&File"));
#endif
		
		menuBar->Append(editMenu, _T("&Simulation"));
		menuBar->Append(graphicsMenu, _T("&Graphics"));
		menuBar->Append(dataMenu, _T("&Data"));

		menuBar->Check(MENU_Pause_Sim, true);
		menuBar->Check(MENU_Slow_Motion,false);
		
		SetMenuBar(menuBar);
		cout << "Created Menus" << endl;
	}
	
	
	// Create panels
	{
		toolpanel = new wxPanel((wxFrame*) this, -1, wxPoint(-1,-1), wxSize(200,400));
		
		wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
		wxBoxSizer *toolpanel_sizer = new wxBoxSizer( wxVERTICAL);
		
		int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, WX_GL_SAMPLE_BUFFERS, GL_TRUE, WX_GL_SAMPLES, 16,0};
		//int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
		
		//cout << "Pane " << endl;
		glPane = new BasicGLPane( (wxFrame*) this, args, wxSize(400,400));
		
		saveViewbutton = new wxButton( toolpanel, HumanoidMainFrame::BUTTON_SaveView, wxT("Save View"));
		applyViewbutton = new wxButton( toolpanel, HumanoidMainFrame::BUTTON_ApplyView, wxT("Apply View"));
		showCoM = new wxCheckBox(toolpanel,HumanoidMainFrame::CHECKBOX_ShowCoM,wxT("Show CoM"));
		showGRF = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowGRF,wxT("Show GRF"));
		showNetForceAtGround = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowNetForceAtGround,wxT("Show Net Force (Ground)"));
		showNetForceAtCoM	 = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowNetForceAtCoM,wxT("Show Net Force (CoM)"));
		showTraces           = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowTraces,wxT("Show Traces"));
		showSkeleton           = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowSkeleton,wxT("Show Skeleton"));
		
		slowMotion           = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_ShowTraces,wxT("Slow Motion"));
		
		slowMoRatio = new wxBoxSlider(toolpanel,-1,.3,4,100);
		slowMoRatio->setValue(1);
		
		// Camera Options
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Camera Options")),0,wxALL,2);
		toolpanel_sizer->Add(saveViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		toolpanel_sizer->Add(applyViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		
		// View Options
		toolpanel_sizer->AddSpacer(15);
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("View Options")),0,wxALL,2);
		toolpanel_sizer->Add(showCoM, 0 ,wxALL  ,2);
		toolpanel_sizer->Add(showGRF, 0 ,wxALL ,2);
		toolpanel_sizer->Add(showNetForceAtGround, 0,wxALL,2 );
		toolpanel_sizer->Add(showNetForceAtCoM, 0,wxALL,2 );
		toolpanel_sizer->Add(showTraces,0,wxALL,2);
		toolpanel_sizer->Add(showSkeleton,0,wxALL,2);
		
		toolpanel_sizer->Add(slowMotion,0,wxALL,2);
		toolpanel_sizer->Add(slowMoRatio,0,wxALL,2);
		
		
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Control Options")),0,wxALL,2);	
		
		wxBoxSlider * CoMControlSlider = new wxBoxSlider(toolpanel,-1,5,15,100);
		toolpanel_sizer->Add(CoMControlSlider,0,wxALL,2);
		
		// Data Logging
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Data Logging")),0,wxALL,2);
		
		logDataCheckBox = new wxCheckBox(toolpanel ,HumanoidMainFrame::CHECKBOX_LogData,wxT("Log Data"));
		saveDataButton = new wxButton(toolpanel ,HumanoidMainFrame::BUTTON_SaveData,wxT("Save Data"));
		toolpanel_sizer->Add(logDataCheckBox,0,wxALL,2);
		toolpanel_sizer->Add(saveDataButton,0,wxALL,2);
		
		
		// Indicators
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Indicators")),0,wxALL,2);
		
		realTimeRatioDisplay = new wxStaticText(toolpanel,-1,wxT("RT Ratio: "));
		toolpanel_sizer->Add(realTimeRatioDisplay,0,wxALL,2);
		
		showCoM->SetValue(true);
		showGRF->SetValue(true);
		
		toolpanel->SetSizer(toolpanel_sizer);
		
		sizer->Add(glPane, 1, wxEXPAND );
		sizer->Add(toolpanel, 0, wxEXPAND);
		
		sizer->SetSizeHints(this);
		SetSizer(sizer);
		
		SetAutoLayout(true);
		SendSizeEvent();
		cout << "Created Panel" << endl;
	}
}

void HumanoidMainFrame::OnSlowMotion(wxCommandEvent & event)
{
	bool state = slowMotion->IsChecked();
	state = !state;
	cout << "Changed Log state to " << state << endl;
	slowMotion->SetValue(state);
	menuBar->Check(MENU_Slow_Motion, state);
}

void HumanoidMainFrame::OnClose(wxCloseEvent & event)
{
	cout << "Requesting Sim Stop" << endl;
	simThread->requestStop();
	delete simThread;
	delete glPane;
	event.Skip();
}

void HumanoidMainFrame::OnQuit(wxCommandEvent & event)
{
	Close(true);
}

void HumanoidMainFrame::OnSaveData(wxCommandEvent & event)
{
	humanoid->saveData();
}


void HumanoidMainFrame::OnPauseSim(wxCommandEvent &event) {
	simThread->paused_flag = !simThread->paused_flag;
	menuBar->Check(MENU_Pause_Sim, simThread->paused_flag);
	if (!simThread->paused_flag) {
		simThread->unPause();
	}
	
}
void HumanoidMainFrame::OnLogData(wxCommandEvent &event) {
	bool logState = logDataCheckBox->IsChecked();
	logState = !logState;
	cout << "Changed Log state to " << logState << endl;
	logDataCheckBox->SetValue(logState);
	menuBar->Check(MENU_Log_Data, logState);
}
void HumanoidMainFrame::OnControlStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Control Step Size"),
							 wxT("Please enter the control step size"),wxString::Format(wxT("%.5lf"), simThread->cdt));
	if(dialog.ShowModal() == wxID_OK) {
		double newcdt = 0;
		if(dialog.GetValue().ToDouble(&newcdt)) {
			if(newcdt > 0) {
				simThread->cdt = newcdt;
			}
		}
	}
}
void HumanoidMainFrame::OnDisplayFreq(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Display Frequency"),
							 wxT("Please enter the display freuency (Hz)"),wxString::Format(wxT("%.2lf"), glPane->render_rate));
	if(dialog.ShowModal() == wxID_OK) {
		double newfreq = 0;
		if(dialog.GetValue().ToDouble(&newfreq)) {
			if(newfreq > 0) {
				glPane->render_rate = newfreq;
				glPane->restartTimer();
			}
		}
	}
}
void HumanoidMainFrame::OnIntegrationStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Integration Step Size"),
							 wxT("Please enter the integration step size"),wxString::Format(wxT("%.5lf"), simThread->idt));
	
	if(dialog.ShowModal() == wxID_OK) {
		double newidt = 0;
		if(dialog.GetValue().ToDouble(&newidt)) {
			if(newidt > 0) {
				simThread->idt = newidt;
			}
		}
	}
}
void HumanoidMainFrame::OnSaveDirectory(wxCommandEvent &event)
{
	wxString dir = wxDirSelector(wxT("Select the Data Save Directory"),wxString(humanoid->dataSaveDirectory.c_str(),wxConvUTF8));
	dir += wxT("/");
	humanoid->dataSaveDirectory = dir.mb_str();
	cout << "Data Directory Changed to " << humanoid->dataSaveDirectory << endl;
}


void HumanoidMainFrame::OnSaveView(wxCommandEvent& WXUNUSED(event))
{
	float x, y, z;
	glPane->camera->getCOI(x, y, z);
	cout<<"current COI is: [ "<<x<<" "<<y<<" "<<z<<" ]"<<endl;
	float r;
	glPane->camera->getRadius(r);
	cout<<"current Radius is: " <<r<<endl;
	float elev;
	glPane->camera->getElevation(elev);
	cout<<"current elevation is: " <<elev<<endl;
	float azim;
	glPane->camera->getAzimuth(azim);
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

void HumanoidMainFrame::OnApplyView(wxCommandEvent& WXUNUSED(event))
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
	glPane->camera->setRadius(r);
	glPane->camera->setCOI(x, y, z);
	glPane->camera->setElevation(elev);
	glPane->camera->setAzimuth(azim);
	
	Refresh();	
}



void HumanoidMainFrame::OnAbout(wxCommandEvent& WXUNUSED(event)) {
	// Note wxMessageBox might crash under Windows OS.
    wxMessageBox( _("This is a wxWidgets Hello world sample"),
				 _("About Hello World"),
				 wxOK | wxICON_INFORMATION, this );
}

