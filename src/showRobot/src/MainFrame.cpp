
//  MainFrame.cpp
//  Nov 12, 2012
//	Project: showRobot
//  YL

#include "globalVariables.h"
#include "MainFrame.h"


BEGIN_EVENT_TABLE(MainFrame, wxFrame)
//EVT_BUTTON  (wxID_OK,   MainFrame::OnAbout)
EVT_BUTTON  (BUTTON_SaveView,   MainFrame::OnSaveView)
EVT_BUTTON  (BUTTON_ApplyView,   MainFrame::OnApplyView)

EVT_MENU  (MENU_Apply_View,   MainFrame::OnApplyView)
EVT_MENU  (MENU_Save_View,   MainFrame::OnSaveView)


EVT_CLOSE   (MainFrame::OnClose)
EVT_MENU    (wxID_EXIT, MainFrame::OnQuit)	// menu: file->quit
EVT_MENU	(MENU_Pause_Sim, MainFrame::OnPauseSim)
EVT_MENU	(MENU_Control_Step, MainFrame::OnControlStep)
EVT_MENU	(MENU_Display_Freq, MainFrame::OnDisplayFreq)
EVT_MENU	(MENU_Integration_Step, MainFrame::OnIntegrationStep)
END_EVENT_TABLE()


MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
: wxFrame(NULL, -1, title, pos, size) {
    CreateStatusBar(4);
    SetStatusText( _("DynaMechs 5.0"),0 );

	
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
							 
		
	
		
		// now append the freshly created menu to the menu bar...
		menuBar = new wxMenuBar;
		menuBar->Append(fileMenu,_T("&File"));
		menuBar->Append(editMenu, _T("&Simulation"));
		menuBar->Append(graphicsMenu, _T("&Graphics"));


		menuBar->Check(MENU_Pause_Sim, true);
		
		SetMenuBar(menuBar);
		// If the frame is destroyed, the menubar and its menus will be destroyed also
		// therefore DO NOT delete the menubar explicitly
	}
	
	
	// Create panels
	{
		toolpanel = new wxPanel((wxFrame*) this, -1, wxPoint(-1,-1), wxSize(200,400));
		
		wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
		wxBoxSizer *toolpanel_sizer = new wxBoxSizer( wxVERTICAL);
		
		int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
		
		//cout << "Pane " << endl;
		glPane = new BasicGLPane( (wxFrame*) this, args, wxSize(400,400));
		
		saveViewbutton = new wxButton( toolpanel, MainFrame::BUTTON_SaveView, wxT("Save View"));
		applyViewbutton = new wxButton( toolpanel, MainFrame::BUTTON_ApplyView, wxT("Apply View"));

		showSkeleton           = new wxCheckBox(toolpanel ,MainFrame::CHECKBOX_ShowSkeleton,wxT("Show Skeleton"));
		showRobot           = new wxCheckBox(toolpanel ,MainFrame::CHECKBOX_ShowRobot,wxT("Show Robot"));

		
		// Camera Options
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Camera Options")),0,wxALL,2);
		toolpanel_sizer->Add(saveViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		toolpanel_sizer->Add(applyViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		
		// View Options
		toolpanel_sizer->AddSpacer(15);
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("View Options")),0,wxALL,2);

		toolpanel_sizer->Add(showSkeleton, 0 ,wxALL  ,2);
		toolpanel_sizer->Add(showRobot, 0 ,wxALL  ,2);

		
		toolpanel_sizer->AddSpacer(15);	
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Control Options")),0,wxALL,2);	
#ifdef SYNC_GRAPHICS			
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Sync Graphics")), 0, wxALL, 2);
#else 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Dynamics AFAP")), 0, wxALL, 2);
#endif
	
		showRobot->SetValue(true);
		showSkeleton->SetValue(false);

		
		toolpanel->SetSizer(toolpanel_sizer);
		
		sizer->Add(glPane, 1, wxEXPAND );
		sizer->Add(toolpanel, 0, wxEXPAND);
		
		sizer->SetSizeHints(this);
		SetSizer(sizer);
		
		SetAutoLayout(true);
	}
}

// When the user clicks on the system close button (in a frame or a dialog),
// wxWidgets calls wxWindow::Close(), This in turn generates an EVT_CLOSE event.
void MainFrame::OnClose(wxCloseEvent & event)
{
	simThread->requestStop();
	delete simThread;

	// The wxCloseEvent handler should only call wxWindow::Destroy()
	// or wxWindow::Close() to delete the window, and NOT use the delete operator
	//delete glPane;

	// Child windows are automatically deleted from within the parent destructor. 
	// This includes any children that are themselves frames or dialogs,
	// so you may wish to close these child frame or dialog windows explicitly from within the parent close handler.
	// http://docs.wxwidgets.org/trunk/overview_windowdeletion.html
	glPane->Close(true);	//true: cannot be vetoed.
	event.Skip();
}

// This is for closing from the menu.
void MainFrame::OnQuit(wxCommandEvent & event)
{
	// A wxWidgets application automatically exits 
	// when the last TopLevelWindow (wxFrame or wxDialog) is destroyed

	Close(true);	// wxWindow::Close generates a wxCloseEvent whose handler tries to do clean-ups.
}



void MainFrame::OnPauseSim(wxCommandEvent &event) 
{
	simThread->paused_flag = !simThread->paused_flag;
	menuBar->Check(MENU_Pause_Sim, simThread->paused_flag);
	if (!simThread->paused_flag) 
	{
		simThread->unPause();
	}
	
}


void MainFrame::OnControlStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Control Step Size"),
							 wxT("Please enter the control step size"),wxString::Format(wxT("%.5lf"), simThread->cdt));
	if(dialog.ShowModal() == wxID_OK) {
		double newcdt = 0;
		if(dialog.GetValue().ToDouble(&newcdt)) {
			if(newcdt > 0) {
				simThread->cdt = newcdt;
				wxString s;
    			s.Printf(wxT("cdt = %f s"), simThread->cdt);
				frame->SetStatusText(s,2);
			}
		}
	}
}

void MainFrame::OnDisplayFreq(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Display Frequency"),
							 wxT("Please enter the display freuency (Hz)"),wxString::Format(wxT("%.2lf"), glPane->render_rate));
	if(dialog.ShowModal() == wxID_OK) 
	{
		double newfreq = 0;
		if(dialog.GetValue().ToDouble(&newfreq)) 
		{
			if(newfreq > 0) 
			{
				glPane->render_rate = newfreq;
				glPane->restartTimer();
				wxString s;
    			s.Printf(wxT("refresh every %.2f ms"), 1000/glPane->render_rate);
				frame->SetStatusText(s,3);
			}
		}
	}
}

void MainFrame::OnIntegrationStep(wxCommandEvent &event)
{
	wxTextEntryDialog dialog(this,wxT("Integration Step Size"),
							 wxT("Please enter the integration step size"),wxString::Format(wxT("%.5lf"), simThread->idt));
	
	if(dialog.ShowModal() == wxID_OK) 
	{
		double newidt = 0;
		if(dialog.GetValue().ToDouble(&newidt)) 
		{
			if(newidt > 0) 
			{
				simThread->idt = newidt;
				wxString s;
    			s.Printf(wxT("idt = %1.2e s"), simThread->idt);
				frame->SetStatusText(s,1);
			}
		}
	}
}



void MainFrame::OnSaveView(wxCommandEvent& WXUNUSED(event))
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
	glPane->camera->setRadius(r);
	glPane->camera->setCOI(x, y, z);
	glPane->camera->setElevation(elev);
	glPane->camera->setAzimuth(azim);
	
	Refresh();	
}



void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event)) 
{
	// Note wxMessageBox might crash under Windows OS.
    wxMessageBox( _("This simulator program is built upon DynaMechs 5.0, a rigid body dynamic engine developed at Ohio State"),
				 _("About DynaMechs Simulator"),
				 wxOK | wxICON_INFORMATION, this );
}

