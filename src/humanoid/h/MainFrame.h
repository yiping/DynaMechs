/*
 *  MainFrame.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __MAINFRAME_H__
#define __MAINFRAME_H__

#include "wx/wx.h"
#include "wx/menu.h"
#include "BasicGLPane.h"


class MainFrame: public wxFrame
{
public:
	enum
	{
		BUTTON_SaveView = wxID_HIGHEST + 1, // declares an id which will be used to call button
		BUTTON_ApplyView,
		CHECKBOX_ShowCoM,
		CHECKBOX_ShowGRF,
		CHECKBOX_ShowNetForceAtGround,
		CHECKBOX_ShowNetForceAtCoM,
		CHECKBOX_LogData,
		BUTTON_SaveData,
		MENU_Control_Step,
		MENU_Display_Freq,
		MENU_Integration_Step,
		MENU_Pause_Sim,
		MENU_Log_Data,
		MENU_Save_Data,
		MENU_Save_Directory,
		MENU_Apply_View,
		MENU_Save_View
	};
	
    MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
	
    void OnAbout(wxCommandEvent& event);
	void OnSaveView(wxCommandEvent& event);
	void OnApplyView(wxCommandEvent& event);
	void OnSaveData(wxCommandEvent& event);
	void OnClose(wxCloseEvent &event);
	
	void OnPauseSim(wxCommandEvent &event);
	void OnLogData(wxCommandEvent &event);
	void OnControlStep(wxCommandEvent &event);
	void OnDisplayFreq(wxCommandEvent &event);
	void OnIntegrationStep(wxCommandEvent &event);
	void OnSaveDirectory(wxCommandEvent &event);
	
	wxCheckBox * showCoM, * showGRF, * showNetForceAtGround, * showNetForceAtCoM, *logDataCheckBox;
	wxStaticText * realTimeRatioDisplay;
	BasicGLPane * glPane;
	
private:
	wxMenuBar * menuBar;
	wxPanel *toolpanel;
	wxButton *saveViewbutton;
	wxButton *applyViewbutton;
	wxButton *saveDataButton;
	
    DECLARE_EVENT_TABLE()

};

#endif