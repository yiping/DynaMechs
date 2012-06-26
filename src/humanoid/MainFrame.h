/*
 *  MainFrame.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "wx/wx.h"

enum
{
	BUTTON_SaveView = wxID_HIGHEST + 1, // declares an id which will be used to call button
	BUTTON_ApplyView,
	CHECKBOX_ShowCoM,
	CHECKBOX_ShowGRF,
	CHECKBOX_ShowNetForceAtGround,
	CHECKBOX_ShowNetForceAtCoM
};

class MainFrame: public wxFrame
{
public:
	
    MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
	
    void OnAbout(wxCommandEvent& event);
	void OnSaveView(wxCommandEvent& event);
	void OnApplyView(wxCommandEvent& event);
	
    DECLARE_EVENT_TABLE()
};