
//  MainFrame.h
//  Nov 26, 2012
//  YL


#ifndef __MAINFRAME_H__
#define __MAINFRAME_H__

#include "wx/wx.h"
#include "wx/menu.h"
#include "BasicGLPane.h"
#include "globalDefs.h"

class MainFrame: public wxFrame
{
public:
	enum
	{
		BUTTON_SaveView = wxID_HIGHEST + 1, // declares an id which will be used to call button
		BUTTON_ApplyView,
		CHECKBOX_ShowSkeleton,
		CHECKBOX_ShowRobot,
		MENU_Control_Step,
		MENU_Display_Freq,
		MENU_Integration_Step,
		MENU_Pause_Sim,
		MENU_Apply_View,
		MENU_Save_View,

		CHECKBOX_ShowCoM,
		CHECKBOX_LogData,
		CHECKBOX_ShowZMP,
		BUTTON_SaveData,
		MENU_Log_Data,
		MENU_Save_Data,
		MENU_Save_Directory
	};
	
    MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
	
    void OnAbout(wxCommandEvent& event);
	void OnSaveView(wxCommandEvent& event);
	void OnApplyView(wxCommandEvent& event);
	void OnClose(wxCloseEvent &event);
	void OnQuit(wxCommandEvent &event);
	
	void OnPauseSim(wxCommandEvent &event);
	void OnControlStep(wxCommandEvent &event);
	void OnDisplayFreq(wxCommandEvent &event);
	void OnIntegrationStep(wxCommandEvent &event);

	void OnSaveData(wxCommandEvent& event);
	void OnSaveDirectory(wxCommandEvent &event);
	void OnLogData(wxCommandEvent &event);

	// if you're creating a derived class that contains child windows,
	// you should use a pointer to the child windows INSTEAD OF the objects themselves
	// as members of the main window.
	// otherwise it would cause double-deletion problems
	// http://wiki.wxwidgets.org/Avoiding_Memory_Leaks
	
	wxCheckBox *showSkeleton, *showRobot;  

	wxCheckBox *showCoM, *logDataCheckBox, *showZMP;
	wxButton *saveDataButton;
	wxStaticText * realTimeRatioDisplay;

	BasicGLPane * glPane;
	
private:
	wxMenuBar * menuBar;
	wxPanel *toolpanel;
	wxButton *saveViewbutton;
	wxButton *applyViewbutton;

	
    DECLARE_EVENT_TABLE()

};

#endif
