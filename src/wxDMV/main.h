#ifndef _glpane_
#define _glpane_

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include "global.h"

enum
{
	BUTTON_SaveView = wxID_HIGHEST + 1, // declares an id which will be used to call button
	BUTTON_ApplyView = wxID_HIGHEST + 2
};

class BasicGLPane : public wxGLCanvas
{
    wxGLContext*	m_context;
 
public:
	BasicGLPane(wxFrame* parent, int* args, const wxSize &size);
	virtual ~BasicGLPane();
    
	void resized(wxSizeEvent& evt);
    
	int getWidth();
	int getHeight();
    
	void render(wxPaintEvent& evt);
	void prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
	void prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
    
	void extractMouseInfo(wxMouseEvent& event);
	void display (void);
	void updateSim(wxIdleEvent& event);
	// events
	void mouseMoved(wxMouseEvent& event);
	void mouseLeftDown(wxMouseEvent& event);
	void mouseLeftUp(wxMouseEvent& event);
	void mouseRightDown(wxMouseEvent& event);
	void mouseRightUp(wxMouseEvent& event);

	void mouseWheelMoved(wxMouseEvent& event);

	void mouseLeftWindow(wxMouseEvent& event);
	void mouseEnteredWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
    
	DECLARE_EVENT_TABLE()
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

#endif 
