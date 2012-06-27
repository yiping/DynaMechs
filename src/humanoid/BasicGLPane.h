#ifndef _glpane_
#define _glpane_

#include <wx/wx.h>
#include "wx/timer.h"
#include <wx/glcanvas.h>

#ifdef __WXMAC__
#include "OpenGL/glu.h"
#include "OpenGL/gl.h"
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif



 
class BasicGLPane : public wxGLCanvas
{
public:
	
	enum GLPaneIDS {
		TIMER_ID = wxID_HIGHEST+1
	};
	
	BasicGLPane(wxFrame* parent, int* args, const wxSize &size);
	~BasicGLPane();
	
	virtual void OnExit();
    
	void resized(wxSizeEvent& evt);
    
	int getWidth();
	int getHeight();
    
	void render(wxPaintEvent& evt);
	void prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
	void prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
    
	void extractMouseInfo(wxMouseEvent& event);
	//void display (void);
	void updateSim(wxTimerEvent& event);
	void userGraphics();
	void glInit();
	
	// events
	void mouseMoved(wxMouseEvent& event);
	void mouseLeftDown(wxMouseEvent& event);
	void mouseLeftUp(wxMouseEvent& event);
	void mouseRightDown(wxMouseEvent& event);
	void mouseRightUp(wxMouseEvent& event);

	void mouseMiddleDown(wxMouseEvent& event);
	void mouseMiddleUp(wxMouseEvent& event);
	
	void mouseWheelMoved(wxMouseEvent& event);

	void mouseLeftWindow(wxMouseEvent& event);
	void mouseEnteredWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
	
	void restartTimer(double freq);
	void stopTimer();
    GLUquadricObj *quadratic;
	
	DECLARE_EVENT_TABLE()
	
	
private:
	wxTimer * timer;
	
};




#endif 
