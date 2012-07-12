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

#include <dm.h>

#include <wxDMGLMouse.hpp>
#include <wxDMGLPolarCamera_zup.hpp>
#include <dmTime.h>
 
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
    
	void extractMouseInfo(wxMouseEvent& event);
	//void display (void);
	void updateSim(wxTimerEvent& event);
	void userGraphics();
	void userGraphics2DPrimary();
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
	
	void restartTimer();
	void stopTimer();


    GLUquadricObj *quadratic;
	wxDMGLMouse *mouse;
	wxDMGLPolarCamera_zup *camera;
	
	volatile bool model_loaded;
	bool IsWireframe;
	double render_rate;
	
	Float real_time_ratio;
	
	DECLARE_EVENT_TABLE()
private:
	wxTimer * timer;
	int timer_count;
	volatile Float last_render_time, rtime;
	dmTimespec last_draw_tv, first_tv , update_tv;
	
	GLdouble modelview[16], projection[16];
	GLint vp[4];
};




#endif 
