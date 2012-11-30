
//  BasicGLPane.cpp
//  Nov 26, 2012
//  YL


#include <dm.h>            
#include "globalVariables.h"
#include "globalFunctions.h"
#include "globalDefs.h"
#include <dmTime.h>
#include "BasicGLPane.h"
#include <dmEnvironment.hpp>
#include <wx/defs.h>

BEGIN_EVENT_TABLE(BasicGLPane, wxGLCanvas)
EVT_MOTION(BasicGLPane::mouseMoved)
EVT_LEFT_DOWN(BasicGLPane::mouseLeftDown)
EVT_MIDDLE_DOWN(BasicGLPane::mouseMiddleDown)
EVT_MIDDLE_UP(BasicGLPane::mouseMiddleUp)

EVT_LEFT_UP(BasicGLPane::mouseLeftUp)
EVT_RIGHT_DOWN(BasicGLPane::mouseRightDown)
EVT_RIGHT_UP(BasicGLPane::mouseRightUp)
EVT_LEAVE_WINDOW(BasicGLPane::mouseLeftWindow)
EVT_ENTER_WINDOW(BasicGLPane::mouseEnteredWindow)
EVT_SIZE(BasicGLPane::resized)
//EVT_KEY_DOWN(BasicGLPane::keyPressed)
EVT_KEY_UP(BasicGLPane::keyReleased)
EVT_MOUSEWHEEL(BasicGLPane::mouseWheelMoved)
EVT_PAINT(BasicGLPane::render)
//EVT_IDLE(BasicGLPane::updateSim)
EVT_TIMER(BasicGLPane::TIMER_ID,BasicGLPane::updateSim)

END_EVENT_TABLE()

BasicGLPane::BasicGLPane(wxFrame* parent, int* args, const wxSize &size) :
wxGLCanvas(parent, wxID_ANY, wxDefaultPosition, size, wxFULL_REPAINT_ON_RESIZE,wxT("GLWindow"),args)
{
	mouse = new wxDMGLMouse(); // has to be put in the front
	camera = new wxDMGLPolarCamera_zup();
	
	// glViewport (0, 0, 0.8*size.GetWidth(), size.GetHeight());  //glViewport is meaningless here
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;
	
	
    // To avoid flashing on MSW
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);
	
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str()};
	glutInit(&argc, argv);
	
	timer = new wxTimer(this, TIMER_ID);
	
	dmGetSysTime(&last_draw_tv);
	dmGetSysTime(&update_tv);
	dmGetSysTime(&first_tv);
	model_loaded = false;
	timer_count = 0;
	cnt = 0;

	//SetCurrent();		// note: one cannot call SetCurrent() if the window is not shown
}

BasicGLPane::~BasicGLPane()
{
	cout<<" -- BasicGLPane destructor -- "<<endl;
	timer->Stop();
	delete timer;
}
void BasicGLPane::restartTimer() {
	if (timer->IsRunning()) {
		timer->Stop();
	}
	timer->Start(1000./render_rate);// called repeatedly
}
void BasicGLPane::stopTimer() {
	timer->Stop();
}

// some useful events to use
void BasicGLPane::extractMouseInfo(wxMouseEvent& event) {
	wxPoint pos = event.GetPosition();
	mouse->xwin = pos.x;
	mouse->ywin = pos.y;
	mouse->xchan = (2.0*((GLfloat) mouse->xwin) - mouse->win_size_x)/
	(GLfloat) mouse->win_size_x;
	mouse->ychan = (2.0*((GLfloat) mouse->ywin) - mouse->win_size_y)/
	(GLfloat) mouse->win_size_y;
	
	//cout << "Pos: " << mouse->xwin << ", " << mouse->ywin << "  Button state: "<< mouse->button_flags << endl;
}
void BasicGLPane::mouseMoved(wxMouseEvent& event) {
	//cout << "mouse moving" << endl;
	//cout << "mouse moving          "<< mouse->button_flags  << endl;
	extractMouseInfo(event);

	Refresh();
}
void BasicGLPane::mouseLeftDown(wxMouseEvent& event) {
	mouse->button_flags |= MOUSE_L_DN;
	//cout << "left button pressed" << endl;
	
	extractMouseInfo(event);
	SetFocus();
}
void BasicGLPane::mouseLeftUp(wxMouseEvent& event)  {
	mouse->button_flags &= ~MOUSE_L_DN;
	//cout << "left button released" << endl;
	//cout << "left button released  "<< mouse->button_flags << endl;
	extractMouseInfo(event);

	Refresh();
}

void BasicGLPane::mouseMiddleDown(wxMouseEvent& event) {
	mouse->button_flags |= MOUSE_M_DN;
	//cout << "mid button pressed" << endl;
	
	extractMouseInfo(event);
	SetFocus();
}
void BasicGLPane::mouseMiddleUp(wxMouseEvent& event)  {
	mouse->button_flags &= ~MOUSE_M_DN;
	//cout << "Mid button released" << endl;
	
	extractMouseInfo(event);

	Refresh();
}


void BasicGLPane::mouseRightDown(wxMouseEvent& event) {
	mouse->button_flags |= MOUSE_R_DN;
    //cout << "right button pressed" << endl;
	extractMouseInfo(event);
}
void BasicGLPane::mouseRightUp(wxMouseEvent& event)  {
	mouse->button_flags &= ~MOUSE_R_DN;
	//cout << "right button released" << endl;
	extractMouseInfo(event);

	Refresh();
}
void BasicGLPane::mouseWheelMoved(wxMouseEvent& event) { }
void BasicGLPane::mouseLeftWindow(wxMouseEvent& event)  {
	mouse->in_canvas_flag = false;
	//cout << "mouse left window" << endl;
}
void BasicGLPane::mouseEnteredWindow(wxMouseEvent& event)  {
	mouse->in_canvas_flag = true;
	//cout << "mouse entered window" << endl;
}
void BasicGLPane::keyPressed(wxKeyEvent& event) {}
void BasicGLPane::keyReleased(wxKeyEvent& event) {
	//normally wxWidgets sends key events to the window that has the focus
	//cout<<"key pressed " << event.GetUnicodeKey()<<endl;
	//cout<<event.GetUnicodeKey()<<endl;

    /*switch ( event.GetUnicodeKey() )
    {
        case 80:
		case 112:  //??
			//cout<<"P matched!"<<endl;
			paused_flag = !paused_flag;
			if (!paused_flag) {
				simThread->unPause();
			}			
            break;
    }*/

		//cout<<"key event!"<<" "<<event.GetUnicodeKey()<<" "<<event.GetKeyCode()<<endl;

	//simThread->mutexProtectSharedData.Lock();
	switch ( event.GetKeyCode() )
    {
        case WXK_UP:
			if (event.GetModifiers()== wxMOD_CMD)
			{}
			else 
			{}	  
            break;

		case WXK_DOWN:
			if (event.GetModifiers()== wxMOD_CMD)
			{}  
			else 
			{}	
			
			break;
    }

	//simThread->mutexProtectSharedData.Unlock(); 


}

void BasicGLPane::resized(wxSizeEvent& evt) {
	//	wxGLCanvas::OnSize(evt);
	
	//cout<<"resize event"<<endl;
	wxSize size = evt.GetSize();

	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	//cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;
	
    Refresh();
	
	//// Update();
}





int BasicGLPane::getWidth() {
    return GetSize().x;
}
int BasicGLPane::getHeight() {
    return GetSize().y;
}

void BasicGLPane::render( wxPaintEvent& evt ) {
	//cout<<"rendering event"<<endl;
    if(!IsShown()) return;
	if(!model_loaded) return;

	//wxGLCanvas::SetCurrent(*m_context);
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	SetCurrent();
	
    /* 
	void glViewport(GLint  x,  GLint  y,  GLsizei  width,  GLsizei  height);

	When a GL context is first attached to a window, width and height are set to the dimensions of that window.
    and the value for (x,y)[lower left corner of the viewport rectangle] is (0,0).           

	This means the glViewport can only be meaningfully applied after the SetCurrent();
	*/


	// primary view port:
	glScissor(0, 0, getWidth(), getHeight());
	glViewport (0, 0, getWidth(), getHeight());

	glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	camera->setPerspective(45.0, (GLfloat)(getWidth())/(GLfloat)getHeight(), 1.0, 200.0);
	camera->update(mouse);
	camera->applyView();  // applyView func. basically: switches to GL_MODELVIEW, load identity then perform a gluLookAt
						  // gluLookAt computes the inverse camera transform and multiplies it onto the current matrix stack

	// set up the lights locations so that they would be fixed with the scene, not with the camera
	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position0);
	
	GLfloat light_position1[] = { -1.0, -2.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT1, GL_POSITION, light_position1);

	// -----------------------------------------------------------
	(dmEnvironment::getEnvironment())->draw();
	
	// -----------------------------------------------------------

	userGraphics();

	// Draw Robot!
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		if (IsWireframe == false )
		{
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		}
		else
		{   
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		}

		if (frame->showSkeleton->IsChecked()) 
		{
			G_robot->drawSkeleton();
		}

		if (frame->showRobot->IsChecked())
		{
			G_robot->draw();
		}

		glPopAttrib();
	}
	// -----------------------------------------------------------

	// Draw Axes
	{
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(2.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor3f(0.0, 1.0, 0.0);
		glVertex3f(0.0, 2.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 0.0, 2.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
	}

	// get the screen locations of the axis tips (X,Y,Z,O)
	// GLdouble modelview[16], projection[16];
	// GLint vp[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, vp);

	//GLdouble tx, ty, tz;
	//gluProject(0.0, 0.0, 0.0, modelview, projection, vp, &tx, &ty, &tz);

	GLdouble Xvp[3], Yvp[3], Zvp[3], Ovp[3];
	gluProject(2.05, 0.0, 0.0, modelview, projection, vp, &Xvp[0], &Xvp[1], &Xvp[2]);
	gluProject(0.0, 2.05, 0.0, modelview, projection, vp, &Yvp[0], &Yvp[1], &Yvp[2]);
	gluProject(0.0, 0.0, 2.05, modelview, projection, vp, &Zvp[0], &Zvp[1], &Zvp[2]);
	gluProject(0.0, 0.0, 0.0,  modelview, projection, vp, &Ovp[0], &Ovp[1], &Ovp[2]);


	glColor3f(1.0, 1.0, 1.0); // prevent the drawing color contaminating the texture.

	// Rendering text on the primary viewport
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		
		GLint viewport [4];
		glGetIntegerv (GL_VIEWPORT, viewport);
		gluOrtho2D (0, viewport[2], viewport[3], 0);
		// build a orthographic projection matrix using width and height of view port
		
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity();
		glDisable(GL_LIGHTING); // need to turn off light before rendering texts, otherwise the colors won't show.
		glDepthFunc (GL_ALWAYS);
		
		int len;
		
		/*glColor3f (0,0,0);
		glRasterPos2f(10, 20);
		char * displaytext = (char *) "Optimization Test";
		len = (int) strlen(displaytext);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, displaytext[i]);*/

		glColor3f (1,1,1);
		glRasterPos2f(10, 40);
		char buffer [50];
		sprintf (buffer, "sim time: %.3f s", simThread->sim_time);
		len = (int) strlen(buffer);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);  //GLUT_BITMAP_HELVETICA_10

		glColor3f (1,1,1);
		//ty = viewport[3]-ty;
		//glRasterPos2f(tx, ty);
		char* axis_text;
		axis_text = (char *) "X";
		glRasterPos2f(Xvp[0], viewport[3]-Xvp[1]);
		len = (int) strlen(axis_text);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis_text[i]);  
		axis_text = (char *) "Y";
		glRasterPos2f(Yvp[0], viewport[3]-Yvp[1]);
		len = (int) strlen(axis_text);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis_text[i]);  
		axis_text = (char *) "Z";
		glRasterPos2f(Zvp[0], viewport[3]-Zvp[1]);
		len = (int) strlen(axis_text);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis_text[i]);  
		axis_text = (char *) "O";
		glRasterPos2f(Ovp[0], viewport[3]-Ovp[1]);
		len = (int) strlen(axis_text);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis_text[i]);  

		userGraphics2DPrimary();
		
		glDepthFunc (GL_LESS);
		//glMatrixMode(GL_MODELVIEW);
		glColor3f(1.0, 1.0, 1.0); 
	
	}

	// auxiliary view ports
/*	glScissor(0.7*getWidth(), 0.7*getHeight(), getWidth() - 0.7*getWidth(), getHeight()-0.7*getHeight() );
	glViewport (0.7*getWidth(), 0.7*getHeight(), getWidth() - 0.7*getWidth(), getHeight()-0.7*getHeight());
	glClearColor (0.0, 0.0, 0.0,1.0); // background colour  lyp
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity ();
	
	gluOrtho2D (-5, 5, -5, 5);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();

	// red triangle
	glColor4f(1, 0, 0, 1);
	glBegin(GL_TRIANGLES);                      // Drawing Using Triangles
	glVertex3f( 0.0f, 1.0f, 0.0f);              // Top
	glVertex3f(-1.0f,-1.0f, 0.0f);              // Bottom Left
	glVertex3f( 1.0f,-1.0f, 0.0f);              // Bottom Right
	glEnd(); 
	glColor3f(1.0, 1.0, 1.0); 
		

	glDepthFunc (GL_ALWAYS);
	glColor3f (0,1,0);
	glRasterPos2f(0, 0);
	char * displaytext = (char *) "Contact";
	int len = (int) strlen(displaytext);
	for (int i = 0; i<len; ++i)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, displaytext[i]); //GLUT_BITMAP_HELVETICA_18
	glDepthFunc (GL_LESS);
	glColor3f(1.0, 1.0, 1.0); */

	glEnable (GL_LIGHTING);// ****

 	glFlush ();
    SwapBuffers();



}

void BasicGLPane::glInit()
{
	//glEnable(GL_MULTISAMPLE);
	//glHint(GL_MULTISAMPLE_FILTER_HINT_NV,GL_NICEST);
	glEnable(GL_SCISSOR_TEST);
		GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
		GLfloat light_diffuse[] = { 0.7, 0.7, 0.7, 1.0 };
		GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
		// light_position is NOT default value
		//GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
		
		glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
		glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
		glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
		//glLightfv (GL_LIGHT0, GL_POSITION, light_position);
		
		glLightfv (GL_LIGHT1, GL_AMBIENT, light_ambient);
		glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);
		glLightfv (GL_LIGHT1, GL_SPECULAR, light_specular);
		
	
		glEnable (GL_LIGHTING);
		glEnable (GL_LIGHT0);
		glEnable (GL_LIGHT1);
		glDepthFunc(GL_LESS);
		glEnable(GL_DEPTH_TEST);
		

		
		// Enable Blending
		glEnable(GL_BLEND);
		// Specifies pixel arithmetic
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		//glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
		//glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

		// ****
		// to enable LINE_SMOOTHing, you need to have blending enabled
		glEnable(GL_LINE_SMOOTH);
		//glEnable(GL_POINT_SMOOTH);
		//glEnable(GL_POLYGON_SMOOTH);

	
		glLineWidth (1.5);
		// ****
		//glShadeModel(GL_FLAT);
		glShadeModel(GL_SMOOTH);
		
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		
		// ****
		glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);//!!

		// When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
		// BUT: 
		// The above two lines mean that glMaterial will control the polygon's specular and emission colors
		// and the ambient and diffuse will both be set using glColor. 
		// glColorMaterial makes it possible to change a subset of material parameters for each
        //    vertex using only the glColor command without calling glMaterial. - yiping
		
		quadratic=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
		gluQuadricNormals(quadratic, GLU_SMOOTH);   // Create Smooth Normals ( NEW )
		gluQuadricTexture(quadratic, GL_TRUE);      // Create Texture Coords ( NEW )
}

void BasicGLPane::OnExit()
{
	cout << "Exiting GL Pane" << endl;
}






// rerender the scene upon Timer event

void BasicGLPane::updateSim(wxTimerEvent & event) 
{	
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);

	cnt ++;
	if (cnt == 10)
	{
		real_time_ratio = (simThread->sim_time-last_sim_time)/timeDiff(last_draw_tv, tv_now);
		last_sim_time = simThread->sim_time;
		dmGetSysTime(&last_draw_tv);
		cnt = 0;
	}


	

	Refresh(); // ask for repaint

#ifdef SYNC_GRAPHICS
	Update(); // execute all the pending repaint events

	// TryLock() is especially important for the main (GUI) thread, 
	// which should never be blocked because your application would become unresponsive to user input. 
	// also, the main thread can still proceed if it cannot lock the mutex

	if (simThread->re_mutex.TryLock()== wxMUTEX_NO_ERROR )// is simThread waiting for me?
	{
		simThread->refreshCondition->Signal();
		simThread->re_mutex.Unlock();
	}
#endif

	timer_count++;
	
	double rtime = timeDiff(first_tv, tv_now);
	double update_time = timeDiff(update_tv, tv_now);
	
	if (update_time > 2.5)
	{
		cerr << "\033[1;31m sim_time\033[0m/real_time: " << "\033[1;31m"<< simThread->sim_time << "\033[0m"<< '/' << rtime
		<< "  frame_rate: " << (double) timer_count/rtime << endl;

		//cerr<<"Sim Time: \033[1;31m"<< simThread->sim_time<< "\033[0m\n";
		
		dmGetSysTime(&update_tv);
	}
}
