/*
 *  BasicGLPane.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/25/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */
#include "GlobalDefines.h"

#include "BasicGLPane.h"
#include <dmEnvironment.hpp>

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
//EVT_KEY_UP(BasicGLPane::keyReleased)
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
	
	glViewport (0, 0, size.GetWidth(), size.GetHeight());
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
	SetCurrent();
}

BasicGLPane::~BasicGLPane()
{
	timer->Stop();
	delete timer;
}
void BasicGLPane::restartTimer() {
	if (timer->IsRunning()) {
		timer->Stop();
	}
	timer->Start(1000./render_rate);
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
	camera->update(mouse);
	camera->applyView();
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
	camera->update(mouse);
	camera->applyView();
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
	camera->update(mouse);
	camera->applyView();
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
	camera->update(mouse);
	camera->applyView();
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
		case 112:
			//cout<<"P matched!"<<endl;
			paused_flag = !paused_flag;
			if (!paused_flag) {
				simThread->unPause();
			}			
            break;
    }*/
	
}

void BasicGLPane::resized(wxSizeEvent& evt) {
	//	wxGLCanvas::OnSize(evt);
	
	//cout<<"resize event"<<endl;
	wxSize size = evt.GetSize();
	glViewport (0, 0, size.GetWidth(), size.GetHeight());
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	//cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;
	
	//camera->setPerspective(45.0, (GLfloat)size.GetWidth()/(GLfloat)size.GetHeight(), 1.0, 200.0);
	
	//camera->setViewMat(view_mat);
	//camera->applyView();
	
    Refresh();
	
	//// Update();
}

/** Inits the OpenGL viewport for drawing in 3D. */
void BasicGLPane::prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y) {
	
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
    glClearDepth(1.0f);	// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	
    glEnable(GL_COLOR_MATERIAL);
	
    glViewport(topleft_x, topleft_y, bottomrigth_x-topleft_x, bottomrigth_y-topleft_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	
    float ratio_w_h = (float)(bottomrigth_x-topleft_x)/(float)(bottomrigth_y-topleft_y);
    gluPerspective(45 /*view angle*/, ratio_w_h, 0.1 /*clip close*/, 200 /*clip far*/);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	
}

/** Inits the OpenGL viewport for drawing in 2D. */
void BasicGLPane::prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
    glEnable(GL_TEXTURE_2D);   // textures
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	
    glViewport(topleft_x, topleft_y, bottomrigth_x-topleft_x, bottomrigth_y-topleft_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    gluOrtho2D(topleft_x, bottomrigth_x, bottomrigth_y, topleft_y);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
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
	
	static bool runOnce =true;
	if (runOnce) {
		cout<<"initilize scene..."<<endl;
		
		camera->setRadius(8.0);
		camera->setCOI(3.0, 3.0, 0.0);
		camera->setTranslationScale(0.02f);
		
		
		glInit();
		dmEnvironment::getEnvironment()->drawInit();
		runOnce = false;
	}
    
    
	
	// When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);*/
	
	glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode (GL_MODELVIEW);
	glPushMatrix ();
	
	// ===============================================================
	(dmEnvironment::getEnvironment())->draw();
	
	// =========
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
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
		G_robot->draw();
		glPopAttrib();
	}
	
	// ===============================================================
	
	glDisable (GL_LIGHTING);
	
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
	
	
	//glEnable (GL_LIGHTING);
	
	glPopMatrix ();
	
	// Setup Viewport
	{
		//// Write some information on the viewport
		// we are currently in MODEL_VIEW
		glPushMatrix ();
		glLoadIdentity ();
		glMatrixMode(GL_PROJECTION);
		glPushMatrix ();
		glLoadIdentity();
		
		GLint viewport [4];
		glGetIntegerv (GL_VIEWPORT, viewport);
		gluOrtho2D (0,viewport[2], viewport[3], 0);
		// build a orthographic projection matrix using width and height of view port
		
		
		glDisable (GL_LIGHTING);// ****
		glDepthFunc (GL_ALWAYS);
		glColor3f (0,0,0);
		glRasterPos2f(10, 20);
		char * displaytext = (char *) "Humanoid Model";
		int len = (int) strlen(displaytext);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, displaytext[i]);
		glColor3f (1,1,1);
		glRasterPos2f(10, 40);
		char buffer [50];
		sprintf (buffer, "%.2f", simThread->sim_time);
		len = (int) strlen(buffer);
		for (int i = 0; i<len; ++i)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);
		
		glDepthFunc (GL_LESS);
		glPopMatrix ();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix ();
		glEnable (GL_LIGHTING);// ****
	}
	
	
    //  When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);
	
 	glFlush ();
    SwapBuffers();
}

void BasicGLPane::glInit()
{
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
		
		// ****
		glEnable(GL_LINE_SMOOTH);
		
		// Enable Blending
		glEnable(GL_BLEND);
		// Specifies pixel arithmetic
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glLineWidth (1.5);
		// ****
		//glShadeModel(GL_FLAT);
		glShadeModel(GL_SMOOTH);
		
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		
		// ****
		glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);//!!
		// The above two lines mean that glMaterial will control the polygon's specular and emission colors
		// and the ambient and diffuse will both be set using glColor. - yiping
		
		quadratic=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
		gluQuadricNormals(quadratic, GLU_SMOOTH);   // Create Smooth Normals ( NEW )
		gluQuadricTexture(quadratic, GL_TRUE);      // Create Texture Coords ( NEW )
}

 void BasicGLPane::OnExit()
{
	cout << "Exiting GL Pane" << endl;
}
