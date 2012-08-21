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
using namespace Eigen;

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
EVT_KEY_DOWN(BasicGLPane::keyPressed)
EVT_KEY_UP(BasicGLPane::keyReleased)
EVT_MOUSEWHEEL(BasicGLPane::mouseWheelMoved)
EVT_PAINT(BasicGLPane::render)
//EVT_IDLE(BasicGLPane::updateSim)
EVT_TIMER(BasicGLPane::TIMER_ID,BasicGLPane::updateSim)

END_EVENT_TABLE()

BasicGLPane::BasicGLPane(wxFrame* parent, int* args, const wxSize &size) :
wxGLCanvas(parent, wxID_ANY, wxDefaultPosition, size, wxFULL_REPAINT_ON_RESIZE,wxT("GLWindow"),args)
{
	cout << "Creating Mouse" << endl;
	mouse = new wxDMGLMouse(); // has to be put in the front
	cout << "Creating Camera" << endl;
	camera = new wxDMGLPolarCamera_zup();
	cout << "Mouse and Camera Created" << endl;
	
	//glViewport (0, 0, size.GetWidth(), size.GetHeight());
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
	//camera->update(mouse);
	//camera->applyView();
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
	//camera->update(mouse);
	//camera->applyView();
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
	//camera->update(mouse);
	//camera->applyView();
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
	//camera->update(mouse);
	//camera->applyView();
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
void BasicGLPane::keyPressed(wxKeyEvent& event) 
{
	if (event.GetUnicodeKey() == 32) {
		mouse->button_flags |= MOUSE_M_DN;
	}
}
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
	if (event.GetUnicodeKey() == 32) {
		mouse->button_flags &= ~MOUSE_M_DN;
		//camera->update(mouse);
		//camera->applyView();
		Refresh();
	}
	
}

void BasicGLPane::resized(wxSizeEvent& evt) {
	//	wxGLCanvas::OnSize(evt);
	
	//cout<<"resize event"<<endl;
	wxSize size = evt.GetSize();
	//glViewport (0, 0, size.GetWidth(), size.GetHeight());
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	//cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;
	
	//camera->setPerspective(45.0, (GLfloat)size.GetWidth()/(GLfloat)size.GetHeight(), 1.0, 200.0);
	
	//camera->setViewMat(view_mat);
	//camera->applyView();
	
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
	//cout << "Paint" << endl;
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	SetCurrent();
	//cout << "Current Set" << endl;
    
	
	// primary view port:
	glScissor(0, 0, getWidth(), getHeight());
	glViewport (0, 0, getWidth(), getHeight());
	
	glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	camera->setPerspective(45.0, (GLfloat)getWidth()/(GLfloat)getHeight(), 1.0, 200.0);
	camera->update(mouse);
	camera->applyView();
	
	// if you want the GL light to move with the camera, comment the following two lines - yiping
	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position0);
	
	GLfloat light_position1[] = { -1.0, -1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
	
	
    
	
	// When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);*/
	
	glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode (GL_MODELVIEW);
	glPushMatrix ();
	
	// ===============================================================
	(dmEnvironment::getEnvironment())->draw();
	
	// =========
	
	glDisable (GL_LIGHTING);
	
	// Draw Axes
	{
		Float AxisLength = .5;
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(AxisLength, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor3f(0.0, 1.0, 0.0);
		glVertex3f(0.0, AxisLength, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 0.0, AxisLength);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		Float qBase[7], qdBase[7];
		G_robot->getLink(0)->getState(qBase,qdBase);
		
		Eigen::Matrix<GLfloat,3,1> p0,p1,p2,pStart,pEnd,zup;
		p0<< 0,0,0;
		p1<< 0,0,.7;
		p2<< qBase[4], qBase[5], qBase[6];
		
		pStart = p0;
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 0.0);
		
		for (Float t=.01; t<=1; t+=.01) {
			Float tc = 1-t;
			pEnd = tc*tc*p0+2*(1-t)*t*p1 + t*t*p2;
			glVertex3fv(pStart.data());
			glVertex3fv(pEnd.data());
			pStart.swap(pEnd);
		}
		glEnd();
		
		pStart = p2-p1;
		pStart /= pStart.norm();
		
		glPushMatrix();
		glTranslatef(p2(0), p2(1), p2(2));
		
		zup << 0,0,1;
		
		const double cosTheta = zup.dot(pStart);
		double theta = acos(cosTheta);
		const double sinTheta = sin(theta);
		
		// Exploit the special form of the rotation matrix (explained above) for find the axis of rotation
		double rX,rY,rZ;
		if(theta > 0) {	
			rX = - pStart(1)/sinTheta;
			rY =   pStart(0)/sinTheta;
			rZ = 0;
		}
		else {
			rX = 0;
			rY = 0;
			rZ = 1;
		}
		glRotated(180/M_PI * theta, rX, rY, rZ);
		
		Float div = 4;
		glTranslatef(0.0, 0.0, -.15f/div);
		gluCylinder(frame->glPane->quadratic,.05f/div,0.0f,.15f/div,16,16);
		
		
		glPopMatrix();
		
		
		
		theta = acos(qBase[3])*2 * 180 / M_PI;
		
		glPushMatrix();
		glTranslatef(qBase[4], qBase[5], qBase[6]);
		glRotatef(theta, qBase[0], qBase[1], qBase[2]);
		
		AxisLength = .25;
		glBegin(GL_LINES);
		glColor4f(1.0, 0.0, 0.0,1.0);
		glVertex3f(AxisLength, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor4f(0.0, 1.0, 0.0,1.0);
		glVertex3f(0.0, AxisLength, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		
		glBegin(GL_LINES);
		glColor4f(0.0, 0.0, 1.0,1.0);
		glVertex3f(0.0, 0.0, AxisLength);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();
		glPopMatrix();
		
		
		
	}
	glEnable(GL_LIGHTING);
	
	
	//cout << "User Graphics" << endl;
	userGraphics();
	//cout << "Complete" << endl;;
	if(glIsEnabled(GL_BLEND)) {
		//glDepthMask(GL_FALSE);
	}
	
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
		//cout << "Draw Robot" << endl;
		if (frame->showSkeleton->IsChecked()) {
			G_robot->drawSkeleton();
		}
		
		G_robot->draw();
		//cout << "Robot Drawn" << endl;
		glPopAttrib();
	}
	if(glIsEnabled(GL_BLEND)) {
		//glDepthMask(GL_TRUE);
	}
	
	// ===============================================================
	
	
	
	
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
		glColor3f(1.0, 1.0, 1.0); 
		glEnable (GL_LIGHTING);// ****
		
	}
	
	
    //  When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);
	
	//cout << "Flushing" << endl;
 	glFlush ();
	//cout << "Swapping" << endl;
    SwapBuffers();
	//cout << "Done drawing" << endl;
}

void BasicGLPane::glInit()
{
	//glEnable(GL_MULTISAMPLE);
	glHint(GL_MULTISAMPLE_FILTER_HINT_NV,GL_NICEST);
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
		
		// ****
		glEnable(GL_LINE_SMOOTH);
		//glEnable(GL_POINT_SMOOTH);
		
		//glEnable(GL_POLYGON_SMOOTH);
		
		// Enable Blending
		//glEnable(GL_BLEND);
		// Specifies pixel arithmetic
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		//glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
		//glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	
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
