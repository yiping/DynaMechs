#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "BasicGLPane.h"
#include "wxBoxSlider.h"

//This is a dirty workaround
#undef Success

#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmu.h>
#include <dmGL.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// include OpenGL
#ifdef __WXMAC__
#include "OpenGL/glu.h"
#include "OpenGL/gl.h"
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif

#include <dmTime.h>

#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmIntegRK4.hpp>
#include <dmIntegEuler.hpp>
#include <dmEnvironment.hpp>
#include <dmMDHLink.hpp>
#include <dmMobileBaseLink.hpp>
#include <wxDMGLMouse.hpp>
#include <wxDMGLPolarCamera_zup.hpp>


#include "GlobalDefines.h"

//#include "functions.h"
//#include "global_typedef.h"

//#include "mosek.h" /* Include the MOSEK definition file. */
#include "TaskSpaceController.h"
#include "humanoidControl.h"

//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO

 
const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

wxDMGLMouse *mouse;
wxDMGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float rtime=0.0;

bool IsWireframe = false;
bool paused_flag = true;


dmIntegEuler *G_integrator;

vector<LinkInfoStruct*> G_robot_linkinfo_list;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;
bool runOnce = true;


GLUquadricObj *quadratic;

void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength);

 
void myInit (void); 

class MyApp: public wxApp
{
    virtual bool OnInit();
    
    wxFrame *frame;
    BasicGLPane * glPane;
	wxButton *welcomebutton;
	wxPanel *toolpanel;
	wxButton *saveViewbutton;
	wxButton *applyViewbutton;
	
	
public:
    
};
 
IMPLEMENT_APP(MyApp)
 
 
bool MyApp::OnInit()
{
	//cout << "Init " << endl;


    //---------------------------------------------------


	mouse = new wxDMGLMouse(); // has to be put in the front

	//wxSpinCtrlDouble * test;

	//---------------------------------------------------

    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer *toolpanel_sizer = new wxBoxSizer( wxVERTICAL);
	
	//cout << "Frame" << endl;
    frame = new MainFrame(wxT("Hello GL World"), wxPoint(50,50), wxSize(600,400));
	
	toolpanel = new wxPanel((wxFrame*) frame, -1, wxPoint(-1,-1), wxSize(200,400));


    int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
   
	//cout << "Pane " << endl;
    glPane = new BasicGLPane( (wxFrame*) frame, args, wxSize(400,400));
	
	
	int i, j;
	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			view_mat[i][j] = 0.0;
		}
		view_mat[i][i] = 1.0;
	}
	camera = new wxDMGLPolarCamera_zup();
	camera->setRadius(8.0);
	camera->setCOI(3.0, 3.0, 0.0);
	camera->setTranslationScale(0.02f);
	
	
	welcomebutton = new wxButton( toolpanel, wxID_OK, wxT("Welcome"));
	saveViewbutton = new wxButton( toolpanel, BUTTON_SaveView, wxT("Save View"));
	applyViewbutton = new wxButton( toolpanel, BUTTON_ApplyView, wxT("Apply View"));
	showCoM = new wxCheckBox(toolpanel,CHECKBOX_ShowCoM,wxT("Show CoM"));
	showGRF = new wxCheckBox(toolpanel,CHECKBOX_ShowGRF,wxT("Show GRF"));
	showNetForceAtGround = new wxCheckBox(toolpanel,CHECKBOX_ShowNetForceAtGround,wxT("Show Net Force (Ground)"));
	showNetForceAtCoM	 = new wxCheckBox(toolpanel,CHECKBOX_ShowNetForceAtCoM,wxT("Show Net Force (CoM)"));
	
	toolpanel_sizer->Add(welcomebutton, 0 ,wxALL | wxALIGN_CENTER,2);
	
	
	toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Camera Options")),0,wxALL,2);
	toolpanel_sizer->Add(saveViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
	toolpanel_sizer->Add(applyViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
	
	toolpanel_sizer->AddSpacer(15);
	toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("View Options")),0,wxALL,2);
	toolpanel_sizer->Add(showCoM, 0 ,wxALL  ,2);
	toolpanel_sizer->Add(showGRF, 0 ,wxALL ,2);
	toolpanel_sizer->Add(showNetForceAtGround, 0,wxALL,2 );
	toolpanel_sizer->Add(showNetForceAtCoM, 0,wxALL,2 );
	
	toolpanel_sizer->AddSpacer(15);					 
	toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Control Options")),0,wxALL,2);	
	
	wxBoxSlider * CoMControlSlider = new wxBoxSlider(toolpanel,-1,5,15,10);
	
	
	
	
	toolpanel_sizer->Add(CoMControlSlider,0);
	
	showCoM->SetValue(true);
	showGRF->SetValue(true);
	
	
	toolpanel->SetSizer(toolpanel_sizer);
	
    
    sizer->Add(glPane, 1, wxEXPAND );
	sizer->Add(toolpanel, 0, wxEXPAND);

	sizer->SetSizeHints(frame);
	frame->SetSizer(sizer);

    frame->SetAutoLayout(true);
	
    frame->Show();


    return true;
} 
 
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
	EVT_IDLE(BasicGLPane::updateSim)
END_EVENT_TABLE()
 
BEGIN_EVENT_TABLE(MainFrame, wxFrame)
	EVT_BUTTON  (wxID_OK,   MainFrame::OnAbout)
	EVT_BUTTON  (BUTTON_SaveView,   MainFrame::OnSaveView)
	EVT_BUTTON  (BUTTON_ApplyView,   MainFrame::OnApplyView)
END_EVENT_TABLE()

 
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
	cout << "mid button pressed" << endl;
	
	extractMouseInfo(event);
	SetFocus();
}
void BasicGLPane::mouseMiddleUp(wxMouseEvent& event)  {
	mouse->button_flags &= ~MOUSE_M_DN;
	cout << "Mid button released" << endl;
	
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
    switch ( event.GetUnicodeKey() )
    {
        case 80:
		case 112:
			//cout<<"P matched!"<<endl;
			paused_flag = !paused_flag;
            break;

    }

}
 
 
 
BasicGLPane::BasicGLPane(wxFrame* parent, int* args, const wxSize &size) :
    wxGLCanvas(parent, wxID_ANY, wxDefaultPosition, size, wxFULL_REPAINT_ON_RESIZE,wxT("GLWindow"),args)
{
 
	glViewport (0, 0, size.GetWidth(), size.GetHeight());
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;

	
    // To avoid flashing on MSW
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);

    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str()};
	glutInit(&argc, argv);
}
  
void BasicGLPane::resized(wxSizeEvent& evt) {
//	wxGLCanvas::OnSize(evt);
	
	cout<<"resize event"<<endl;
	wxSize size = evt.GetSize();
	glViewport (0, 0, size.GetWidth(), size.GetHeight());
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;

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
    
    //wxGLCanvas::SetCurrent(*m_context);
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	SetCurrent();

	// initialization has to be put here somehow.
	if (runOnce == true)
	{
		cout<<"initilize scene..."<<endl;
		myInit();

		// load robot stuff
		char *filename = (char *) "humanoid.cfg";
		ifstream cfg_ptr;
		cfg_ptr.open(filename);

		// Read simulation timing information.
		readConfigParameterLabel(cfg_ptr,"Integration_Stepsize");
		cfg_ptr >> idt;
		if (idt <= 0.0)
		{
			cerr << "main error: invalid integration stepsize: " << idt << endl;
			exit(3);
		}

		readConfigParameterLabel(cfg_ptr,"Display_Update_Rate");
		cfg_ptr >> render_rate;
		if (render_rate < 1) render_rate = 1;

		// ------------------------------------------------------------------
		// Initialize DynaMechs environment - must occur before any linkage systems
		char env_flname[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr,"Environment_Parameter_File");
		readFilename(cfg_ptr, env_flname);
		dmEnvironment *environment = dmuLoadFile_env(env_flname);
		environment->drawInit();
		dmEnvironment::setEnvironment(environment);

		// ------------------------------------------------------------------
		// Initialize a DynaMechs linkage system
		char robot_flname[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr,"Robot_Parameter_File");
		readFilename(cfg_ptr, robot_flname);
		G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));

		//G_integrator = new dmIntegRK4();
		G_integrator = new dmIntegEuler();
		G_integrator->addSystem(G_robot);

		dmGetSysTime(&last_tv);
		
		runOnce = false;
		
		// -- Project specific -- 
		initControl();
		// -----------------------
	}

	//cout<<"entering display()"<<endl;
	display();

 	glFlush ();
    SwapBuffers();
}


MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
       : wxFrame(NULL, -1, title, pos, size) {
    CreateStatusBar();
    SetStatusText( _("Welcome to DynaMechs wxViewr!") );
}

void MainFrame::OnSaveView(wxCommandEvent& WXUNUSED(event))
{
	float x, y, z;
	camera->getCOI(x, y, z);
	cout<<"current COI is: [ "<<x<<" "<<y<<" "<<z<<" ]"<<endl;
	float r;
	camera->getRadius(r);
	cout<<"current Radius is: " <<r<<endl;
	float elev;
	camera->getElevation(elev);
	cout<<"current elevation is: " <<elev<<endl;
	float azim;
	camera->getAzimuth(azim);
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
	camera->setRadius(r);
	camera->setCOI(x, y, z);
	camera->setElevation(elev);
	camera->setAzimuth(azim);
	
	Refresh();	
}



void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event)) {
	// Note wxMessageBox might crash under Windows OS.
    wxMessageBox( _("This is a wxWidgets Hello world sample"),
                  _("About Hello World"),
                  wxOK | wxICON_INFORMATION, this );
}


void myInit (void) {
	
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
	
	grfInfo.localContacts = 0;
	
	
}




void BasicGLPane::display (void) {
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
	
	
	if (showCoM->IsChecked()) {
		// Draw COM Info
		glBegin(GL_LINES);
		glColor4f(0.0, 0.0, 1.0,1.0);
		glVertex3f(ComDes[0], ComDes[1], ComDes[2]);
		glVertex3f(ComDes[0], ComDes[1], 0      );
		glEnd();
		
		glBegin(GL_LINES);
		glColor4f(1.0, 0.0, 0.0,1.0);
		glVertex3f(ComPos[0], ComPos[1], ComPos[2]);
		glVertex3f(ComPos[0], ComPos[1], 0      );
		glEnd();
		
		glPushMatrix();
		glTranslatef(ComPos[0],ComPos[1],ComPos[2]); 
		gluSphere(quadratic,.03f,32,32);
		glPopMatrix();
		//glTranslatef(-ComPos[0],-ComPos[1],-ComPos[2]);
	}
	 
	
	const Float forceScale = 250;
	glColor4f(0.0, 0.0, 0.0,0.75);
	
	if (showGRF->IsChecked()) {
		// Draw GRF Info
		for (int i=0; i< grfInfo.localContacts; i++) {
			Vector3F footPoint = grfInfo.pCoPs[i];
			Vector3F grf = grfInfo.fCoPs[i]/forceScale;
			
			drawArrow(footPoint, grf, .005, .01, .03);
			
		}
	}

	if (showNetForceAtGround->IsChecked()) {
		//Draw ZMPInfo
		Vector3F footPoint = grfInfo.pZMP;
		Vector3F grf = grfInfo.fZMP/forceScale;
		drawArrow(footPoint, grf, .005, .01, .03);
	}
	
	
	
	//glDisable(GL_BLEND);
	// =========
	
	
	
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
	
	// ===============================================================
	
	glDisable (GL_LIGHTING);
	
	
	
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
	
	
	
	//glEnable (GL_LIGHTING);
	
	glPopMatrix ();
	
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
    sprintf (buffer, "%f", sim_time);
    len = (int) strlen(buffer);
	for (int i = 0; i<len; ++i)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);
	glDepthFunc (GL_LESS);
	glPopMatrix ();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix ();
	glEnable (GL_LIGHTING);// ****
	
	
    //  When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);
	


}

void BasicGLPane::updateSim(wxIdleEvent& event) {
	
	if (!paused_flag)
	{
		if (sim_time > .2) {
			
			ControlInfo ci;
			HumanoidControl(ci); 
			
			cout << ci.totalTime << "\t" << ci.calcTime << "\t" << ci.setupTime << "\t" << ci.optimTime << "\t" << ci.iter << endl;
		}
		
		//exit(-1);
		for (int i=0; i<render_rate; i++)
		{
			
			G_integrator->simulate(idt);
			sim_time += idt;
		}
	}
	
	camera->setPerspective(45.0, (GLfloat)getWidth()/(GLfloat)getHeight(), 1.0, 200.0);
	camera->update(mouse);
	camera->applyView();
	Refresh(); // ask for repaint
	
	// if you want the GL light to move with the camera, comment the following two lines - yiping
	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position0);
	
	GLfloat light_position1[] = { -1.0, -1.0, 1.0, 0.0 };
	glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
	
	
	timer_count++;
	dmGetSysTime(&tv);
	
	double elapsed_time = ((double) tv.tv_sec - last_tv.tv_sec) +
	(1.0e-9*((double) tv.tv_nsec - last_tv.tv_nsec));
	
	if (elapsed_time > 2.5)
	{
		rtime += elapsed_time;
		cerr << "time/real_time: " << sim_time << '/' << rtime
		<< "  frame_rate: " << (double) timer_count/elapsed_time << endl;
		
		timer_count = 0;
		last_tv.tv_sec = tv.tv_sec;
		last_tv.tv_nsec = tv.tv_nsec;
	}
	
	// WakeUpIdle(); //?
	event.RequestMore();// render continuously, not only once on idle
}

void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength) {
	Vector3F zup; zup << 0,0,1;
	
	Vector3F normedDirection = direction.normalized();
	
	// Note: We need to create a rotation such that the z-axis points in the direction of the 'direction' argument
	//       Thus, we can realize the rotation with a pure x-y axial rotation
	//       The rotation matrix of the rotated frame 'r' to the current frame 'c' (c_R_r) has special form.
	//       It's 3-rd column in particular has form: [ wy*sin(theta) -wx*sin(theta) (1- cos(theta))]'
	//       where theta , [wx wy 0] is the angle, axis of rotation
	
	// Find the rotation angle by comparing the z-axis of the current and rotated frame
	const double cosTheta = zup.dot(normedDirection);
	const double theta = acos(cosTheta);
	const double sinTheta = sin(theta);
	
	
	// Exploit the special form of the rotation matrix (explained above) for find the axis of rotation
	double rX, rY, rZ;
	if(theta > 0) {	
		rX = - normedDirection(1)/sinTheta;
		rY =   normedDirection(0)/sinTheta;
		rZ = 0;
	}
	else {
		rX = 0;
		rY = 0;
		rZ = 1;
	}
	
	glPushMatrix();
	glTranslatef(location(0), location(1), location(2));
	glRotated(RADTODEG * theta, rX, rY, rZ);
	
	
	double cylinderLength = direction.norm();
	if (cylinderLength > headLength) {
		cylinderLength -= headLength;
	}
	else {
		headLength = cylinderLength ;
		cylinderLength = 0;
	}

	
	//Draw Cylinder
	gluCylinder(quadratic,lineWidth,lineWidth,cylinderLength,32,32);
	
	//Draw Cylinder Base
	gluDisk(quadratic,0,lineWidth,32,32);
	
	glTranslatef(0, 0, cylinderLength);
	//Draw Arrowhead
	gluCylinder(quadratic,headWidth,0.0f,headLength,32,32);
	
	//Draw Arrowhead Base
	gluDisk(quadratic,lineWidth,headWidth,32,32);
	
	glEnd();
	glPopMatrix();
	
}

