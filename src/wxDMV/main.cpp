#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "main.h"

 
// include OpenGL
#ifdef __WXMAC__
#include "OpenGL/glu.h"
#include "OpenGL/gl.h"
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif
 
void myInit (void); 

class MyApp: public wxApp
{
    virtual bool OnInit();
    
    wxFrame *frame;
    BasicGLPane * glPane;
	wxButton *welcomebutton;
	wxPanel *toolpanel;
public:
    
};
 
IMPLEMENT_APP(MyApp)
 
 
bool MyApp::OnInit()
{
	//cout << "Init " << endl;


    //---------------------------------------------------


	mouse = new wxDMGLMouse(); // has to be put in the front


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
	toolpanel_sizer->Add(welcomebutton, 0 );
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
END_EVENT_TABLE()

 
// some useful events to use
void BasicGLPane::extractMouseInfo(wxMouseEvent& event)
{
	wxPoint pos = event.GetPosition();
	mouse->xwin = pos.x;
	mouse->ywin = pos.y;
	mouse->xchan = (2.0*((GLfloat) mouse->xwin) - mouse->win_size_x)/
		                 (GLfloat) mouse->win_size_x;
	mouse->ychan = (2.0*((GLfloat) mouse->ywin) - mouse->win_size_y)/
		                 (GLfloat) mouse->win_size_y;

	//cout << "Pos: " << mouse->xwin << ", " << mouse->ywin << "  Button state: "<< mouse->button_flags << endl;
}

void BasicGLPane::mouseMoved(wxMouseEvent& event)
{
	//cout << "mouse moving" << endl;
	//cout << "mouse moving          "<< mouse->button_flags  << endl;
	extractMouseInfo(event);
	camera->update(mouse);
	camera->applyView();
	Refresh();
}
void BasicGLPane::mouseLeftDown(wxMouseEvent& event)
{
	mouse->button_flags |= MOUSE_L_DN;
	//cout << "left button pressed" << endl;

	extractMouseInfo(event);
	SetFocus();
}

void BasicGLPane::mouseLeftUp(wxMouseEvent& event) 
{
	mouse->button_flags &= ~MOUSE_L_DN;
	//cout << "left button released" << endl;
	//cout << "left button released  "<< mouse->button_flags << endl;
	extractMouseInfo(event);
	camera->update(mouse);
	camera->applyView();
	Refresh();
}
void BasicGLPane::mouseRightDown(wxMouseEvent& event) 
{
	mouse->button_flags |= MOUSE_R_DN;
    //cout << "right button pressed" << endl;
	extractMouseInfo(event);
}
void BasicGLPane::mouseRightUp(wxMouseEvent& event) 
{
	mouse->button_flags &= ~MOUSE_R_DN;
	//cout << "right button released" << endl;
	extractMouseInfo(event);
	camera->update(mouse);
	camera->applyView();
	Refresh();
}

void BasicGLPane::mouseWheelMoved(wxMouseEvent& event) { }

void BasicGLPane::mouseLeftWindow(wxMouseEvent& event) 
{
	mouse->in_canvas_flag = false;
	//cout << "mouse left window" << endl;
}
void BasicGLPane::mouseEnteredWindow(wxMouseEvent& event) 
{
	mouse->in_canvas_flag = true;
	//cout << "mouse entered window" << endl;
}
void BasicGLPane::keyPressed(wxKeyEvent& event) {}
void BasicGLPane::keyReleased(wxKeyEvent& event)
{
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
 




// Vertices and faces of a simple cube to demonstrate 3D render
// source: 
GLfloat v[8][3];
GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
    {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
    {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };
 
 
 
BasicGLPane::BasicGLPane(wxFrame* parent, int* args, const wxSize &size) :
    wxGLCanvas(parent, wxID_ANY, wxDefaultPosition, size, wxFULL_REPAINT_ON_RESIZE,wxT("GLWindow"),args)
{
	//m_context = new wxGLContext(this);
    // prepare a simple cube to demonstrate 3D render
    // source: 
    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -1;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = 1;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -1;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = 1;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = 1;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = -1;    
 
    // To avoid flashing on MSW
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);

}
 
BasicGLPane::~BasicGLPane()
{
	//delete m_context;
}
 
void BasicGLPane::resized(wxSizeEvent& evt)
{
//	wxGLCanvas::OnSize(evt);
	
	cout<<"resize event"<<endl;
	wxSize size = evt.GetSize();
	glViewport (0, 0, size.GetWidth(), size.GetHeight());
	mouse->win_size_x = size.GetWidth();
	mouse->win_size_y = size.GetHeight();
	cout << "w= " << mouse->win_size_x << ",  h=" << mouse->win_size_y << endl;

	camera->setPerspective(45.0, (GLfloat)size.GetWidth()/(GLfloat)size.GetHeight(), 1.0, 200.0);

	//camera->setViewMat(view_mat);
	//camera->applyView();

    Refresh();

	//// Update();
}
 
/** Inits the OpenGL viewport for drawing in 3D. */
void BasicGLPane::prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y)
{
	
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
void BasicGLPane::prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y)
{
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
 
int BasicGLPane::getWidth()
{
    return GetSize().x;
}
 
int BasicGLPane::getHeight()
{
    return GetSize().y;
}
 
 
void BasicGLPane::render( wxPaintEvent& evt )
{
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
		char *filename = (char *) "flywheel_biped.cfg";
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


		// -----------------------
	}

	//cout<<"entering display()"<<endl;
	display();

 	glFlush ();
    SwapBuffers();
}


MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
       : wxFrame(NULL, -1, title, pos, size)
{
    CreateStatusBar();
    SetStatusText( _("Welcome to DynaMechs wxViewr!") );
}


void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
	// Note wxMessageBox might crash under Windows OS.
    wxMessageBox( _("This is a wxWidgets Hello world sample"),
                  _("About Hello World"),
                  wxOK | wxICON_INFORMATION, this );
}


void myInit (void)
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
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	// Enable Blending
	glEnable(GL_BLEND);
	// Specifies pixel arithmetic
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
}




void BasicGLPane::display (void)
{
	glClearColor (0.49, 0.62, 0.75,1.0); /* background colour */ //lyp
	//glClearColor (1.0, 1.0, 1.0,1.0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



	glMatrixMode (GL_MODELVIEW);

	glPushMatrix ();
	// ------------------------------------------
	(dmEnvironment::getEnvironment())->draw();

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
	// ------------------------------------------

	glPopMatrix ();




	//glEnable (GL_LIGHTING);// ****


	// When lighting is enabled, the primary color is calculated from the lighting equation instead of being taken from glColor and equivalent functions
	//glEnable (GL_LIGHTING);


}


