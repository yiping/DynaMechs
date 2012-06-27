#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "BasicGLPane.h"
#include "MainFrame.h"
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

//#include "mosek.h" /* Include the MOSEK definition file. */
#include "TaskSpaceController.h"
#include "humanoidControl.h"
#include "SimulationThread.h"
//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO

 
const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

GLfloat view_mat[4][4];

vector<LinkInfoStruct*> G_robot_linkinfo_list;

dmTimespec tv, last_tv;


int render_count = 0;
int timer_count = 0;
bool runOnce = true;






void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength);

 
void myInit (void); 

class MyApp: public wxApp
{
    virtual bool OnInit();
	virtual int OnExit();
    
    
	wxButton *welcomebutton;
	wxPanel *toolpanel;
	wxButton *saveViewbutton;
	wxButton *applyViewbutton;
	
	wxButton *saveDataButton;
	
	
public:
    
};
 
IMPLEMENT_APP(MyApp)
 
 
bool MyApp::OnInit()
{
	simThread = new SimulationThread();
	simThread->Create();
	simThread->SetPriority(100);
	
    //---------------------------------------------------
	
	mouse = new wxDMGLMouse(); // has to be put in the front

	//wxSpinCtrlDouble * test;

	//---------------------------------------------------

	// Populate GUI
	{
		wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
		wxBoxSizer *toolpanel_sizer = new wxBoxSizer( wxVERTICAL);
		
		//cout << "Frame" << endl;
		frame = new MainFrame(wxT("Hello GL World"), wxPoint(50,50), wxSize(600,400));
		
		toolpanel = new wxPanel((wxFrame*) frame, -1, wxPoint(-1,-1), wxSize(200,400));


		int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
	   
		//cout << "Pane " << endl;
		glPane = new BasicGLPane( (wxFrame*) frame, args, wxSize(400,400));
		
		
		welcomebutton = new wxButton( toolpanel, wxID_OK, wxT("Welcome"));
		saveViewbutton = new wxButton( toolpanel, MainFrame::BUTTON_SaveView, wxT("Save View"));
		applyViewbutton = new wxButton( toolpanel, MainFrame::BUTTON_ApplyView, wxT("Apply View"));
		showCoM = new wxCheckBox(toolpanel,MainFrame::CHECKBOX_ShowCoM,wxT("Show CoM"));
		showGRF = new wxCheckBox(toolpanel,MainFrame::CHECKBOX_ShowGRF,wxT("Show GRF"));
		showNetForceAtGround = new wxCheckBox(toolpanel,MainFrame::CHECKBOX_ShowNetForceAtGround,wxT("Show Net Force (Ground)"));
		showNetForceAtCoM	 = new wxCheckBox(toolpanel,MainFrame::CHECKBOX_ShowNetForceAtCoM,wxT("Show Net Force (CoM)"));
		
		toolpanel_sizer->Add(welcomebutton, 0 ,wxALL | wxALIGN_CENTER,2);
		
		// Camera Options
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Camera Options")),0,wxALL,2);
		toolpanel_sizer->Add(saveViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		toolpanel_sizer->Add(applyViewbutton, 0 ,wxALL | wxALIGN_CENTER,2);
		
		// View Options
		toolpanel_sizer->AddSpacer(15);
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("View Options")),0,wxALL,2);
		toolpanel_sizer->Add(showCoM, 0 ,wxALL  ,2);
		toolpanel_sizer->Add(showGRF, 0 ,wxALL ,2);
		toolpanel_sizer->Add(showNetForceAtGround, 0,wxALL,2 );
		toolpanel_sizer->Add(showNetForceAtCoM, 0,wxALL,2 );
		
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Control Options")),0,wxALL,2);	
		
		wxBoxSlider * CoMControlSlider = new wxBoxSlider(toolpanel,-1,5,15,100);
		toolpanel_sizer->Add(CoMControlSlider,0,wxALL,2);
		
		// Data Logging
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Data Logging")),0,wxALL,2);
		
		logDataCheckBox = new wxCheckBox(toolpanel,MainFrame::CHECKBOX_LogData,wxT("Log Data"));
		saveDataButton = new wxButton(toolpanel,MainFrame::BUTTON_SaveData,wxT("Save Data"));
		toolpanel_sizer->Add(logDataCheckBox,0,wxALL,2);
		toolpanel_sizer->Add(saveDataButton,0,wxALL,2);
		
		
		// Indicators
		toolpanel_sizer->AddSpacer(15);					 
		toolpanel_sizer->Add(new wxStaticText(toolpanel,-1,wxT("Indicators")),0,wxALL,2);
		
		realTimeRatioDisplay = new wxStaticText(toolpanel,-1,wxT("RT Ratio: "));
		toolpanel_sizer->Add(realTimeRatioDisplay,0,wxALL,2);
		
		showCoM->SetValue(true);
		showGRF->SetValue(true);
		
		toolpanel->SetSizer(toolpanel_sizer);

		sizer->Add(glPane, 1, wxEXPAND );
		sizer->Add(toolpanel, 0, wxEXPAND);

		sizer->SetSizeHints(frame);
		frame->SetSizer(sizer);

		frame->SetAutoLayout(true);
	}
	
	// Load DM File
	{
		// load robot stuff
		char *filename = (char *) "humanoid.cfg";
		ifstream cfg_ptr;
		cfg_ptr.open(filename);
		
		Float tmp;
		// Read simulation timing information.
		readConfigParameterLabel(cfg_ptr,"Integration_Stepsize");
		cfg_ptr >> tmp;
		idt = tmp;
		if (idt <= 0.0)
		{
			cerr << "main error: invalid integration stepsize: " << idt << endl;
			exit(3);
		}
		
		readConfigParameterLabel(cfg_ptr,"Control_Stepsize");
		cfg_ptr >> tmp;
		cdt = tmp;
		last_control_time = -2*cdt;
		if (cdt <= 0.0)
		{
			cerr << "main error: invalid control stepsize: " << idt << endl;
			exit(3);
		}
		
		readConfigParameterLabel(cfg_ptr,"Display_Update_Rate");
		cfg_ptr >> render_rate;
		dmGetSysTime(&last_draw_tv);
		
		if (render_rate < 1) render_rate = 1;
		
		// ------------------------------------------------------------------
		// Initialize DynaMechs environment - must occur before any linkage systems
		char env_flname[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr,"Environment_Parameter_File");
		readFilename(cfg_ptr, env_flname);
		dmEnvironment *environment = dmuLoadFile_env(env_flname);
		dmEnvironment::setEnvironment(environment);
		
		// ------------------------------------------------------------------
		// Initialize a DynaMechs linkage system
		char robot_flname[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr,"Robot_Parameter_File");
		readFilename(cfg_ptr, robot_flname);
		G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));
		
		// --------
		// Read in data directory
		char data_dir[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr, "Data_Save_Directory");
		readFilename(cfg_ptr, data_dir);
		dataSaveDirectory = std::string(data_dir);
		
		
		//G_integrator = new dmIntegRK4();
		G_integrator = new dmIntegEuler();
		G_integrator->addSystem(G_robot);
		
		grfInfo.localContacts = 0;
	}
	
	// -- Project specific -- 
	initControl();
	// -----------------------
	
	
	// Scene Init
	{
		cout<<"initilize scene..."<<endl;
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
		
		
		glPane->glInit();
		dmEnvironment::getEnvironment()->drawInit();
		
		frame->Show();
	}

	glPane->restartTimer(render_rate);
	simThread->Run();
    return true;
} 

int MyApp::OnExit() {
	//simThread->requestStop();
	return 1;
}
 

void BasicGLPane::userGraphics()
{
	// Plot User Stuff
	{
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
		
		if (sim_time>7) {
			glPushMatrix();
			glColor4f(1.0, 0.0, 0.0,0.75);
			glTranslatef(2.18, 2, .15);
			glRotated(-90, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			glRotated(180, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			
			glPopMatrix();
		}
		realTimeRatioDisplay->SetLabel(wxString::Format(wxT("RT Ratio: %.2lf"), real_time_ratio));
		
	}
	
}




void BasicGLPane::updateSim(wxTimerEvent & event) {
	
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);
	
	real_time_ratio = (sim_time-last_render_time)/timeDiff(last_draw_tv, tv_now);
	last_render_time = sim_time;
	
	dmGetSysTime(&last_draw_tv);
	
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

	const int detail = 16;
	//Draw Cylinder
	gluCylinder(glPane->quadratic,lineWidth,lineWidth,cylinderLength,detail,detail);
	
	//Draw Cylinder Base
	gluDisk(glPane->quadratic,0,lineWidth,detail,detail);
	
	glTranslatef(0, 0, cylinderLength);
	//Draw Arrowhead
	gluCylinder(glPane->quadratic,headWidth,0.0f,headLength,detail,detail);
	
	//Draw Arrowhead Base
	gluDisk(glPane->quadratic,lineWidth,headWidth,detail,detail);
	
	glEnd();
	glPopMatrix();
	
}

