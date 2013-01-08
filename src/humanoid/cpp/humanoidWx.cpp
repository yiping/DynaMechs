#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "wxBoxSlider.h"

//This is a dirty workaround
#undef Success

#include "BasicGLPane.h"
#include "MainFrame.h"

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
#include "wx/cmdline.h"
#include "wx/dcclient.h"
#include "GlobalDefines.h"

//#include "mosek.h" /* Include the MOSEK definition file. */
#include "TaskSpaceController.h"
#include "HumanoidController.h"
#include "SimulationThread.h"
#include "BalanceDemoStateMachine.h"
//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO

 
const float RADTODEG = (float)(180.0/M_PI);    // M_PI is defined in math.h
const float DEGTORAD = (float)(M_PI/180.0);

void drawArrow(Vector3F & location, Vector3F & direction,double lineWidth, double headWidth, double headLength);

 
void myInit (void); 

class MyApp: public wxApp
{
    virtual bool OnInit();
	virtual int OnExit();
	
public:
    
};
 
IMPLEMENT_APP(MyApp)
 
 
bool MyApp::OnInit()
{	
	wxCmdLineParser parser(argc,argv);
	parser.AddOption(wxT("c"), wxT("Config File"));
	parser.Parse();
	

	
	simThread = new SimulationThread();
	simThread->Create();
	simThread->SetPriority(100);

	// Populate GUI
	{
		//cout << "Frame" << endl;
		frame = new MainFrame(wxT("Hello GL World"), wxPoint(50,50), wxSize(600,400));
		
		
	}
	
	frame->Show();
	
	wxClientDC(frame->glPane);
	frame->glPane->SetCurrent();
	
	
	// Load DM File
	{
		// load robot stuff
		const char *filename;
		wxString configFileName;
		if(parser.Found(wxT("c"), &configFileName))
		{
			filename = "humanoid.cfg";//configFileName.mb_str();
		}
		else {
			filename = (char *) "config.cfg";
		}
		cout<<filename<<endl;
		
		ifstream cfg_ptr;
		cfg_ptr.open(filename);
		
		Float tmp;
		// Read simulation timing information.
		readConfigParameterLabel(cfg_ptr,"Integration_Stepsize");
		cfg_ptr >> tmp;
		simThread->idt = tmp;
		if (simThread->idt <= 0.0)
		{
			cerr << "main error: invalid integration stepsize: " << simThread->idt << endl;
			exit(3);
		}
		
		readConfigParameterLabel(cfg_ptr,"Control_Stepsize");
		cfg_ptr >> tmp;
		simThread->cdt = tmp;
		simThread->last_control_time = -2*simThread->cdt;
		if (simThread->cdt <= 0.0)
		{
			cerr << "main error: invalid control stepsize: " << simThread->cdt << endl;
			exit(3);
		}
		
		readConfigParameterLabel(cfg_ptr,"Display_Update_Rate");
		cfg_ptr >> frame->glPane->render_rate;
		
		
		if (frame->glPane->render_rate < 1) frame->glPane->render_rate = 1;
		
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
		humanoid = (HumanoidDataLogger *) new BalanceDemoStateMachine(G_robot);
		
		// --------
		// Read in data directory
		char data_dir[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr, "Data_Save_Directory");
		readFilename(cfg_ptr, data_dir);
		humanoid->dataSaveDirectory = std::string(data_dir);
		
		
		simThread->G_integrator->addSystem(G_robot);
		
		grfInfo.localContacts = 0;
	}
	
	
	
	// -- Project specific -- 
	//initControl();
	// -----------------------
	{
		cout<<"initilize scene..."<<endl;
		
		frame->glPane->camera->setRadius(8.0);
		frame->glPane->camera->setCOI(3.0, 3.0, 0.0);
		frame->glPane->camera->setTranslationScale(0.02f);
		
		frame->glPane->glInit();
		
		dmEnvironment::getEnvironment()->drawInit();
		
		frame->glPane->model_loaded = true;
	}
	frame->glPane->restartTimer();
	simThread->Run();
    return true;
} 

int MyApp::OnExit() {
	//simThread->requestStop();
	return 1;
}
 

void BasicGLPane::userGraphics()
{
	
	frame->glPane->camera->setCOI(ComPos[0], ComPos[1], ComPos[2]);
	
	// Plot User Stuff
	{
		if (frame->showCoM->IsChecked()) {
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
		
		if (frame->showGRF->IsChecked()) {
			// Draw GRF Info
			for (int i=0; i< grfInfo.localContacts; i++) {
				Vector3F footPoint = grfInfo.pCoPs[i];
				Vector3F grf = grfInfo.fCoPs[i]/forceScale;
				
				drawArrow(footPoint, grf, .005, .01, .03);
				
			}
		}
		
		if (frame->showNetForceAtGround->IsChecked()) {
			//Draw ZMPInfo
			Vector3F footPoint = grfInfo.pZMP;
			Vector3F grf = grfInfo.fZMP/forceScale;
			drawArrow(footPoint, grf, .005, .01, .03);
		}
		
		if (simThread->sim_time>7) {
			glPushMatrix();
			glColor4f(1.0, 0.0, 0.0,0.75);
			glTranslatef(2.18, 2, .15);
			glRotated(-90, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			glRotated(180, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			
			glPopMatrix();
		}
		frame->realTimeRatioDisplay->SetLabel(wxString::Format(wxT("RT Ratio: %.2lf"), real_time_ratio));
		
	}
	
}




void BasicGLPane::updateSim(wxTimerEvent & event) {
	
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);
	
	real_time_ratio = (simThread->sim_time-last_render_time)/timeDiff(last_draw_tv, tv_now);
	last_render_time = simThread->sim_time;
	
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
	
	double rtime = timeDiff(first_tv, tv_now);
	double update_time = timeDiff(update_tv, tv_now);
	
	if (update_time > 2.5)
	{
		timer_count ++;
		cerr << "time/real_time: " << simThread->sim_time << '/' << rtime
		<< "  frame_rate: " << (double) timer_count/rtime << endl;
		dmGetSysTime(&update_tv);
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
	gluCylinder(frame->glPane->quadratic,lineWidth,lineWidth,cylinderLength,detail,detail);
	
	//Draw Cylinder Base
	gluDisk(frame->glPane->quadratic,0,lineWidth,detail,detail);
	
	glTranslatef(0, 0, cylinderLength);
	//Draw Arrowhead
	gluCylinder(frame->glPane->quadratic,headWidth,0.0f,headLength,detail,detail);
	
	//Draw Arrowhead Base
	gluDisk(frame->glPane->quadratic,lineWidth,headWidth,detail,detail);
	
	glEnd();
	glPopMatrix();
	
}

