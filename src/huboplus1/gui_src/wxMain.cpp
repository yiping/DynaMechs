//  wxMain.cpp
//  Nov 26, 2012
//  YL

#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"


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
#include <dmContactSystem.hpp>
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
#include "globalVariables.h"




#include "SimulationThread.h"

//#define KURMET_DEBUG

#define OUTPUT_DEBUG_INFO

 




 
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
	parser.AddOption(wxT("c"), wxT("config-file"));
	parser.Parse();
	

	
	simThread = new SimulationThread();
	simThread->Create();
	simThread->SetPriority(100);

	// Populate GUI
	{
		//cout << "Frame" << endl;
		frame = new MainFrame(wxT("Huboplus Control Test 1"), wxPoint(50,50), wxSize(600,400));
		
		
	}
	
	frame->Show();
	
	wxClientDC(frame->glPane);
	frame->glPane->SetCurrent();
	
	
	// Load configurations
	{
		// load robot stuff
		const char *filename;
		wxString configFileName;
		if(parser.Found(wxT("c"), &configFileName))
		{
			//filename = configFileName.mb_str();  //not working on Linux
			char cstring[256];
			strncpy(cstring, (const char*)configFileName.mb_str(wxConvUTF8), 255);
			filename = (const char *) cstring;
		}
		else 
		{
			cout<< "Error: cfg file not supplied !\n\n";
			exit(1);
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

		cout<<"Robot parameter file: [\033[1;34m"<< robot_flname<< "\033[0m]"<<endl;
		G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));
		cout << "Robot created" << endl;

		huboCtrl = (HuboController *) new HuboBalanceController(G_robot);
		cout << "Robot controller created" << endl;

		// set status bar
    	wxString s;
    	s.Printf(wxT("idt = %1.1e s"), simThread->idt);
		frame->SetStatusText(s,1);
		s.Printf(_T("cdt = %f s" ), simThread->cdt);
		frame->SetStatusText(s,2);
		s.Printf(_T("refresh every %.2f ms"), 1000/(frame->glPane->render_rate) );
		frame->SetStatusText(s,3);
		
		// ------------------------
		// Read in data directory
		char data_dir[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr, "Data_Save_Directory");
		readFilename(cfg_ptr, data_dir);
		// std::string(data_dir)

		// ...


		cout<<"Add robot to integrator "<<endl;
		simThread->G_integrator->addSystem(G_robot);


	}
	

	// -----------------------
	{
		cout<<"Initilize scene..."<<endl;
		
		frame->glPane->camera->setRadius(8.0);
		frame->glPane->camera->setCOI(3.0, 3.0, 0.0);
		frame->glPane->camera->setTranslationScale(0.02f);
		
		frame->glPane->glInit();
		
		dmEnvironment::getEnvironment()->drawInit();
		
		frame->glPane->model_loaded = true;

	}

	cout << "Restart Timer" << endl;
	frame->glPane->restartTimer();

	cout << "Start Sim Thread" << endl;
	simThread->Run();

    return true;
} 

// OnExit is called after destroying all application windows and controls
int MyApp::OnExit() 
{
	cout<<"Trying to Exit ... "<<endl;
	//Put any application-wide cleanup code here
	
	delete G_robot;
	return 1;
}
 


