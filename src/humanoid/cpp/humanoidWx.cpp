#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "wxBoxSlider.h"

//This is a dirty workaround
#undef Success

#include "BasicGLPane.h"
#include "HumanoidMainFrame.h"

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
#include "JumpingStateMachine.h"
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

GLuint neckList, headList;

IMPLEMENT_APP(MyApp)
 
 
bool MyApp::OnInit()
{	
	wxCmdLineParser parser(argc,argv);
	parser.AddOption(wxT("c"), wxT("Config File"));
	parser.Parse();
	

	
	
	simThread = new SimulationThread();
	simThread->Create();
	simThread->SetPriority(100);
	cout << "Created Sim Thread" << endl;
	
	
	// Populate GUI
	{
		//cout << "Frame" << endl;
		frame = new HumanoidMainFrame(wxT("Hello GL World"), wxPoint(50,50), wxSize(600,400));
		
		
	}
	
	cout << "Showing Frame" << endl;
	frame->Show();
	cout << "Frame Shown" << endl;
	
	wxClientDC(frame->glPane);
	frame->glPane->SetCurrent();
	cout << "Context Set" << endl;
	
	// Load DM File
	{
		// load robot stuff
		const char *filename;
		wxString configFileName;
		if(parser.Found(wxT("c"), &configFileName))
		{
			//filename = "human.cfg";//configFileName.mb_str();
			//filename = "humanoid.cfg";//configFileName.mb_str();
			filename = "humanoid_box.cfg";//configFileName.mb_str();
		}
		else {
			filename = (char *) "config.cfg";
		}
		filename = "humanoid_box.cfg";//configFileName.mb_str();
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
		
		cout << "Environment Set" << endl;
		
		// ------------------------------------------------------------------
		// Initialize a DynaMechs linkage system
		char robot_flname[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr,"Robot_Parameter_File");
		readFilename(cfg_ptr, robot_flname);

		cout << "Creating Robot" << endl;
		
		glEnable(GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);
		
		G_robot = dynamic_cast<dmArticulation*>(dmuLoadFile_dm(robot_flname));
		
		humanoid = (HumanoidDataLogger *) new JumpingStateMachine(G_robot);
		//humanoid = (HumanoidDataLogger *) new BalanceDemoStateMachine(G_robot);
		
		cout << "Robot Created" << endl;
		
		
		
		// --------
		// Read in data directory
		char data_dir[FILENAME_SIZE];
		readConfigParameterLabel(cfg_ptr, "Data_Save_Directory");
		readFilename(cfg_ptr, data_dir);
		humanoid->dataSaveDirectory = std::string(data_dir);
		
		
		/*humanoid->grfInfo.localContacts = 2;
		humanoid->grfInfo.pCoPs.resize(2);
		humanoid->grfInfo.pCoPs[0]<<2,2-.09,0;
		humanoid->grfInfo.pCoPs[1]<<2,2+.09,0;
		
		humanoid->grfInfo.fCoPs.resize(2);
		humanoid->grfInfo.fCoPs[0]<<0,-50,100;
		humanoid->grfInfo.fCoPs[1]<<0,50,100;*/
		
		
		
		
		simThread->G_integrator->addSystem(G_robot);
		
		/*cout << "Initial state" << endl;
		Float qInit[31], qdInit[31];
		G_robot->getState(qInit,qdInit);
		for (int i =0; i<31; i++) {
			cout << setprecision(6) << endl;
			cout << qInit[i] << "\t" << qdInit[i] << endl;
		}*/
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
		
		dmEnvironment::getEnvironment()->drawInitSpecial();
		
		frame->glPane->model_loaded = true;
		cout << "Environment Drawn" << endl;
		
	}
	cout << "Starting Timer" << endl;
	frame->glPane->restartTimer();
	
	cout << "Starting Sim Thread" << endl;
	simThread->paused_flag = true;
	simThread->Run();
    return true;
} 

int MyApp::OnExit() {
	cout << "Trying to Exit" << endl;
	//simThread->requestStop();
	return 1;
}
 

void BasicGLPane::userGraphics()
{
	typedef vector<Vector3F > PositionList;
	static deque<PositionList *> traces;
	static deque<FloatVector *> distanceList;
	static deque<int> traceIds;
	
	const int MaxTraceSize = 2000;
	const Float DashSize = .05;
	const int NumTraces = 3;
	
	// Plot User Stuff
	{
		// Traces
		
		if (humanoid->state > 0) {
			PositionList * curr = new PositionList;
			FloatVector  * distances = new FloatVector(NumTraces,0);
			
			curr->resize(NumTraces);
			
			curr->at(0) = humanoid->pCom;
			curr->at(1) = humanoid->pFoot[0];
			curr->at(2) = humanoid->pFoot[1];
			
			if (traceIds.size() > 0) {
				traceIds.push_back(1- (traceIds.back() % 2));
				for (int j=0; j<NumTraces; j++) {
					Vector3F relPos = curr->at(j) - traces.back()->at(j);
					distances->at(j) = distanceList.back()->at(j) + relPos.norm();
				}
			}
			else {
				traceIds.push_back(1);
				for (int j=0; j<NumTraces; j++) {
					distances->at(j) = 0;
				}
			}
			traces.push_back(curr);
			distanceList.push_back(distances);
			
			
			if (traces.size() > MaxTraceSize) {
				delete traces.front();
				delete distanceList.front();
				traces.pop_front();
				traceIds.pop_front();
				distanceList.pop_front();
			}
			
			if (frame->showTraces->IsChecked()) {
			
				// Index Based Drawing
				for (int j=0; j<NumTraces; j++) {
					glBegin(GL_LINES);
					glLineWidth(1.0);
					glColor4f(0.0, 0.0, 0.0,1.0);
					for (int i=1; i<traces.size(); i++) {
						if (traceIds[i]) {
							glVertex3f(traces[i-1]->at(j)(0), traces[i-1]->at(j)(1), traces[i-1]->at(j)(2));
							glVertex3f(traces[i]->at(j)(0), traces[i]->at(j)(1), traces[i]->at(j)(2));
						}

						
					}
					glEnd();
				}
				
				// Distance Based Drawing
				/*for (int j=0; j<NumTraces; j++) {
					glBegin(GL_LINES);
					glLineWidth(2);
					
					
					for (int i=1; i<traces.size(); i++) {
						Float normSegLength = (distanceList[i]->at(j)-distanceList[i-1]->at(j))/DashSize;
						Float tmp, id, idEnd, gamma = 0, gammaEnd = 0;
						bool drawing;
						tmp = (distanceList[i-1]->at(j))/(2*DashSize);
						id = tmp - floor(tmp);
						if (id < .5) {
							drawing = true;
						}
						else {
							drawing = false;
							id-=.5;
						}
						id *=2;
						
						Vector3F startPoint, relPos,pointA, pointB;
						startPoint = traces[i-1]->at(j);
						relPos = traces[i]->at(j) - traces[i-1]->at(j);
						pointA = startPoint;
						
						while (gamma < 1) {
							idEnd = id + (1-gamma)*normSegLength;
							if (idEnd > 1) {
								idEnd = 1;
								gammaEnd = gamma + (idEnd-id)/normSegLength;
							}
							else {
								gammaEnd = 1;
							}
							pointB = startPoint + gammaEnd*relPos;
							
							if (drawing) {
								glColor4f(1.0, 0.1, 0.1,1.0);
								glVertex3f(pointA(0)-.01*relPos(0),pointA(1)-.01*relPos(1),pointA(2)-.01*relPos(2));
								glVertex3f(pointB(0)+.01*relPos(0), pointB(1)+.01*relPos(1), pointB(2)+.01*relPos(2));
							}
							else {
								glColor4f(0.2, 0.2, 0.2,1.0);
								glVertex3f(pointA(0)-.01*relPos(0),pointA(1)-.01*relPos(1),pointA(2)-.01*relPos(2));
								glVertex3f(pointB(0)+.01*relPos(0), pointB(1)+.01*relPos(1), pointB(2)+.01*relPos(2));
							}
							
							gamma = gammaEnd;
							id = idEnd;
							pointA.swap(pointB);
							if (id==1) {
								id = 0;
								drawing = !drawing;
							}
						}
						
					}
					glEnd();
				}*/
			}
			
			
			
			
		}
		
		
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
		
		humanoid->grfInfo.localContacts = 2;
		
		humanoid->grfInfo.pCoPs.resize(2);
		humanoid->grfInfo.pCoPs[0] << .54 + .07*cos(M_PI/4),.54-.07*cos(M_PI/4) ,0;
		humanoid->grfInfo.pCoPs[1] << .54 - .07*cos(M_PI/4),.54+.07*cos(M_PI/4), 0;
		
		humanoid->grfInfo.fCoPs.resize(2);
		humanoid->grfInfo.fCoPs[0] << 10,10,19.2*9.81/2;
		humanoid->grfInfo.fCoPs[1] << 10,10,19.2*9.81/2;
		
		
		
		if (frame->showGRF->IsChecked()) {
			// Draw GRF Info
			for (int i=0; i< humanoid->grfInfo.localContacts; i++) {
				Vector3F footPoint = humanoid->grfInfo.pCoPs[i];
				Vector3F grf = humanoid->grfInfo.fCoPs[i]/forceScale;
				
				drawArrow(footPoint, grf, .005, .01, .03);
				
			}
		}
		
		if (frame->showNetForceAtGround->IsChecked()) {
			//Draw ZMPInfo
			Vector3F footPoint = humanoid->grfInfo.pZMP;
			Vector3F grf = humanoid->grfInfo.fZMP/forceScale;
			drawArrow(footPoint, grf, .005, .01, .03);
		}
		
		/*if (simThread->sim_time>7) {
			glPushMatrix();
			glColor4f(1.0, 0.0, 0.0,0.75);
			glTranslatef(2.18, 2, .15);
			glRotated(-90, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			glRotated(180, 0, 1, 0);
			gluDisk(quadratic,0.0,.05f,32,32);
			
			glPopMatrix();
		}*/
		frame->realTimeRatioDisplay->SetLabel(wxString::Format(wxT("RT Ratio: %.2lf"), real_time_ratio));
		
		
		/*
		Float qBase[7], qdBase[7];
		G_robot->getLink(0)->getState(qBase,qdBase);
		
		Float theta = acos(qBase[3])*2 * 180 / M_PI;

		glPushMatrix();
		glTranslatef(qBase[4], qBase[5], qBase[6]);
		glRotatef(theta, qBase[0], qBase[1], qBase[2]);
		
		
		glTranslatef(0, 0, .4);
		glColor4f(0.700, 0.700, 0.700,.6);
		
		
		Float cylRad = .02;
		Float cylHeight = .04;
		Float headRad = .06;
		
		gluQuadricOrientation(frame->glPane->quadratic,GLU_INSIDE);
		gluCylinder(frame->glPane->quadratic,cylRad,cylRad,cylHeight,16,16);
		
		
		gluQuadricOrientation(frame->glPane->quadratic,GLU_OUTSIDE);
		gluCylinder(frame->glPane->quadratic,cylRad,cylRad,cylHeight,16,16);
		
		
		glTranslatef(0, 0, cylHeight + sqrt(headRad*headRad - cylRad*cylRad));
		gluSphere(quadratic,headRad,32,32);
		
		//glEnd();
		glPopMatrix();*/
		 
		
		/*const Float neckHeight = .03, neckWidth =.04/2, neckDepth = neckWidth;
		const Float headHeight = .12, headDepth = .1/2, headWidth = .1/2;
		
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);
		
		glBegin(GL_QUAD_STRIP);
		
		glVertex3f(neckDepth, -neckWidth, neckHeight);
		glVertex3f(neckDepth, -neckWidth, 0);
		glVertex3f(neckDepth, neckWidth, neckHeight);
		glVertex3f(neckDepth, neckWidth, 0);
		
		glVertex3f(-neckDepth, neckWidth, neckHeight);
		glVertex3f(-neckDepth, neckWidth, 0);
		
		glVertex3f(-neckDepth, -neckWidth, neckHeight);
		glVertex3f(-neckDepth, -neckWidth, 0);
		
		glVertex3f(neckDepth, -neckWidth, neckHeight);
		glVertex3f(neckDepth, -neckWidth, 0);
		
		glEnd();
		
		glTranslatef(0, 0, neckHeight);
		
		glBegin(GL_QUAD_STRIP);
		
		glVertex3f(headDepth, -headWidth, headHeight);
		glVertex3f(headDepth, -headWidth, 0);
		glVertex3f(headDepth, headWidth, headHeight);
		glVertex3f(headDepth, headWidth, 0);
		
		glVertex3f(-headDepth, headWidth, headHeight);
		glVertex3f(-headDepth, headWidth, 0);
		
		glVertex3f(-headDepth, -headWidth, headHeight);
		glVertex3f(-headDepth, -headWidth, 0);
		
		glVertex3f(headDepth, -headWidth, headHeight);
		glVertex3f(headDepth, -headWidth, 0);
		
		glEnd();
		
		glBegin(GL_QUADS);
		
		glVertex3f(-headDepth, headWidth, headHeight);
		glVertex3f(-headDepth, -headWidth, headHeight);
		glVertex3f(headDepth, -headWidth, headHeight);
		glVertex3f(headDepth, headWidth, headHeight);
		
		
		
		glVertex3f(-headDepth, -headWidth, 0);
		glVertex3f(-headDepth, headWidth, 0);
		glVertex3f(headDepth, headWidth, 0);
		glVertex3f(headDepth, -headWidth, 0);
		
		
		
		
		glEnd();*/
		
		
		
		
		
		//glPopMatrix();
		
	}
	
}




void BasicGLPane::updateSim(wxTimerEvent & event) {
	//cout << "Time went off!" << endl;
	dmTimespec tv_now;
	dmGetSysTime(&tv_now);
	
	real_time_ratio = (simThread->sim_time-last_render_time)/timeDiff(last_draw_tv, tv_now);
	last_render_time = simThread->sim_time;
	
	dmGetSysTime(&last_draw_tv);
	
	
	
	//cout << "Asking for repaint" << endl;
	Refresh(); // ask for repaint
	
	
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
	double rX,rY,rZ;
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

	const int detail = 8;

/*	double w, h;
	w = lineWidth;
	h =cylinderLength;
	GLfloat v1tex[8][3];
	v1tex[0][0] = -w; v1tex[0][1] = -w; v1tex[0][2] = 0; 
	v1tex[1][0] = w;  v1tex[1][1] = -w; v1tex[1][2] = 0; 
	v1tex[2][0] = w;  v1tex[2][1] = w; v1tex[2][2] = 0; 
	v1tex[3][0] = -w;  v1tex[3][1] = w; v1tex[3][2] = 0; 

	v1tex[4][0] = -w; v1tex[4][1] = -w; v1tex[4][2] = h; 
	v1tex[5][0] = w;  v1tex[5][1] = -w; v1tex[5][2] = h; 
	v1tex[6][0] = w;  v1tex[6][1] = w; v1tex[6][2] = h; 
	v1tex[7][0] = -w;  v1tex[7][1] = w; v1tex[7][2] = h; 
	
	// quadric | base radius | top radius | height | slices | stacks	
	glBegin(GL_QUADS);
	glVertex3fv(v1tex[0]); glVertex3fv(v1tex[1]);glVertex3fv(v1tex[2]);glVertex3fv(v1tex[3]);
	glVertex3fv(v1tex[0]); glVertex3fv(v1tex[4]);glVertex3fv(v1tex[5]);glVertex3fv(v1tex[1]);
	glVertex3fv(v1tex[1]); glVertex3fv(v1tex[5]);glVertex3fv(v1tex[6]);glVertex3fv(v1tex[2]);
	glVertex3fv(v1tex[2]); glVertex3fv(v1tex[6]);glVertex3fv(v1tex[7]);glVertex3fv(v1tex[3]);
	glVertex3fv(v1tex[3]); glVertex3fv(v1tex[7]);glVertex3fv(v1tex[4]);glVertex3fv(v1tex[0]);
	glEnd();

	glBegin(GL_LINES);
	glVertex3fv(v1tex[0]); glVertex3fv(v1tex[1]);
	glVertex3fv(v1tex[1]); glVertex3fv(v1tex[2]);
	glVertex3fv(v1tex[2]);glVertex3fv(v1tex[3]);
	glVertex3fv(v1tex[3]);glVertex3fv(v1tex[0]);

	glVertex3fv(v1tex[0]); glVertex3fv(v1tex[4]);
	glVertex3fv(v1tex[1]);glVertex3fv(v1tex[5]);

	glVertex3fv(v1tex[2]); glVertex3fv(v1tex[6]);
	glVertex3fv(v1tex[3]); glVertex3fv(v1tex[7]);

	glVertex3fv(v1tex[4]); glVertex3fv(v1tex[5]);
	glVertex3fv(v1tex[5]); glVertex3fv(v1tex[6]);
	glVertex3fv(v1tex[6]);glVertex3fv(v1tex[7]);
	glVertex3fv(v1tex[7]);glVertex3fv(v1tex[4]);

	glEnd();*/

	//glPolygonMode(GL_FRONT, GL_FILL);
	//Draw Cylinder
	gluCylinder(frame->glPane->quadratic,lineWidth,lineWidth,cylinderLength,detail,detail);
	
	//Draw Cylinder Base
	glRotated(180, 1, 0, 0);
	gluDisk(frame->glPane->quadratic,0,lineWidth,detail,detail);
	glRotated(180, 1, 0, 0);
	
	glTranslatef(0, 0, cylinderLength);
	//Draw Arrowhead
	gluCylinder(frame->glPane->quadratic,headWidth,0.0f,headLength,detail,detail);
	
	glRotated(180, 1, 0, 0);
	//Draw Arrowhead Base
	gluDisk(frame->glPane->quadratic,lineWidth,headWidth,detail,detail);


	glPopMatrix();


	//again!
	glPushMatrix();
	glTranslatef(location(0), location(1), location(2));
	glRotated(RADTODEG * theta, rX, rY, rZ);
	
	cylinderLength = direction.norm();
	if (cylinderLength > headLength) {
		cylinderLength -= headLength;
	}
	else {
		headLength = cylinderLength ;
		cylinderLength = 0;
	}

	glPolygonMode(GL_FRONT, GL_LINE);
	gluCylinder(frame->glPane->quadratic,lineWidth,lineWidth,cylinderLength,detail,detail);
	gluDisk(frame->glPane->quadratic,0,lineWidth,detail,detail);
	glTranslatef(0, 0, cylinderLength);
	gluCylinder(frame->glPane->quadratic,headWidth,0.0f,headLength,detail,detail);
	gluDisk(frame->glPane->quadratic,lineWidth,headWidth,detail,detail);
	glLineWidth(1);
	glPolygonMode(GL_FRONT, GL_FILL);
	glPopMatrix();
}

