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
#include "RunningStateMachine.h"
#include "CoordinatedCubicSpline.h"

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
	

	/*CoordinatedCubicSpline cs;
	MatrixXF pts(2,7);
	VectorXF times(7);
	VectorXF vInit(2);
	VectorXF vFinal(2);
	
	pts << -0.268065, -0.858982, -0.945076, -0.405032, 0.170231, 0.577220, 0.268065, 0.000000, 0.181749, 0.450000, 0.354752, 0.135097, 0.061231, 0.000000;
	times << 0.000000, 0.071430, 0.142859, 0.227157, 0.311454, 0.382884, 0.454313;
	
	vInit << -3.5,0;
	vFinal << -3.5,0;
	cs.computeCoefficients(times, pts, vInit, vFinal);
	
	VectorXF pos(2), vel(2), acc(2);
	
	FILE * splineData = fopen("testSpline.dat","w");
	for (Float t = 0; t<.5; t+=.001) {
		
		cs.eval(t, pos, vel, acc);
		fprintf(splineData,"%f %f\n",pos(0),pos(1));
	}
	fclose(splineData);
	exit(1);
	
	return true;*/
	
	
	
	
	
	
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
		environment = dmuLoadFile_env(env_flname);
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
		
		humanoid = (HumanoidDataLogger *) new RunningStateMachine(G_robot);
		//humanoid = (HumanoidDataLogger *) new JumpingStateMachine(G_robot);
		//humanoid = (HumanoidDataLogger *) new BalanceDemoStateMachine(G_robot);
		
		cout << "Robot Created" << endl;
		
		ball = dynamic_cast<dmArticulation*>(dmuLoadFile_dm("ball.dm"));
		
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
		simThread->G_integrator->addSystem(ball);
		
		
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
		
		dmEnvironment::getEnvironment()->drawInit();
		
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
 

void drawPortion(float starta, float enda, float startb, float endb, float r, float slices, float color)
{
	glColor4f(1.0*color, 1.0*color, 1.0*color,1.0);
	
	
	float da = (enda-starta)/slices;
	float db = (endb-startb)/slices;
	for(float a = starta; a< enda;a+=da)
	{
		for(float b=startb;b<endb;b+=db)
		{
			glBegin(GL_QUADS);
			float a1 = a;
			float a2 = a+da;
			float b1 = b;
			float b2 = b+db;
			
			glVertex3f(r*sin(b1)*cos(a2), r*sin(b1)*sin(a2), r*cos(b1));
			glVertex3f(r*sin(b1)*cos(a1), r*sin(b1)*sin(a1), r*cos(b1));
			glVertex3f(r*sin(b2)*cos(a1), r*sin(b2)*sin(a1), r*cos(b2));
			glVertex3f(r*sin(b2)*cos(a2), r*sin(b2)*sin(a2), r*cos(b2));
			
			glEnd();
		}
	}
	
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
	
	
	
	//if (simThread->sim_time < 7) {
		frame->glPane->camera->setCOI(ComPos[0], ComPos[1], .8);
	//}
	//else {
	//	
	//	frame->glPane->camera->setCOI(ComPos[0]-(simThread->sim_time-7)*1.5, 2, .5);
	//}

	
	
	// Plot User Stuff
	{
		// Traces
		
		if (humanoid->state > 0) {
			PositionList * curr = new PositionList;
			FloatVector  * distances = new FloatVector(NumTraces,0);
			
			curr->resize(NumTraces);
			
			curr->at(0)(0) = ComPos[0];
			curr->at(0)(1) = ComPos[1];
			curr->at(0)(2) = ComPos[2];
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
		
		Float qBase[7], qdBase[7];
		humanoid->artic->getLink(0)->getState(qBase,qdBase);
		Float theta = acos(qBase[3])*2 * 180 / M_PI;
		
		glPushMatrix();
		glTranslatef(qBase[4], qBase[5], qBase[6]);
		glRotatef(theta, qBase[0], qBase[1], qBase[2]);
		SpatialVector forceVec;
		( (dmRigidBody *) humanoid->artic->getLink(0))->getExternalForce(forceVec);
		
		glColor4f(1.0, 0.0, 0.0, 1.0);
		Vector3F myVec,zero;
		zero.setZero();
		myVec << 0, forceVec[4]/1600,0;
		drawArrow(zero, myVec, .03, .05, .1);
		glPopMatrix();
		
		Vector3F CoM;
		CoM(0) = ComPos[0];
		CoM(1) = ComPos[1];
		CoM(2) = ComPos[2];
		
		Vector3F anchor;
		anchor = ((RunningStateMachine *) humanoid)->SLIP.anchor;
		
		
		if (frame->showCoM->IsChecked()) {
			// Draw COM Info
			/*glBegin(GL_LINES);
			glColor4f(0.0, 0.0, 1.0,1.0);
			glVertex3f(ComDes[0], ComDes[1], ComDes[2]);
			glVertex3f(ComDes[0], ComDes[1], 0      );
			glEnd();
			
			glBegin(GL_LINES);
			glColor4f(1.0, 0.0, 0.0,1.0);
			glVertex3f(ComPos[0], ComPos[1], ComPos[2]);
			glVertex3f(ComPos[0], ComPos[1], 0      );
			glEnd();*/
			
			glPushMatrix();
			glTranslatef(ComPos[0],ComPos[1],ComPos[2]); 
			//gluSphere(quadratic,.03f,32,32);
			{
				
				float slices = 100;
				float r = .07/1.3;
				
				drawPortion(0, M_PI/2, 0, M_PI/2, r, slices, 1);
				drawPortion(M_PI/2, M_PI, 0, M_PI/2, r, slices, 0);
				drawPortion(M_PI, M_PI*3./2., 0, M_PI/2, r, slices, 1);
				drawPortion(M_PI*3./2., M_PI*2, 0, M_PI/2, r, slices, 0);
				
				drawPortion(0, M_PI/2, M_PI/2, M_PI, r, slices, 0);
				drawPortion(M_PI/2, M_PI, M_PI/2, M_PI, r, slices, 1);
				drawPortion(M_PI, M_PI*3./2., M_PI/2, M_PI, r, slices, 0);
				drawPortion(M_PI*3./2., M_PI*2, M_PI/2, M_PI, r, slices, 1);
				
			}
			
			glPopMatrix();
			
			{
				glColor4f(0.0, 0.0, 0.0,1.0);
				
				glPushMatrix();
				glTranslatef(anchor(0), anchor(1), anchor(2));
				Vector3F zero,dir;
				zero.setZero();
				dir = CoM-anchor;
				
				if(false)
				{
					Vector3F normDir;
					normDir(0) = 0;
					normDir(1) = dir(2);
					normDir(2) = -dir(1);
					
					
					Float lineWid = .001;
					int numZigs = 6;
					Float zigWidth = .15;
					Float zigHeight =.15*tan(25*M_PI/180) * dir.norm()/.97;
					
					Float otherLenth = (dir.norm()-numZigs*zigHeight)/2;
					
					dir.normalize();
					normDir.normalize();
					
					
					Vector3F zag = -zigWidth*normDir+zigHeight*dir;
					Vector3F zig = zigWidth*normDir+zigHeight*dir;
					Vector3F shortZig = zig/2;
					
					Vector3F pos;
					pos.setZero();
					
					Vector3F otherDist = dir*otherLenth;
					
					drawArrow(zero, otherDist, lineWid, 0, 0);
					pos+=otherDist;
					drawArrow(pos, shortZig, lineWid, 0, 0);
					pos+=shortZig;
					for(int i=1;i<numZigs;i++)
					{
						drawArrow(pos, zag, lineWid, 0, 0);
						pos+=zag;
						i++;
						if (i<numZigs) {
							drawArrow(pos, zig, lineWid, 0, 0);
							pos+=zig;
						}
					}
					drawArrow(pos, shortZig, lineWid, 0, 0);
					pos+=shortZig;
					
					drawArrow(pos, otherDist, lineWid, 0, 0);
					pos+=otherDist;
				}
				else {
				
					Vector3F normDir1,normDir2;
					normDir1(0) = 0;
					normDir1(1) = dir(2);
					normDir1(2) = -dir(1);
					
					normDir2(0) = dir(2);
					normDir2(1) = 0;
					normDir2(2) = -dir(0);
					
					
					
					Float lineWid = .003;
					int numZigs = 6;
					int numSlices = 50;
					
					Float diam = .115;
					Float zigWidth = diam;
					Float zigHeight =diam*tan(35*M_PI/180) * dir.norm()/.97;
					
					Float otherLenth = (dir.norm()-(numZigs+.5)*zigHeight)/2;
					
					dir.normalize();
					normDir1.normalize();
					normDir2.normalize();
					
					
					Vector3F zag = -zigWidth*normDir1+zigHeight*dir;
					Vector3F zig = zigWidth*normDir1+zigHeight*dir;
					Vector3F shortZig = zig/2;
					
					Vector3F pos;
					pos.setZero();
					
					Vector3F otherDist = dir*otherLenth;
					
					//glColor4f(.5,.1,.1, .8);
					glColor4f(1.,0.,0., .8);
					drawArrow(zero, otherDist, lineWid, 0, 0);
					pos+=otherDist;
					
					{
						Float tStart = 3*M_PI/2;
						Float tEnd   = 2*M_PI;
						Float dt = (tEnd-tStart)/numSlices;
						Float rStart = 0;
						Float rEnd   = zigWidth/2.;
						Float dr  = (rEnd-rStart)/numSlices;
						
						Float r = rStart;
						for(Float t= tStart ; t<tEnd ; t+=dt)
						{
							Vector3F posA = (normDir1*cos(t)+normDir2*sin(t))*r;
							Vector3F posB = (normDir1*cos(t+dt)+normDir2*sin(t+dt))*(r+dr);
							posB+= zigHeight*dir/numSlices/2;
							Vector3F rel = posB-posA;
							
							drawArrow(pos, rel, lineWid, 0, 0);
							
							pos+=rel;
							r+=dr;
						}
					}
					
					//drawArrow(pos, shortZig, lineWid, 0, 0);
					//pos+=shortZig;
					for(int i=0;i<numZigs;i++)
					{
						Float tEnd = 2*M_PI;
						if (i==(numZigs-1)) {
							tEnd=M_PI;
						}
						Float dt = 2*M_PI/numSlices;
						for(Float t=0; t<tEnd ; t+=dt)
						{
							Vector3F posA = (normDir1*cos(t)+normDir2*sin(t))*zigWidth/2;
							Vector3F posB = (normDir1*cos(t+dt)+normDir2*sin(t+dt))*zigWidth/2;
							posB+= zigHeight*dir/numSlices;
							Vector3F rel = posB-posA;
							
							drawArrow(pos, rel, lineWid, 0, 0);
							pos+=rel;
						}
					}
							
					//drawArrow(pos, shortZig, lineWid, 0, 0);
					//pos+=shortZig;
					
					{
						Float tStart = M_PI;
						Float tEnd   = 3*M_PI/2;
						Float dt = (tEnd-tStart)/numSlices;
						Float rStart = zigWidth/2;
						Float rEnd   = 0;
						Float dr  = (rEnd-rStart)/numSlices;
						
						Float r = rStart;
						for(Float t= tStart ; t<tEnd ; t+=dt)
						{
							Vector3F posA = (normDir1*cos(t)+normDir2*sin(t))*r;
							Vector3F posB = (normDir1*cos(t+dt)+normDir2*sin(t+dt))*(r+dr);
							posB+= zigHeight*dir/numSlices/2;
							Vector3F rel = posB-posA;
							
							drawArrow(pos, rel, lineWid, 0, 0);
							
							pos+=rel;
							r+=dr;
						}
					}
					
					
					
					drawArrow(pos, otherDist, lineWid, 0, 0);
					pos+=otherDist;
					
				}

				
				glPopMatrix();
			}
			
			
			/*Vector3F vDraw = humanoid->vCom/6.;
			glColor4f(1, 0, 0, 1.0);
			drawArrow(humanoid->pCom, vDraw, .001, .02, .05);
			
			vDraw = humanoid->vComDes/6.;
			//vDraw << ((RunningStateMachine *) humanoid)->vDesDisplay/6.,0,0;
			
			glColor4f(0, 0, 1, 1.0);
			drawArrow(humanoid->pCom, vDraw, .001, .02, .05);*/
			
			
			//glTranslatef(-ComPos[0],-ComPos[1],-ComPos[2]);
		}
		
		
		const Float forceScale = 250*5;
		glColor4f(0.0, 0.0, 0.0,0.75);
		
		/*humanoid->grfInfo.localContacts = 2;
		
		humanoid->grfInfo.pCoPs.resize(2);
		humanoid->grfInfo.pCoPs[0] << .54 + .07*cos(M_PI/4),.54-.07*cos(M_PI/4) ,0;
		humanoid->grfInfo.pCoPs[1] << .54 - .07*cos(M_PI/4),.54+.07*cos(M_PI/4), 0;
		
		humanoid->grfInfo.fCoPs.resize(2);
		humanoid->grfInfo.fCoPs[0] << 10,10,19.2*9.81/2;
		humanoid->grfInfo.fCoPs[1] << 10,10,19.2*9.81/2;*/
		
		
		
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
		
		
		
		
		static dmTimespec thisRenderTime, prevRenderTime;
		dmGetSysTime(&thisRenderTime);
		Float renderHz = 1/timeDiff(prevRenderTime, thisRenderTime);
		dmGetSysTime(&prevRenderTime);
		
		frame->renderHzDisplay->SetLabel(wxString::Format(wxT("Render Hz: %.2lf"), renderHz));
		
		frame->desiredSpeedDisplay->SetLabel(wxString::Format(wxT("vDes: %.2f"),((RunningStateMachine *) humanoid)->vDesDisplay));
		frame->actualSpeedDisplay->SetLabel(wxString::Format(wxT("vAct: %.2f"),((RunningStateMachine *) humanoid)->vActDisplay));
		
		
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
		//cerr << "time/real_time: " << simThread->sim_time << '/' << rtime
		//<< "  frame_rate: " << (double) timer_count/rtime << endl;
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

