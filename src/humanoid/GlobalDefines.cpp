/*
 *  GlobalDefines.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "dmArticulation.hpp"
#include "GlobalTypes.h"
#include "wx/wx.h"
#include <dmIntegEuler.hpp>
#include <dmTime.h>
#include <wxDMGLPolarCamera_zup.hpp>
#include <wxDMGLMouse.hpp>
#include "BasicGLPane.h"

dmArticulation *G_robot;
volatile Float sim_time=0.0;
volatile Float ComPos[3];
volatile Float ComDes[3];
GRFInfo grfInfo;

wxDMGLMouse *mouse;
wxDMGLPolarCamera_zup *camera;
BasicGLPane * glPane;


wxCheckBox * showCoM, * showGRF, * showNetForceAtGround, * showNetForceAtCoM;
wxStaticText * realTimeRatioDisplay;

Float idt, cdt, last_control_time, last_render_time, real_time_ratio = 1;
Float rtime=0.0;
dmTimespec last_draw_tv;
dmIntegEuler *G_integrator;

bool IsWireframe = false;
bool paused_flag = true;