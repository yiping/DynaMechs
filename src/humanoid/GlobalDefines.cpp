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


dmArticulation *G_robot;
volatile Float sim_time=0.0;
volatile Float ComPos[3];
volatile Float ComDes[3];
GRFInfo grfInfo;

wxCheckBox * showCoM, * showGRF, * showNetForceAtGround, * showNetForceAtCoM;