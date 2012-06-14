/*
 *  humanoidControl.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/5/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlobalDefines.h"

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

void HumanoidControl(ControlInfo &);
void initControl();

