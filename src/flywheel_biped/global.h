/*
 *****************************************************************************
 *     File: global.h
 *   Author: Yiping Liu
 *  Created: 17 Apr 2012
 *  Summary: includes, typedefs, global variable declarations 
 *****************************************************************************/

#ifndef _FLYWHEEL_BIPED_GLOBAL_H
#define _FLYWHEEL_BIPED_GLOBAL_H

#include <dm.h>            // DynaMechs typedefs, globals, etc.
#include <dmu.h>
#include <dmGL.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <dmTime.h>
#include <dmGLMouse.hpp>
#include <dmGLPolarCamera_zup.hpp>

#include <dmSystem.hpp>      // DynaMechs simulation code.
#include <dmArticulation.hpp>
#include <dmLink.hpp>
#include <dmIntegRK4.hpp>
#include <dmIntegEuler.hpp>
#include <dmEnvironment.hpp>
#include <dmMDHLink.hpp>
#include <dmMobileBaseLink.hpp>
#include <dmContactModel.hpp>
#include <dmRevoluteLink.hpp>
#include <dmRigidBody.hpp>












struct DataRecord{  
	Float sim_time;
	Float q[10];//system states
	Float qd[10];
	Float desired_torso_acc[6];// desired torso acc
	Float JointTorque[10];// joint torques 
	Float ZMPx_ICS;
	Float actual_ZMPx_ICS;
	Float p_Rf_ICS[3];
	Float p_Lf_ICS[3];
	Float CoMx_ICS;
};

typedef vector<DataRecord *> DataRecVector; 




extern const float RADTODEG ;  //declaration 
extern const float DEGTORAD ;


extern dmGLMouse *mouse;
extern dmGLPolarCamera_zup *camera;
extern GLfloat view_mat[4][4];

extern Float idt;
extern Float sim_time;
extern Float rtime;

extern DataRecVector MyVec;
extern Vector6F desired_torso_acc;
extern Float tr[10][1];
extern Vector3F p_ZMP_ICS;
extern Vector3F actual_p_ZMP_ICS;



extern bool outputOnce;
extern bool pc_ini;
extern bool IsWireframe;

extern bool paused_flag;
extern dmArticulation *G_robot;
//extern dmIntegRK4 *G_integrator;
extern dmIntegEuler *G_integrator;

extern vector<LinkInfoStruct*> G_robot_linkinfo_list;

extern dmTimespec tv, last_tv;

extern int render_rate;
extern int render_count;
extern int timer_count;

extern int control_count;

///

extern Matrix3F Rot_rf_ICS;
extern Matrix3F Rot_lf_ICS;
extern Matrix3F Rot_rk_ICS;
extern Matrix3F Rot_lk_ICS;
extern Matrix3F Rot_t_ICS;

extern Vector3F p_rk_ICS;
extern Vector3F p_lk_ICS;
extern Vector3F p_rf_ICS;
extern Vector3F p_lf_ICS;
extern Vector3F p_t_ICS;
extern Vector3F prf;
extern Vector3F plf;

extern Vector6F TorsoVel_curr;
extern Vector6F RkneeVel_curr;
extern Vector6F LkneeVel_curr;


extern Vector6F f_torso_ICS;
extern Vector6F f_torso;
extern Vector6F GRF_rf;
extern Float pGain;
extern Float dGain;

extern dmRigidBody *Rfoot;
extern dmContactModel *Rcontact;    //

extern Vector3F CoM_pos_ICS;
extern vector<Matrix3F> tRotVec;
extern vector<Vector3F> tpVec;

#endif
