/*
 *****************************************************************************
 *     File: global.cpp
 *   Author: Yiping Liu
 *  Created: 17 Apr 2012
 *  Summary: global variable definitions
 *****************************************************************************/
#include "global.h"   

extern const float RADTODEG = (float)(180.0/M_PI);    // definition, M_PI is defined in math.h
extern const float DEGTORAD = (float)(M_PI/180.0);    

dmGLMouse *mouse;
dmGLPolarCamera_zup *camera;
GLfloat view_mat[4][4];

Float idt;
Float sim_time=0.0;
Float rtime=0.0;

DataRecVector MyVec;
Vector6F err = Vector6F::Zero();
Vector6F desired_torso_acc = Vector6F::Zero();
Float tr[10][1];
Vector3F p_ZMP_ICS = Vector3F::Zero();
Vector3F actual_p_ZMP_ICS = Vector3F::Zero();


bool outputOnce = true;
bool pc_ini = true;
bool IsWireframe = false;

bool paused_flag = true;
dmArticulation *G_robot;
//dmIntegRK4 *G_integrator;
dmIntegEuler *G_integrator;

vector<LinkInfoStruct*> G_robot_linkinfo_list;

dmTimespec tv, last_tv;

int render_rate;
int render_count = 0;
int timer_count = 0;

int control_count = 0;


///

Matrix3F Rot_rf_ICS; // right foot to ICS
Matrix3F Rot_lf_ICS; // left foot to ICS
Matrix3F Rot_rk_ICS; // right knee to ICS
Matrix3F Rot_lk_ICS; // left knee to ICS
Matrix3F Rot_t_ICS;  // torso to ICS

Vector3F p_rk_ICS;
Vector3F p_lk_ICS;
Vector3F p_rf_ICS = Vector3F::Zero();
Vector3F p_lf_ICS = Vector3F::Zero();
Vector3F p_t_ICS;
Vector3F prf(0.25,0, 0); // rf to rk
Vector3F plf(0.25,0, 0); // lf to lk

Vector6F TorsoVel_curr;
Vector6F RkneeVel_curr;
Vector6F LkneeVel_curr;


Vector6F f_torso_ICS;
Vector6F f_torso;
Vector6F GRF_rf = Vector6F::Zero();
Vector6F GRF_lf = Vector6F::Zero();
Float pGain = 30;
Float dGain = 5;
Float pGain_lh = 30;
Float dGain_lh = 5;

dmRigidBody *Rfoot;
dmContactModel *Rcontact;    //
dmRigidBody *Lfoot;
dmContactModel *Lcontact;    //

Vector3F CoM_pos_ICS = Vector3F::Zero();
vector<Matrix3F> tRotVec;
vector<Vector3F> tpVec;

bool doubleSupportOn = false;
