// **************************************************************************
// FILENAME: LegLink.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     1993 June 14
// **************************************************************************


#include "LegLink.hpp"


// **************************************************************************
// LegLink Class Member Function
// **************************************************************************
LegLink::LegLink()
{
  // LegLink class
  joint_no         = 0;
  link_no          = 0;
  link_length      =  37.5;
  joint_angle_pos  =   0.0;
  joint_angle_vel  =   0.0;
  joint_angle_acl  =   0.0;
  min_joint_angle  = -60.0;
  max_joint_angle  =  60.0;
  joint_limit_flag = 0;
  B_joint_pos = Vector();
  W_joint_pos = Vector();
}


// **************************************************************************
LegLink::LegLink(int j_no,
                 double l_length,
                 double j_angle,
                 double min_ang,
                 double max_ang)
{
  // LegLink class
  joint_no         = j_no;
  link_no          = j_no;
  link_length      = l_length;
  joint_angle_pos  = j_angle;
  joint_angle_vel  = 0.0;
  joint_angle_acl  = 0.0;
  min_joint_angle  = min_ang;
  max_joint_angle  = max_ang;
  joint_limit_flag = 0;
  B_joint_pos = Vector();
  W_joint_pos = Vector();
}


// EOF
