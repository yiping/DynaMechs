// *************************************************************************
// FILENAME: GaitAlgorithm.cpp
// AUTHOR:   Kenji Suzuki
// COMMENT:
// UPDATE:   September 5, 1993 by Kenji Suzuki
// *************************************************************************

#include "GaitAlgorithm.h"

#define RADTODEG 57.29577951

// **************************************************************************
// FUNCTION: gait_algorithm()
// **************************************************************************
GAIT_DLL_API void gait_algorithm(double, // time,
                                 double sampling_time, Robot &new_robot,
                                 float motion_command[3])
{
  ////////////////////////////////////
  // Generating body motion command //
  ////////////////////////////////////

  // new motion_command = [vel.x vel.y rotation_speed]

  if (new_robot.body.W_body_pos.azimuth < 0.0f)
      new_robot.body.W_body_pos.azimuth += 2*M_PI;
  else if (new_robot.body.W_body_pos.azimuth > 2*M_PI)
      new_robot.body.W_body_pos.azimuth -= 2*M_PI;

  new_robot.body.B_body_vel.x = motion_command[0];
  new_robot.body.B_body_vel.y = motion_command[1];
  new_robot.body.B_body_vel.azimuth = motion_command[2];

#ifdef DEBUG
  double trans_speed = sqrt(motion_command[0]*motion_command[0] +
			    motion_command[1]*motion_command[1]);  //[cm/sec]
  float crab_angle = atan2(motion_command[1],motion_command[0]);   //[rad]

  printf("\33[3;1H trans_speed  = %f[cm/sec]\n", trans_speed);
  printf("\33[4;1H duty factor  = %f\n", new_robot.leg1.duty_factor);
  printf("\33[5;1H crab_angle   = %f[deg]\n", crab_angle*RADTODEG);
  printf("\33[6;1H body_v.x     = %f\n", new_robot.body.B_body_vel.x);
  printf("\33[7;1H body_v.y     = %f\n", new_robot.body.B_body_vel.y);
  printf("\33[8;1H body azimuth = %f\n",
	 RADTODEG * new_robot.body.W_body_pos.azimuth);
#endif

  /////////////////////////////////////////////////////////////////////////
  // Here is the gait algolithm! //////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////

  // Find Duty Factor();
  new_robot.Find_Duty_Factor();

  // Find Relative Phase
  new_robot.Find_Relative_Phase();

  // Resize Constrained Working Volume
//  new_robot.Resize_CWV();

  // Determine Kinematic Period and Support Phase Variable
  new_robot.Support_Phase_Block(sampling_time);

  // Plan Transfer Phase Foot Trajectory
  new_robot.Transfer_Phase_Block();

  // Update Body Posture
  new_robot.Update_Body_Posture_Block(sampling_time);

  // Update Homogenious Transformation Matrices
  new_robot.Update_H_Matrix();

  // Update CWV Position
  new_robot.Update_CWV_Position_Block();
  
  // Update Foot Position
  new_robot.Update_Foot_Position_Block(sampling_time);

/*
  // Compute Inverse Jacobian
  new_robot.Inverse_Jacobian_Block();

  // Update Joint Angle
  new_robot.Update_Joint_Angle_Block(sampling_time);
*/

  // Compute Inverse Kinematics
  new_robot.Inverse_Kinematics_Block();
  
  // interface
//  interface_R2N(new_robot, next)
    
}// End of gait_algorithm()


// **************************************************************************
// FUNCTION: mod1()
// **************************************************************************
GAIT_DLL_API double mod1(double dividend)
{
  double ans;

  ans = fmod(dividend, 1.0);
  if (ans < 0.0) {
    ans = ans + 1.0;
  }
  return (ans); 

}


// **************************************************************************
// FUNCTION: display_robot_position()
// **************************************************************************
GAIT_DLL_API void display_robot_position(Posture &pos, Posture &) //vel)
{
  Vector W_body_position(pos.x, pos.y, pos.z);
  //Vector W_body_velocity(vel.x, vel.y, vel.z);
  //Vector W_body_eulerang(pos.roll, pos.elevation, pos.azimuth);
  //Vector W_body_eulerrat(vel.roll, vel.elevation, vel.azimuth);


  printf("body positon ="); W_body_position.PrintVector();
//  printf("body eulerang="); W_body_eulerang.PrintVector();
//  printf("body velocity="); W_body_velocity.PrintVector();
//  printf("body eulerrat="); W_body_eulerrat.PrintVector();
}


// **************************************************************************
// FUNCTION: display_leg_position()
// **************************************************************************
GAIT_DLL_API void display_leg_position(Leg &) // (Leg &leg)
{
//  double a = leg.leg_no;
//  printf("LEG%d----------------------------------\n", leg.leg_no);
//  printf("cwv_pos   =");  leg.W_cwv_pos.PrintVector();
//  printf("cwv_vel   =");  leg.W_cwv_vel.PrintVector();
//  printf("foothold  =");  leg.foothold.PrintVector();
//  printf("W_foot_pos=");  leg.W_foot_pos.PrintVector();
//  printf("B_foot_pos=");  leg.B_foot_pos.PrintVector();
//  printf("W_foot_vel=");  leg.W_foot_vel.PrintVector();
}

// EOF
