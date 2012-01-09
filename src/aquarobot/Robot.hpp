// **************************************************************************
// FILENAME:Robot.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    15 June 1993
// **************************************************************************

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Gait.hpp>
#include <Aquarobo.h>
#include <Leg.hpp>
#include <Body.hpp>


// **************************************************************************
// class Robot
// **************************************************************************
class GAIT_DLL_API Robot
{
public:
  // Structure
  Body body;        // torso
  Leg leg1;         // leg1
  Leg leg2;         // leg2
  Leg leg3;         // leg3
  Leg leg4;         // leg4
  Leg leg5;         // leg5
  Leg leg6;         // leg6
  MatrixMy H_matrix_B2W; // Homogeneous Transformation Matrix(BODY->WORLD)
  MatrixMy H_matrix_W2B; // Homogeneous Transformation Matrix(WORLD->BODY)
  MatrixMy R_matrix_B2W; // Rotation Transformation Matrix(BODY->WORLD)
  MatrixMy R_matrix_W2B; // Rotation Transformation Matrix(WORLD->BODY)

  // time elements
  double current_time;     // ccurrent time
  double duty_factor;      // duty factor is in Leg class.
//  double kinematic_cycle_phase_variable;
  double kinematic_phase_variable;
  double kinematic_period; // same as locomotion period
  double transfer_period;  // transfer time[sec]
  double support_period;   // support time[sec] 

  // Constructor
  Robot();
  Robot(double, double, double, double, double, double);
  
  // Destrucor
  ~Robot() {}

  // Operator

  // Member Function
  void Resize_CWV();
  void Find_Duty_Factor();
  void Find_Relative_Phase();

  void Support_Phase_Block(double sampling_time);
  void Find_Kinematic_Period();

  void Transfer_Phase_Block();

  void Update_H_Matrix();
  void Update_Body_Posture_Block(double sampling_time);
  void Update_CWV_Position_Block();
  void Update_Foot_Position_Block(double sampling_time);
  void Inverse_Jacobian_Block();
  void Inverse_Kinematics_Block();
  void Update_Joint_Angle_Block(double sampling_time);

};


#endif

// EOF
