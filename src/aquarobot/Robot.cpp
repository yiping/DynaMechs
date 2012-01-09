// **************************************************************************
// FILENAME: Robot.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     15 June 1993
// **************************************************************************

#include "Robot.hpp"

// **************************************************************************
// Robot Class Member Function
// Constructor
// **************************************************************************
Robot::Robot()
{
  body = Body();
  leg1 = Leg(1);
  leg2 = Leg(2);
  leg3 = Leg(3);
  leg4 = Leg(4);
  leg5 = Leg(5);
  leg6 = Leg(6);
  H_matrix_B2W = MatrixMy(4, 4);
  H_matrix_B2W.Set_H_Matrix(body.W_body_pos.x,
                            body.W_body_pos.y,
                            body.W_body_pos.z,
                            body.W_body_pos.roll,
                            body.W_body_pos.elevation,
                            body.W_body_pos.azimuth);
  H_matrix_W2B = MatrixMy(4, 4);
  H_matrix_W2B = H_matrix_B2W.Inverse();

  R_matrix_B2W = MatrixMy(3, 3);
  R_matrix_B2W.Set_Hr_Matrix(body.W_body_pos.roll,
                             body.W_body_pos.elevation,
                             body.W_body_pos.azimuth);
  R_matrix_W2B = MatrixMy(3, 3);
  R_matrix_W2B = R_matrix_B2W.Inverse();
  kinematic_phase_variable = 0.0;
  kinematic_period = 5.0;
//  support_period = kinematic_period*duty_factor;
//  transfer_period = kinematic_period - support_period;
}



// **************************************************************************
// Robot Class Member Function
// Constructor
// **************************************************************************
Robot::Robot(double xx, double yy, double zz,
             double rl, double el, double az)
{
  body = Body(xx, yy, zz, rl, el, az);
  leg1 = Leg(1);
  leg2 = Leg(2);
  leg3 = Leg(3);
  leg4 = Leg(4);
  leg5 = Leg(5);
  leg6 = Leg(6);
  H_matrix_B2W = MatrixMy(4, 4);
  H_matrix_B2W.Set_H_Matrix(body.W_body_pos.x,
                            body.W_body_pos.y,
                            body.W_body_pos.z,
                            body.W_body_pos.roll,
                            body.W_body_pos.elevation,
                            body.W_body_pos.azimuth);
  H_matrix_W2B = MatrixMy(4, 4);
  H_matrix_W2B = H_matrix_B2W.Inverse();
  R_matrix_B2W = MatrixMy(3, 3);
  R_matrix_B2W.Set_Hr_Matrix(body.W_body_pos.roll,
                             body.W_body_pos.elevation,
                             body.W_body_pos.azimuth);
  R_matrix_W2B = MatrixMy(3, 3);
  R_matrix_W2B = R_matrix_B2W.Inverse();
  kinematic_phase_variable = 0.0;
  kinematic_period = 5.0;
//   support_period = kinematic_period*duty_factor;
//   transfer_period = kinematic_period - support_period;

} //   End of Robot::Robot()



// **************************************************************************
// FUNCTION: Resize_CWV()
// AUTHOR:   Kan Yoneda and Kenji Suzuki.
// DATE:     16 Aug 1993.
// **************************************************************************
void Robot::Resize_CWV()
{
  double body_speed;
  double v0 = 10.0; // velocity constant.[cm/sec]
  body_speed = sqrt( sqr(body.B_body_vel.x) + sqr(body.B_body_vel.y));

  leg1.cwv_radius = 
  leg2.cwv_radius = 
  leg3.cwv_radius = 
  leg4.cwv_radius = 
  leg5.cwv_radius = 
  leg6.cwv_radius = 0.8*R_CWV*(1.0 - exp(-body_speed/v0)) + 0.2*R_CWV;


} // End of Resize_CWV()



// **************************************************************************
// FUNCTION: Find_Duty_Factor().
// AUTHOR:   Kan Yoneda and Kenji Suzuki.
// DATE:     09 Aug 1993.
// **************************************************************************
void Robot::Find_Duty_Factor()
{
  double body_speed; // scaler body velocity
  double leg_arrange_radius = STANCE;
  double max_leg_transfer_vel = MAX_TRANSFER_B_FOOT_VEL;

  
  // Compute scaler body speed
  body_speed = sqrt( sqr(body.B_body_vel.x) + sqr(body.B_body_vel.y) ) + 
               leg_arrange_radius*fabs(body.B_body_vel.azimuth);

  // Compute duty factor
  duty_factor = max_leg_transfer_vel/(max_leg_transfer_vel + body_speed);

  leg1.duty_factor = duty_factor;
  leg2.duty_factor = duty_factor;
  leg3.duty_factor = duty_factor;
  leg4.duty_factor = duty_factor;
  leg5.duty_factor = duty_factor;
  leg6.duty_factor = duty_factor;

} // End of Find_duty_factor()




// **************************************************************************
// FUNCTION: void Find_Relative_Phase().
// AUTHOR:   Kan Yoneda and Kenji Suzuki.
// DATE:     09 Aug 1993.
// **************************************************************************
void Robot::Find_Relative_Phase()
{
  double crab_angle;
  double relative_alpha; // crab angle from each leg
  double relative_phase[7];
  double f23;            //relative_pahse_beta_2of3
  int lg;
  double beta = leg1.duty_factor;


  for (lg = 1; lg <= 6; lg++) {
  
    // Find CrabAngle
    if ((body.B_body_vel.x == 0.0) || (body.B_body_vel.y == 0.0)) {
      crab_angle = 0.0;
    }
    else {
      crab_angle = atan2( body.B_body_vel.y, body.B_body_vel.x );
    }
    
    relative_alpha = 2.0*PI*
                     mod1( (crab_angle - double(lg - 1)*PI/3.0)/(2.0*PI) );

    // Find RelativePhase
    if( relative_alpha < PI*(1.0/6.0) ) {
      f23 = 0.0;
    }
    else if( relative_alpha < PI*(5.0/6.0) ) {
      f23 = 0.5*((relative_alpha/PI) - (1.0/6.0));
    }
    else if( relative_alpha < PI*(7.0/6.0) ) {
      f23 = 1.0/3.0;
    }
    else if( relative_alpha < PI*(11.0/6.0) ) {
      f23 = (1.0/3.0) - 0.5*((relative_alpha/PI) - (7.0/6.0));
    }
    else {
      f23 = 0.0;
    }

    // 
    relative_phase[lg] = 
      (6.0*f23 - (3.0/2.0))*(beta - 0.5) + (1.0/4.0);

    // shift relative phase 0.5
    if ( (lg == 2) || (lg == 4) || (lg == 6) ) { 
      relative_phase[lg] -= 0.5;  
    }

    relative_phase[lg] = mod1( relative_phase[lg] );
  }

  leg1.relative_phase = relative_phase[1];
  leg2.relative_phase = relative_phase[2];
  leg3.relative_phase = relative_phase[3];
  leg4.relative_phase = relative_phase[4];
  leg5.relative_phase = relative_phase[5];
  leg6.relative_phase = relative_phase[6];


// relative phase
//    |
//    |
//    |         _____
//    |        /     \            /
//    |       /       \          /
//    |      /         \        /
//    |  ___/           \______/
//    |
//    ----------------------------------> CrabAngle
//   0     (1/6)PI   ...          2PI
//
//  Ask Yoneda for detail. 


} // End of find_relative_phase( void )



// **************************************************************************
// void Robot::Support_Phase_Block()
// **************************************************************************
void Robot::Support_Phase_Block(double sampling_time)
{

  // Find Maximum Instantaneous Support Period
  leg1.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 
  leg2.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 
  leg3.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 
  leg4.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 
  leg5.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 
  leg6.Find_Max_Instantaneous_Support_Period_Sequence(body.B_body_vel); 


  // Update CWV Intersection Position(Predicted liftoff point)
  // (BODY->WORLD coordinates).
  leg1.W_foothold = H_matrix_B2W*leg1.B_foothold;
  leg2.W_foothold = H_matrix_B2W*leg2.B_foothold;
  leg3.W_foothold = H_matrix_B2W*leg3.B_foothold;
  leg4.W_foothold = H_matrix_B2W*leg4.B_foothold;
  leg5.W_foothold = H_matrix_B2W*leg5.B_foothold;
  leg6.W_foothold = H_matrix_B2W*leg6.B_foothold;


  // Find Kinematic Period
  Find_Kinematic_Period();


  // Find Kinematic Phase Variable    
  kinematic_phase_variable = 
    fmod(kinematic_phase_variable + sampling_time/kinematic_period, 1.0);

  
  // Find Leg Phase Variable
  leg1.Find_Leg_Phase_Var(kinematic_phase_variable);
  leg2.Find_Leg_Phase_Var(kinematic_phase_variable);
  leg3.Find_Leg_Phase_Var(kinematic_phase_variable);
  leg4.Find_Leg_Phase_Var(kinematic_phase_variable);
  leg5.Find_Leg_Phase_Var(kinematic_phase_variable);
  leg6.Find_Leg_Phase_Var(kinematic_phase_variable);


  // Find Support Phase Variable
  leg1.Find_Support_Phase_Var(kinematic_phase_variable);
  leg2.Find_Support_Phase_Var(kinematic_phase_variable);
  leg3.Find_Support_Phase_Var(kinematic_phase_variable);
  leg4.Find_Support_Phase_Var(kinematic_phase_variable);
  leg5.Find_Support_Phase_Var(kinematic_phase_variable);
  leg6.Find_Support_Phase_Var(kinematic_phase_variable);


  // Compute Foot Velocity (WORLD coordinates)
  leg1.W_foot_vel = Vector(0.0, 0.0, 0.0);
  leg2.W_foot_vel = Vector(0.0, 0.0, 0.0);
  leg3.W_foot_vel = Vector(0.0, 0.0, 0.0);
  leg4.W_foot_vel = Vector(0.0, 0.0, 0.0);
  leg5.W_foot_vel = Vector(0.0, 0.0, 0.0);
  leg6.W_foot_vel = Vector(0.0, 0.0, 0.0);


} // End of Robot::Support_Phase_Block()



// **************************************************************************
// FUNCTION: Find_Kinematic_Period()
// **************************************************************************
void Robot::Find_Kinematic_Period()
{
  kinematic_period
    = aqmin(leg1.max_instantaneous_support_period/
          leg1.duty_factor, 
          aqmin(leg2.max_instantaneous_support_period/
              leg2.duty_factor, 
              aqmin(leg3.max_instantaneous_support_period/
                  leg3.duty_factor,
                  aqmin(leg4.max_instantaneous_support_period/
                      leg4.duty_factor,
                      aqmin(leg5.max_instantaneous_support_period/
                          leg5.duty_factor,
                          leg6.max_instantaneous_support_period/
                          leg6.duty_factor)
                      )
                  )
              )
          );

} // End of Find Kinematic_Period()



// **************************************************************************
// void Robot::Transfer_Phase_Block()
// **************************************************************************
void Robot::Transfer_Phase_Block()
{
  leg1.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

  leg2.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

  leg3.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

  leg4.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

  leg5.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

  leg6.Transfer_Phase_Sequence(body.B_body_vel,
                               R_matrix_B2W,
                               R_matrix_W2B,
                               kinematic_period);

} // End of Robot::Transfer_Phase_Block()



// **************************************************************************
// FUNCTION: Update_H_matrix()
// **************************************************************************
void Robot::Update_H_Matrix()
{
  H_matrix_B2W.Set_H_Matrix(body.W_body_pos.x,
                            body.W_body_pos.y,
                            body.W_body_pos.z,
                            body.W_body_pos.roll,
                            body.W_body_pos.elevation,
                            body.W_body_pos.azimuth);

  H_matrix_W2B = H_matrix_B2W.Inverse();

  R_matrix_B2W.Set_Hr_Matrix(body.W_body_pos.roll,
                             body.W_body_pos.elevation,
                             body.W_body_pos.azimuth);

  R_matrix_W2B = R_matrix_B2W.Inverse();

} // End of Robot::Update_H_Matrix()




// **************************************************************************
// void Robot::Update_Body_Posture_Block()
// **************************************************************************
void Robot::Update_Body_Posture_Block(double sampling_time)
{
  Vector W_body_position(body.W_body_pos.x,
                         body.W_body_pos.y,
                         body.W_body_pos.z);
  Vector W_body_eulerang(body.W_body_pos.roll,
                         body.W_body_pos.elevation,
                         body.W_body_pos.azimuth);

  Vector B_body_trans_rate(body.B_body_vel.x,
                           body.B_body_vel.y,
                           body.B_body_vel.z);
  Vector B_body_euler_rate(body.B_body_vel.roll,
                           body.B_body_vel.elevation,
                           body.B_body_vel.azimuth);

  Vector W_body_trans_rate;
  Vector W_body_euler_rate;

  W_body_trans_rate = R_matrix_B2W*B_body_trans_rate;
  W_body_euler_rate = R_matrix_B2W*B_body_euler_rate;

  body.W_body_vel = Posture(W_body_trans_rate.x,
                            W_body_trans_rate.y,
                            W_body_trans_rate.z,
                            W_body_euler_rate.x,
                            W_body_euler_rate.y,
                            W_body_euler_rate.z);


  // Update body posture(WORLD frame)
  W_body_position = W_body_position + W_body_trans_rate*sampling_time;
  W_body_eulerang = W_body_eulerang + W_body_euler_rate*sampling_time;
  body.W_body_pos = Posture(W_body_position.x,
                            W_body_position.y,
                            W_body_position.z,
                            W_body_eulerang.x,
                            W_body_eulerang.y,
                            W_body_eulerang.z);

} // End of Robot::Update_Body_Posture()



// **************************************************************************
// void Robot::Update_CWV_Position_Block()
// **************************************************************************
void Robot::Update_CWV_Position_Block()
{
  leg1.W_cwv_pos = H_matrix_B2W*leg1.B_cwv_pos;
  leg2.W_cwv_pos = H_matrix_B2W*leg2.B_cwv_pos;
  leg3.W_cwv_pos = H_matrix_B2W*leg3.B_cwv_pos;
  leg4.W_cwv_pos = H_matrix_B2W*leg4.B_cwv_pos;
  leg5.W_cwv_pos = H_matrix_B2W*leg5.B_cwv_pos;
  leg6.W_cwv_pos = H_matrix_B2W*leg6.B_cwv_pos;

} // End of Robot::Update_CWV_Position()



// **************************************************************************
// FUNCTION: Robot::Update_Foot_Position_Block(double sampling_time)
// **************************************************************************
void Robot::Update_Foot_Position_Block(double sampling_time)
{
  leg1.Update_Foot_Position(H_matrix_W2B, sampling_time);
  leg2.Update_Foot_Position(H_matrix_W2B, sampling_time);
  leg3.Update_Foot_Position(H_matrix_W2B, sampling_time);
  leg4.Update_Foot_Position(H_matrix_W2B, sampling_time);
  leg5.Update_Foot_Position(H_matrix_W2B, sampling_time);
  leg6.Update_Foot_Position(H_matrix_W2B, sampling_time);

} // End of Update_Foot_Position_Block()



// **************************************************************************
// FUNCTION:Inverse_Jacobian_Block()
// **************************************************************************
void Robot::Inverse_Jacobian_Block()
{
  leg1.Inv_Jacobian();
  leg2.Inv_Jacobian();
  leg3.Inv_Jacobian();
  leg4.Inv_Jacobian();
  leg5.Inv_Jacobian();
  leg6.Inv_Jacobian();

} // End of Inverse_Jacobian_Block()



// **************************************************************************
// FUNCTION:Inverse_Kinematics_Block()
// **************************************************************************
void Robot::Inverse_Kinematics_Block()
{
  leg1.Inv_Kinematics();
  leg1.Update_Joint_Position(H_matrix_B2W);

  leg2.Inv_Kinematics();
  leg2.Update_Joint_Position(H_matrix_B2W);

  leg3.Inv_Kinematics();
  leg3.Update_Joint_Position(H_matrix_B2W);

  leg4.Inv_Kinematics();
  leg4.Update_Joint_Position(H_matrix_B2W);

  leg5.Inv_Kinematics();
  leg5.Update_Joint_Position(H_matrix_B2W);

  leg6.Inv_Kinematics();
  leg6.Update_Joint_Position(H_matrix_B2W);

} // End of Inverse_Kinematics_Block()



// **************************************************************************
// FUNCTION:Update_Joint_Angle_Block(double sampling_time)
// **************************************************************************
void Robot::Update_Joint_Angle_Block(double sampling_time)
{
  leg1.Update_Joint_Angle(H_matrix_B2W, sampling_time);
  leg2.Update_Joint_Angle(H_matrix_B2W, sampling_time);
  leg3.Update_Joint_Angle(H_matrix_B2W, sampling_time);
  leg4.Update_Joint_Angle(H_matrix_B2W, sampling_time);
  leg5.Update_Joint_Angle(H_matrix_B2W, sampling_time);
  leg6.Update_Joint_Angle(H_matrix_B2W, sampling_time);

} // End of Update_Joint_Angle_Block()


// EOF
