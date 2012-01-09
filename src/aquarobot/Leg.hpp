// **************************************************************************
// FILENAME:Leg.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    14 June 1993
// **************************************************************************

#ifndef LEG_HPP
#define LEG_HPP

#include <Gait.hpp>
#include <Posture.hpp>
#include <LegLink.hpp>
#include <Body.hpp>
#include <MatrixMy.hpp>
#include <Aquarobo.h>

// **************************************************************************
// class Leg 
// **************************************************************************

class GAIT_DLL_API Leg
{
public:
  // Structure
  int leg_no;           // from 1 through 6
  LegLink link0;        // CB to HIP
  LegLink link1;        // HIP to KNEE1
  LegLink link2;        // KNEE1 to KNEE2
  LegLink link3;        // KNEE2 to FOOT(ANKLE)
  LegLink link4;        // FOOT(ANKLE) to SOLE
  int contact_flag;     // FOOT Contact Flag( 0:Free, 1:Contact )
  int old_contact_flag; // previous contact flag
  Vector B_foot_pos;    // FOOT Position[cm]     BODY coordinates
  Vector B_foot_vel;    // FOOT Velocity[cm]     BODY coordinates
//  Vector B_foot_acl;    // FOOT Acceleration[cm] BODY coordinates
  Vector W_foot_pos;    // FOOT Position[cm]     WORLD coordinates
  Vector W_foot_vel;    // FOOT Velocity[cm]     WORLD coordinates
//  Vector W_foot_acl;    // FOOT Acceleration[cm] WORLD coordinates
  Vector B_foothold;    // predicted Foothold (BODY coordinates)
  Vector W_foothold;    // predicted Foothold (WORLD coordinates)
  Vector B_cwv_pos;  // center of Constrained Working Volume BODY coord.
  Vector W_cwv_pos;  // center of Constrained Working Volume WORLD coord.
  Vector W_cwv_vel;     // Velocity of CWV center
  double cwv_radius;    // radius of the cwv
  double cwv_height;    // maxmum foot height [cm]
  int leg_phase_status_flag;  // 0:Support, 1:Transfer
  double foot_lift_height;
  double ground_level;



  // Time elements
  // See the article, for definition of variables
  // Lee and Orin "Omnidirectional Supervisory Control 
  // of a Multilegged Vehicle Using Periodic Gaits" 
  // IEEE Robotics and Automation, Vol.4, No.6, December 1988
  double duty_factor;
  double relative_phase;
  double leg_phase_variable;
  double transfer_phase_variable;
  double support_phase_variable;
  double temporal_kinematic_margin;
  double max_instantaneous_support_period;

  // Constructor
  Leg();
  Leg(int);

  // Destructor
  ~Leg() {}

  // Operator

  // Member Function
  void Kinematics();
  void Inv_Kinematics();
  void Jacobian();
  void Inv_Jacobian();



  void Find_Max_Instantaneous_Support_Period_Sequence(Posture &B_body_vel);
  void Find_Temporal_Kinematic_Margin();
  int Find_CWV_Intersection(Vector &);
  void Find_Max_Instantaneous_Support_Period();

  void Find_Leg_Phase_Var(double);
  void Find_Support_Phase_Var(double);

  void Transfer_Phase_Sequence(Posture &B_body_posrate,
                               MatrixMy &R_matrix_B2W,
                               MatrixMy &R_matrix_W2B,
                               double kinematic_period);
  void Find_Transfer_Phase_Var(double);
  void Find_CWV_Velocity(Posture &, MatrixMy &);
  void Find_Predicted_Foothold(double);
  void Find_Transfer_Foot_Velocity(int, double);

  void Update_Foot_Position(MatrixMy &H_matrix_W2B, double sampling_time);
  void Update_Joint_Angle(MatrixMy &H_matrix_B2W, double sampling_time);
  void Update_Joint_Position(MatrixMy &H_matrix_B2W);

  void Body_to_World(Posture &);
  void World_to_Body(Posture &);
  void Velocity_Body_to_World(Body &);
  void Velocity_World_to_Body(Body &);
};


GAIT_DLL_API Vector Body_to_World(Posture &, Vector &);
GAIT_DLL_API Vector World_to_Body(Posture &, Vector &);
GAIT_DLL_API double mod1(double);

#endif


// EOF
