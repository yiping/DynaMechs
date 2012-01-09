// **************************************************************************
// FILENAME:LegLink.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    09 June 1993
// **************************************************************************

#ifndef LEGLINK_HPP
#define LEGLINK_HPP

#include <Gait.hpp>
#include <Vector.hpp>

// **************************************************************************
// class LegLink
// **************************************************************************
class GAIT_DLL_API LegLink
{
public:
  // Structure
  int joint_no; // CB:0, HIP:1, KNEE1:2, KNEE2:3, FOOT(ANKLE):4, SOLE:5
  int link_no;  // CB->HIP:0, HIP->KNEE1:1, etc.
  double link_length;       // DH parameter, a[cm]
  double twist_angle;       // DH parameter, alpha[rad]
  double displacement;      // DH parameter, d[cm]
  double joint_angle_pos;   // Joint Angle Position[rad]
  double joint_angle_vel;   // Joint Angular Velocity[rad/sec]
  double joint_angle_acl;   // Joint Angular Acceleration[rad/sec^2]
  double min_joint_angle;   // Minimum Joint Angle Limit[rad]
  double max_joint_angle;   // Maximum Joint Angle Limit[rad]
  int joint_limit_flag;     // 0:free, 1:limit
  Vector B_joint_pos;       // Joint Position(BODY coordinates)
  Vector W_joint_pos;       // Joint Position(BODY coordinates)
//   MatrixMy T_matrix;          // Transformation
//   MatrixMy Pos_matrix;        // Joint Position Vector

  // Constructor
  LegLink();
  LegLink(int, double, double, double, double);

  // Destructor
  ~LegLink() {}

  // Operator

  // Member Function
};


#endif
// EOF
