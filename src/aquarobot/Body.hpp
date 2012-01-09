// **************************************************************************
// FILENAME:Body.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    15 June 1993
// **************************************************************************
#ifndef BODY_HPP
#define BODY_HPP

#include <Gait.hpp>
#include <Aquarobo.h>
#include <Posture.hpp>

// **************************************************************************
// class Body
// **************************************************************************
class GAIT_DLL_API Body
{
public:
  Posture W_body_pos;  // Body Position [cm]
  Posture W_body_vel;  // Body Velocity [cm/sec]
  Posture W_body_acl;  // Body Acceleration [cm/sec^2]
//  Posture B_body_pos;  // Body Position [cm]
  Posture B_body_vel;  // Body Velocity [cm/sec]
  Posture B_body_acl;  // Body Acceleration [cm/sec^2]
  double heading;    // heading angle [rad]
  double direction;  // direction of the motion [rad]
  double body_speed; // Body Speed [cm/sec]
  double body_height;// Body height [cm] == -(body_pos.z)

  // Constructor
  Body();
  Body(double, double, double, double, double, double);
  
  // Destrucor
  ~Body() {}

  // Operator

  // Member Function
};


#endif
// EOF
