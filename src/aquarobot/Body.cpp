// **************************************************************************
// FILENAME: Body.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     15 June 1993
// **************************************************************************

#include <stdio.h>
#include <math.h>

#include "Body.hpp"


// **************************************************************************
// Body Class Member Function
// **************************************************************************
Body::Body()
{
  W_body_pos = Posture(0.0, 0.0, -94.57, 0.0, 0.0, 0.0);
  W_body_vel = Posture();
  W_body_acl = Posture();
  heading   = 0.0;
  direction = 0.0;
  body_speed = 0.0;
  body_height = fabs(W_body_pos.y);
}

// **************************************************************************
Body::Body(double xx, double yy, double zz,
           double az, double el, double rl)
{
  W_body_pos = Posture(xx, yy, zz, az, el, rl);
  W_body_vel = Posture();
  W_body_acl = Posture();
  heading = az;
  direction = 0.0;
  body_speed = 0.0;
  body_height = fabs(W_body_pos.y);
  body_speed  =   5.0;
  body_height = BODY_HEIGHT;
}


// End of file
