// **************************************************************************
// FILENAME: EulerAng.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     14 June 1993
// **************************************************************************

#include "EulerAng.hpp"


// **************************************************************************
// EulerAng Class Member Function
// **************************************************************************
EulerAng::EulerAng()
{
  azimuth = 0.0;   // yaw
  elevation = 0.0; // pitch
  roll = 0.0;      // roll
}


// **************************************************************************
EulerAng::EulerAng(double rl, double el, double az)
{
  azimuth = az;   // yaw
  elevation = el; // pitch
  roll = rl;      // roll
}


ostream &operator<<(ostream &strm, EulerAng &e)
{
  return (strm << "(" << 
          e.azimuth << ", " << e.elevation << ", " << e.roll << ")");
}


// **************************************************************************
EulerAng EulerAng::operator+(EulerAng e)
{
  EulerAng temp;

  temp.azimuth   = azimuth   + e.azimuth;
  temp.elevation = elevation + e.elevation;
  temp.roll      = roll      + e.roll;

  return (temp);
}


// **************************************************************************
EulerAng EulerAng::operator-(EulerAng e)
{
  EulerAng temp;

  temp.azimuth   = azimuth   - e.azimuth;
  temp.elevation = elevation - e.elevation;
  temp.roll      = roll      - e.roll;

  return (temp);
}


// EOF
