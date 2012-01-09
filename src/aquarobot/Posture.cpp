// **************************************************************************
// FILENAME: Posture.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     1993 June 14
// **************************************************************************

#include "Posture.hpp"


// **************************************************************************
// Posture Class Member Function
// **************************************************************************
Posture::Posture()
: Vector(), EulerAng()
{
}


// **************************************************************************
Posture::Posture(double xx, double yy, double zz,
                 double rl, double el, double az)
: Vector(xx, yy, zz), EulerAng(rl, el, az)
{
}


// **************************************************************************
ostream &operator<<(ostream &strm, Posture &p)
{
  return (strm << "Posture(" <<
          p.x << ", " << p.y << ", " << p.z << ", " <<
          p.azimuth << ", " << p.elevation << ", " << p.roll << ")");
}



// EOF
