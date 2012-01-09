// **************************************************************************
// FILENAME:EulerAng.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    14 June 1993
// **************************************************************************

#ifndef EULERANG_HPP
#define EULERANG_HPP

#include <Gait.hpp>
#include <iostream>
using namespace std;

// **************************************************************************
// CLASS: EulerAng
// **************************************************************************
class GAIT_DLL_API EulerAng
{
public:
  // Structure
  double azimuth;   // yaw
  double elevation; // pitch
  double roll;      // roll

  // Constructor
  EulerAng();
  EulerAng(double, double, double);
  
  // Destructor
  virtual ~EulerAng() {}

  // Operator
  friend ostream &operator<<(ostream &, EulerAng &);
  EulerAng operator+(EulerAng);
  EulerAng operator-(EulerAng);

  // Member Function
};


#endif
// EOF
