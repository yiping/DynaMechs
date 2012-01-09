// **************************************************************************
// FILENAME:Posture.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    14 June 1993
// **************************************************************************
#ifndef POSTURE_HPP
#define POSTURE_HPP

#include <Gait.hpp>
#include <Vector.hpp>
#include <EulerAng.hpp>


// **************************************************************************
// CLASS: Posture
// **************************************************************************
class GAIT_DLL_API Posture : public Vector, public EulerAng
{
public:
  // Structure

  // Constructor
  Posture();
  Posture(double, double, double, double, double, double);

  // Destructor
  ~Posture() {}

  // Operator
  friend ostream &operator<<(ostream &, Posture &);

  // Member Function
};


#endif
// EOF
