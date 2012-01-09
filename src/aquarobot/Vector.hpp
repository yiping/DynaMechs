// **************************************************************************
// FILENAME:Vector.hpp
// AUTHOR:  Kenji Suzuki
// DATE:    07 July 1993
// **************************************************************************

#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <Gait.hpp>
#include <iostream>
using namespace std;
// **************************************************************************
// class Vector
// **************************************************************************
class GAIT_DLL_API Vector
{
private:
public:
  // Structure
  double x;
  double y;
  double z;

public:
  // Constructor
  Vector();
  Vector(double, double, double);
  
  // Destructor
  virtual ~Vector() {}
  
  // Operator
  friend ostream &operator<<(ostream &, Vector &);
  Vector operator+();
  Vector operator-();
  friend Vector operator+(const Vector &, const Vector &);
  friend Vector operator-(const Vector &, const Vector &);
  friend Vector operator*(const Vector &, const double);
  friend Vector operator*(const double, const Vector &);
  friend Vector operator/(const Vector &, const double);

  // Member Function
  double Norm();
  double Distance(const Vector &);
  Vector Normalize();
  Vector Outerproduct(const Vector &);
  double Innerproduct(const Vector &);
  void PrintVector();
};


#endif

// EOF
