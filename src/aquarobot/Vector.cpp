// **************************************************************************
// FILENAME: Vector.C
// AUTHOR:   Kenji Suzuki
// DATE:     07 July 1993
// **************************************************************************

#include "Vector.hpp"


// **************************************************************************
// Vector Class Member Function
// **************************************************************************
Vector::Vector()
{
   x = 0.0;
   y = 0.0;
   z = 0.0;
}


// **************************************************************************
Vector::Vector(double xx, double yy, double zz)
{
  x = xx;
  y = yy;
  z = zz;
}


// **************************************************************************
ostream &operator<<(ostream &strm, Vector &p)
{
  return strm << "(" << p.x << ", " << p.y << ", " << p.z << ")";
}


// **************************************************************************
// "+" operator function
// **************************************************************************
Vector Vector::operator+()
{
  Vector temp;

  temp.x = -x;
  temp.y = -y;
  temp.z = -z;

  return (temp);
}


// **************************************************************************
// "-" operator function
// **************************************************************************
Vector Vector::operator-()
{
  Vector temp;

  temp.x = -x;
  temp.y = -y;
  temp.z = -z;

  return (temp);
}


// **************************************************************************
// Friend "+" operator function
// **************************************************************************
Vector operator+(const Vector &left, const Vector &right)
{
  Vector temp;
  
  temp.x = left.x + right.x;
  temp.y = left.y + right.y;
  temp.z = left.z + right.z;

  return (temp);
}


// **************************************************************************
// Friend "-" operator function
// **************************************************************************
Vector operator-(const Vector &left, const Vector &right)
{
  Vector temp;
  
  temp.x = left.x - right.x;
  temp.y = left.y - right.y;
  temp.z = left.z - right.z;

  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
Vector operator*(const Vector &left, const double a)
{
  Vector temp;
  
  temp.x = left.x*a;
  temp.y = left.y*a;
  temp.z = left.z*a;

  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
Vector operator*(const double a, const Vector &right)
{
  Vector temp;
  
  temp.x = a*right.x;
  temp.y = a*right.y;
  temp.z = a*right.z;

  return (temp);
}


// **************************************************************************
// Friend "/" operator function
// **************************************************************************
Vector operator/(const Vector &left, const double a)
{
  Vector temp;
  
  temp.x = left.x/a;
  temp.y = left.y/a;
  temp.z = left.z/a;

  return (temp);
}


// **************************************************************************
double Vector::Norm()
{
  return ( sqrt(x*x + y*y + z*z) );
}


// **************************************************************************
double Vector::Distance(const Vector &p)
{
  return (sqrt( (x - p.x)*(x - p.x)
               +(y - p.y)*(y - p.y)
               +(z - p.z)*(z - p.z)) );
}


// **************************************************************************
Vector Vector::Normalize()
{
  Vector temp(x, y, z);

  temp = temp*(1.0/temp.Norm());
  return (temp);
}


// **************************************************************************
// Member function: Outerproduct()
// **************************************************************************
Vector Vector::Outerproduct(const Vector &v)
{
  Vector temp;

  temp.x = y*v.z - z*v.y;
  temp.y = z*v.x - x*v.z;
  temp.z = x*v.y - y*v.x;
  
  return (temp);

} // End of Outerproduct()


// **************************************************************************
// Member function: Innerproduct()
// **************************************************************************
double Vector::Innerproduct(const Vector &v)
{

  return ( sqrt(x*v.x + y*v.y + z*v.z) );

} // End of Innerproduct()


// **************************************************************************
// Member function: PrintVector()
// COMMENT: print out Vector elements
// **************************************************************************
void Vector::PrintVector()
{
  printf("%f %f %f\n", x, y, z);
}  

// EOF
