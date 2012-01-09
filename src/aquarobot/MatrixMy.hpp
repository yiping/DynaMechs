// **************************************************************************
// FILENAME: MatrixMy.hpp
// PURPOSE:  To provide for a matrix class to accomplish
//           some necessary robotic and kinematic needs.
// AUTHOR:   S L Davidson
// DATE:     29 Oct 92
// COMMENTS: DHMatrix(), HomogeneousTransform(), and
//           TransformNodeList() are included 
// REVISION: 13 June 1993, by Kenji Suzuki
// **************************************************************************

#ifndef MATRIX_MY_HPP
#define MATRIX_MY_HPP

#include <Gait.hpp>
#include <Aquarobo.h>
#include <Vector.hpp>

// **************************************************************************
// class MatrixMy
// **************************************************************************
class GAIT_DLL_API MatrixMy
{
private:
  double **m; // matrix array
  int row;    // number of row
  int column; // number of column

public:
  // Constructor
  MatrixMy();
  MatrixMy(int, int);
  MatrixMy(const MatrixMy &); // copy constructor
  MatrixMy(Vector);         // Type Transfer(Vector to MatrixMy)

  // Destructor
  ~MatrixMy();

  // Operator
  MatrixMy &operator=(const MatrixMy &);
  operator Vector(); // this does not work
  MatrixMy operator+();
  MatrixMy operator-();
  friend MatrixMy operator+(const MatrixMy &, const MatrixMy &);
  friend MatrixMy operator-(const MatrixMy &, const MatrixMy &);
  friend MatrixMy operator*(const MatrixMy &, const MatrixMy &);
  friend MatrixMy operator*(const MatrixMy &, const double);
  friend MatrixMy operator*(const double, const MatrixMy &);
  friend Vector operator*(const MatrixMy &, const Vector &);

  // Member function
  void LU_Decomposition(MatrixMy &, double &);
  void LU_Backsubstitution(MatrixMy &, MatrixMy &);
  MatrixMy Inverse();

  MatrixMy Transpose();
  MatrixMy Outerproduct(MatrixMy &);
  double Innerproduct(MatrixMy &);

  double &Set_Value(int, int) const;
  double Get_Value(int, int);
  int rows() const { return row; };
  int cols() const { return column; };  
  void PrintMatrix();

  void Set_Vector3x1(double, double, double);
  void Set_Vector4x1(double, double, double);
  void Set_Vector(double, double, double);
  void Set_Hr_Matrix(double, double, double);
  void Set_H_Matrix(double, double, double, double, double, double);
  void Set_DH_Matrix(double, double, double, double);
  void Set_J_Matrix(double, double, double, double);
}; 



void nrerror(char []);


#endif


// EOF
