// **************************************************************************
// FILENAME: MatrixMy.cpp
// PURPOSE:  Implementation of MatrixMy class
// CONTAINS:
// AUTHOR:   Kenji Suzuki
// DATE:     18 June 1993
// COMMENT:  Modified version of MatrixMy.C written by S L Davidson
// REVISION:
// **************************************************************************

#include <stdlib.h>
#include "MatrixMy.hpp"

 
// **************************************************************************
// Constructor
// **************************************************************************
MatrixMy::MatrixMy()
{
  row = 4;
  column = 4;
  m = new double *[4];

  int i, j;
  for (i = 0; i < 4; i++) {
    m[i] = new double[4];
  }
  
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      m[i][j] = 0.0;
    }
  }
}


// **************************************************************************
// Constructor
// **************************************************************************
//MatrixMy::MatrixMy(int r = 4, int c = 4)
MatrixMy::MatrixMy(int r = 3, int c = 3)
{
  row = r;
  column = c;
  m = new double *[row];

  int i, j;
  for (i = 0; i < row; i++) {
    m[i] = new double[column];
  }
  
  for (i = 0; i < row; i++) {
    for (j = 0; j < column; j++) {
      m[i][j] = 0.0;
    }
  }
}


// **************************************************************************
// Copy Constructor
// **************************************************************************
MatrixMy::MatrixMy(const MatrixMy &x)
{
  int i, j;
  
  row = x.row;
  column = x.column;
  
  m = new double *[row];
  for (i = 0; i < row; i++) {
    m[i] = new double[column];
  }
  
  for (i = 0; i < row; i++) {
    for (j = 0; j < column; j++) {
      m[i][j] = x.m[i][j];
    }
  }
}


// **************************************************************************
// Constructor fot Type Transfer(Vector -> MatrixMy)
// **************************************************************************
MatrixMy::MatrixMy(Vector p)
{
  row = 3;
  column = 1;
  int i;

  m = new double *[row];
  for (i = 0; i < row; i++) {
    m[i] = new double[column];
  }
  
  m[0][0] = p.x;
  m[1][0] = p.y;
  m[2][0] = p.z;
}


// **************************************************************************
// Destructor
// **************************************************************************
MatrixMy::~MatrixMy()
{
  if (m != 0) {
    int i;
    for (i = 0; i < row; i++) {
      delete m[i];
    }
    delete m;
  }
}


// **************************************************************************
// substitution operator
// **************************************************************************
MatrixMy &MatrixMy::operator=(const MatrixMy &x)
{
//   if ( (row == x.row) &&
//        (column == x.column) ) {

    int i, j;
    
    row = x.row;
    column = x.column;
    
    m = new double *[row];
    for (i = 0; i < row; i++) {
      m[i] = new double[column];
    }
    
    for (i = 0; i < row; i++) {
      for (j = 0; j < column; j++) {
        m[i][j] = x.m[i][j];
      }
    }
    return *this;
//   }
//   else {
//     printf("ERROR: Matrix Dimension Not Match(= operator)\n");
//   }
}


// **************************************************************************
// Type Transfer operator, Vector <- MatrixMy(3, 1)
// **************************************************************************
MatrixMy::operator Vector()
{
  Vector p;

  if ((row <= 4) && (column == 1)) {
    p.x = m[0][0];
    p.y = m[1][0];
    p.z = m[2][0];
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(= MatrixMy->Vector)\n");
  }
  return (p);
}


// **************************************************************************
// "+" operator function
// **************************************************************************
MatrixMy MatrixMy::operator+()
{
  MatrixMy temp(row, column);
  int i, j;

  for (i = 0; i < row; i++) {
    for (j = 0; j < column; j++) {
      temp.m[i][j] = m[i][j];
    }
  }
  
  return (temp);

}


// **************************************************************************
// "-" operator function
// **************************************************************************
MatrixMy MatrixMy::operator-()
{
  MatrixMy temp(row, column);
  int i, j;

  for (i = 0; i < row; i++) {
    for (j = 0; j < column; j++) {
      temp.m[i][j] =  - m[i][j];
    }
  }
  
  return (temp);

}


// **************************************************************************
// Friend "+" operator function
// **************************************************************************
MatrixMy operator+(const MatrixMy &left, const MatrixMy &right)
{
  MatrixMy temp(left.row, left.column);
  int i, j;

  if ( (left.row == right.row) &&
       (left.column == right.column) ) {

    for (i = 0; i < temp.row; i++) {
      for (j = 0; j < temp.column; j++) {
        temp.m[i][j] = left.m[i][j] + right.m[i][j];
      }
    }

  }
  else {
    printf("ERROR: Matrix Dimension Not Match(+ operator)\n");
  }
  
  return (temp);
}


// **************************************************************************
// Friend "-" operator function
// **************************************************************************
MatrixMy operator-(const MatrixMy &left, const MatrixMy &right)
{
  MatrixMy temp(left.row, left.column);
  int i, j;

  if ( (left.row == right.row) &&
       (left.column == right.column) ) {
    for (i = 0; i < temp.row; i++) {
      for (j = 0; j < temp.column; j++) {
        temp.m[i][j] = left.m[i][j] - right.m[i][j];
      }
    }
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(- operator)\n");
  }
  
  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
MatrixMy operator*(const MatrixMy &left, const MatrixMy &right)
{
  MatrixMy temp(right.row, right.column);
  int i, j, k;
  
  if (left.column == right.row) {
    for (i = 0; i < temp.row; i++) {
      for (j = 0; j < temp.column; j++) {
        for (k = 0; k < left.column; k++) {
          temp.m[i][j] += left.m[i][k]*right.m[k][j];
        }
      }
    }
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(* operator)\n");
  }

  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
MatrixMy operator*(const MatrixMy &left, const double a)
{
  MatrixMy temp(left.row, left.column);
  int i, j;
  
  for (i = 0; i < temp.row; i++) {
    for (j = 0; j < temp.column; j++) {
      temp.m[i][j] = a*left.m[i][j];
    }
  }

  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
MatrixMy operator*(const double a, const MatrixMy &right)
{
  MatrixMy temp(right.row, right.column);
  int i, j;
  
  for (i = 0; i < temp.row; i++) {
    for (j = 0; j < temp.column; j++) {
      temp.m[i][j] = a*right.m[i][j];
    }
  }
  return (temp);
}


// **************************************************************************
// Friend "*" operator function
// **************************************************************************
Vector operator*(const MatrixMy &left, const Vector &right)
{
  Vector temp;
  
  // for Usual Matrix Multiplication
  if ( (left.column == 3) && (left.row == 3) ) { // Mat(3x3) * Vec(3x1)
    temp.x = left.m[0][0]*right.x
           + left.m[0][1]*right.y 
           + left.m[0][2]*right.z;

    temp.y = left.m[1][0]*right.x
           + left.m[1][1]*right.y 
           + left.m[1][2]*right.z;

    temp.z = left.m[2][0]*right.x
           + left.m[2][1]*right.y 
           + left.m[2][2]*right.z;
  }
  // for Homogenious transformation
  else if ( (left.column == 4) && (left.row == 4) ) {// Mat(4x4) * Vec(3x1)
    temp.x = left.m[0][0]*right.x   
           + left.m[0][1]*right.y 
           + left.m[0][2]*right.z
           + left.m[0][3];

    temp.y = left.m[1][0]*right.x
           + left.m[1][1]*right.y 
           + left.m[1][2]*right.z
           + left.m[1][3];

    temp.z = left.m[2][0]*right.x
           + left.m[2][1]*right.y 
           + left.m[2][2]*right.z
           + left.m[2][3];
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(M*V operator)\n");
  }

  return (temp);
}


#define TINY 1.0e-20;
// **************************************************************************
// Member Function: LU_Decomposition()
// PURPOSE: Compute LU Decomposition. 
//          Preparation for computation of Inverse Matrix.
// COMMENT: Modified ludcmp() function from "Numerical Recipes in C"
// **************************************************************************
void MatrixMy::LU_Decomposition(MatrixMy &indx,
                                double &d)
{
  int i, imax=0, j, k;
  int n = row;
  double big, dum, sum, temp;
  MatrixMy vv(row, 1); // Vector
  
  if ((column != row) || 
      (indx.row != row) || 
      (indx.column != 1)) {
    printf("ERROR: Matrix Dimension Not Match(LUD)\n");
  }
  else {

    d = 1.0;
    for (i = 0; i < n; i++) {
      big = 0.0;
      for (j = 0; j < n; j++) {
        if ( (temp = fabs(m[i][j])) > big ) {
          big = temp;
        }
      }

      if (big == 0.0) {
        nrerror("Singular matrix in routine LUDCMP");
      }
      vv.m[i][0] = 1.0/big;
    }
    
    for (j = 0; j < n; j++) {
      for (i = 0; i < j; i++) {
        sum = m[i][j];
        for (k = 0; k < i; k++) {
          sum -= m[i][k]*m[k][j];
        }
        m[i][j] = sum;
      }

      big = 0.0;
      for (i = j; i < n; i++) {
        sum = m[i][j];
        for (k = 0; k < j; k++) { // 1=>0
          sum -= m[i][k]*m[k][j];
        }

        m[i][j] = sum;
        if ( (dum = vv.m[i][0]*fabs(sum)) >= big ) {
          big = dum;
          imax = i;
        }
      }

      if (j != (imax)) { //
        for (k = 0; k < n; k++) { // 1=>0
          dum = m[imax][k];
          m[imax][k] = m[j][k];
          m[j][k] = dum;
        }
        d = -d;
        vv.m[imax][0] = vv.m[j][0];
      }

      indx.m[j][0] = imax;

      if (m[j][j] == 0.0) {
        m[j][j] = TINY;
      }

      if (j != (n - 1)) {
        dum = 1.0/m[j][j];
        for (i = (j + 1); i < n; i++) {
          m[i][j] *= dum;
        } // end of if()
      } // end of if()
    } // end of for()
  } // end of if()
  
} //End of ludcmp()


// **************************************************************************
// Member Function: LU_Backsubstitution()
// PURPOSE: Compute LU Back substitution. 
//          Preparation for computation of Inverse Matrix.
// COMMENT: Modified lubksb() function from "Numerical Recipes in C"
// **************************************************************************
void MatrixMy::LU_Backsubstitution(MatrixMy &indx,
                                   MatrixMy &b)
{
  int i, ii = 0, ip, j;
  int n = row;
  double sum;
  
  if (row != column) {
    printf("ERROR: Matrix Dimension Not Match(LUB)\n");
  }
  else {
    
    for (i = 0; i < n; i++) {
      ip = (int)indx.m[i][0];
      sum = b.m[ip][0];
      b.m[ip][0] = b.m[i][0];
      
      if (ii != 0) {
        for (j = (ii - 1); j <= (i - 1); j++) {
          sum -= m[i][j]*b.m[j][0];
        }
      }
      else if (sum != 0.0) {
        ii = i + 1;
      }
      b.m[i][0] = sum;
    }
    
    for (i = (n - 1); i >= 0; i--) {
      sum = b.m[i][0];
      for (j = (i + 1); j < n; j++) {
        sum -= m[i][j]*b.m[j][0];
      }
      b.m[i][0] = sum/m[i][i];
    }
  }

} // End of LU_Backsubstitution()


// **************************************************************************
// Member function Inverse()
// PURPOSE: solve inverse matrix.
// **************************************************************************
MatrixMy MatrixMy::Inverse()
{
  MatrixMy temp(row, column);
  MatrixMy ans(row, column);
  MatrixMy col(row, 1);
  MatrixMy indx(row, 1);
  double d;
  int i, j, n = row; 
  
  if (row == column) {
    
    // copy matrix elements' value to temporal matrix
    for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
        temp.m[i][j] = m[i][j];
      }
    }

    temp.LU_Decomposition(indx, d);

    for (j = 0; j < n; j++) {
      for (i = 0; i < n; i++) {
        col.m[i][0] = 0.0;
      }
      col.m[j][0] = 1.0;

      temp.LU_Backsubstitution(indx, col);

      for (i = 0; i < n; i++) {
        ans.m[i][j] = col.m[i][0];
      }
    }
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(Inverse)\n");
  }

  return (ans);

} // End of Inverse()



// **************************************************************************
// Member function: Transpose()
// **************************************************************************
MatrixMy MatrixMy::Transpose()
{
  MatrixMy temp(column, row);
  int i, j;
  
  for (i = 0; i < temp.row; i++) {
    for (j = 0; j < temp.column; j++) {
      temp.m[i][j] = m[j][i];
    }
  }
  return (temp);
}


// **************************************************************************
// Member function: Outerproduct()
// **************************************************************************
MatrixMy MatrixMy::Outerproduct(MatrixMy &x)
{
  MatrixMy temp(3, 1);

  if ( (row == x.row) && (column == x.column) &&
       (x.row == 3)   && (x.column == 1) ) {
    temp.m[0][0] = m[1][0]*x.m[2][0] - m[2][0]*x.m[1][0];
    temp.m[1][0] = m[2][0]*x.m[0][0] - m[0][0]*x.m[2][0];
    temp.m[2][0] = m[0][0]*x.m[1][0] - m[1][0]*x.m[0][0];
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(outerproduct)\n");
  }

  return (temp);

} // End of Outerproduct()


// **************************************************************************
// Member function: Innerproduct()
// **************************************************************************
double MatrixMy::Innerproduct(MatrixMy &x)
{
  double ans = 0.0;

  if ( (row == x.row) && (column == x.column) &&
       (x.row == 3)   && (x.column == 1)) {
    ans = m[0][0]*x.m[0][0] + m[1][0]*x.m[1][0] + m[2][0]*x.m[2][0];
    ans = sqrt(ans);
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(innerproduct)\n");
  }

  return (ans);

} // End of Innerproduct()


// **************************************************************************
// Member function: Set_Value()
// COMMENT: Set Matrix elements
// **************************************************************************
double &MatrixMy::Set_Value(int r, int c) const
{
  if ( (r < row) &&
       (c < column) ) {
    return ( m[r][c] );
  }
  else {
    fprintf(stderr, "ERROR: Matrix Dimension Not Match(set value)\n");
    exit(1);

    return m[0][0];   // keep some compilers from complaining
  }
}


// **************************************************************************
// Member function: Get_Value()
// COMMENT: Set Matrix elements
// **************************************************************************
double MatrixMy::Get_Value(int r, int c)
{
  if ( (r < row) &&
       (c < column) ) {
    return ( m[r][c] );
  }
  else {
    fprintf(stderr,
            "MatrixMy::Get_Value() error: Matrix Dimension Not Match\n");
    exit(1);

    return 0.0;   // keep some of the compilers from complaining
  }
}


// **************************************************************************
// Member function: Set_Vector3x1()
// COMMENT: Set 3 by 1 vector
// **************************************************************************
void MatrixMy::Set_Vector3x1(double x, double y, double z)
{
  if ( (row == 3) &&
       (column == 1) ) {
    Set_Value(0, 0) = x;
    Set_Value(1, 0) = y;
    Set_Value(2, 0) = z;
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set vector)\n");
    exit(1);
  }
}

// **************************************************************************
// Member function: Set_Vector4x1()
// COMMENT: Set 4 by 1 Vector for Homogeneous Transformation
// **************************************************************************
void MatrixMy::Set_Vector4x1(double x, double y, double z)
{
  if ( (row == 4) &&
       (column == 1) ) {
    Set_Value(0, 0) = x;
    Set_Value(1, 0) = y;
    Set_Value(2, 0) = z;
    Set_Value(3, 0) = 1.0;
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set vector)\n");
    exit(1);
  }
}


// **************************************************************************
// Member function: Set_Hr_Matrix()
// COMMENT: Set Hr Rotation Matrix(3 by 3)
//          X-Y-Z fixed angle, R_XYZ
//          All rotation occur about axes of reference(WORLD) frame
// INPUT: rotation anlge about WORLD coordinates axes
// **************************************************************************
void MatrixMy::Set_Hr_Matrix(double roll, double elevation, double azimuth)

{
  if ( (row == 3) &&
       (column == 3) ) {

    double cz = cos(azimuth);
    double sz = sin(azimuth);
    double cy = cos(elevation);
    double sy = sin(elevation);
    double cx = cos(roll);
    double sx = sin(roll);

    Set_Value(0, 0) = (cz*cy);
    Set_Value(1, 0) = (sz*cy);
    Set_Value(2, 0) = (-sy);
    
    Set_Value(0, 1) = (cz*sy*sx - sz*cx);
    Set_Value(1, 1) = (sz*sy*sx + cz*cx);
    Set_Value(2, 1) = (cy * sx);
    
    Set_Value(0, 2) = (cz*sy*cx + sz*sx);
    Set_Value(1, 2) = (sz*sy*cx - cz*sx);
    Set_Value(2, 2) = (cy*cx);
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set Hr matrix)\n");
  }
} // End of Set_Hr_Matrix()


// **************************************************************************
// Member function
// COMMENT: Set Hr Rotation Matrix(4 by 4)
//          X-Y-Z fixed angle, R_XYZ
//          All rotation occur about axes of reference(WORLD) frame
// **************************************************************************
void MatrixMy::Set_H_Matrix(double x, double y, double z,
                            double roll, double elevation, double azimuth)
{
  if ( (row == 4) &&
       (column == 4) ) {

    double cx = cos(roll);
    double sx = sin(roll);
    double cy = cos(elevation);
    double sy = sin(elevation);
    double cz = cos(azimuth);
    double sz = sin(azimuth);

    Set_Value(0, 0) = (cz*cy);
    Set_Value(1, 0) = (sz*cy);
    Set_Value(2, 0) = (-sy);
    Set_Value(3, 0) = 0.0;
    
    Set_Value(0, 1) = (cz*sy*sx - sz*cx);
    Set_Value(1, 1) = (sz*sy*sx + cz*cx);
    Set_Value(2, 1) = (cy * sx);
    Set_Value(3, 1) = 0.0;
    
    Set_Value(0, 2) = (cz*sy*cx + sz*sx);
    Set_Value(1, 2) = (sz*sy*cx - cz*sx);
    Set_Value(2, 2) = (cy*cx);
    Set_Value(3, 2) = 0.0;
    
    Set_Value(0, 3) = x;
    Set_Value(1, 3) = y;
    Set_Value(2, 3) = z;
    Set_Value(3, 3) = 1.0;
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set H matrix)\n");
  }
} // End of Set_H_Matrix()


// **************************************************************************
// Member function: Set_DH_Matrix()
// COMMENT: Set Joint Transformation Matrix(4 by 4)
//          X-Y-Z fixed angle, R_XYZ
//          All rotation occur about axes of reference(WORLD) frame
// **************************************************************************
void MatrixMy::Set_DH_Matrix(double twist_angle,
                             double link_length,
                             double displacement,
                             double joint_angle)
{
  if ( (row == 4) &&
       (column == 4) ) {

    double ct, st;
    double cj, sj;
    
    ct = cos(twist_angle);  st = sin(twist_angle);
    cj = cos(joint_angle);  sj = sin(joint_angle);
    
    Set_Value(0, 0) =  cj;
    Set_Value(1, 0) =  sj*ct;
    Set_Value(2, 0) =  sj*st;
    Set_Value(3, 0) =  0.0;

    Set_Value(0, 1) = -sj;
    Set_Value(1, 1) =  cj*ct;
    Set_Value(2, 1) =  cj*st;
    Set_Value(3, 1) =  0.0;
    
    Set_Value(0, 2) =  0.0; 
    Set_Value(1, 2) = -st;
    Set_Value(2, 2) =  ct;
    Set_Value(3, 2) =  0.0;
    
    Set_Value(0, 3) = link_length;
    Set_Value(1, 3) =-st*displacement;
    Set_Value(2, 3) = ct*displacement;
    Set_Value(3, 3) = 1.0;
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set DH matrix)\n");
  }
} // End of Set_DH_Matrix()


// **************************************************************************
// Member function
// COMMENT: Set Jacobian Matrix(3 by 3)
// **************************************************************************
void MatrixMy::Set_J_Matrix(double theta0,
                            double theta1,
                            double theta2,
                            double theta3)
{
  if ( (row == 3) &&
       (column == 3) ) {

    double c2, s2;
    double c01, s01, c23, s23;

    c2 = cos(theta2);  s2 = sin(theta2);
    c01 = cos(theta0 + theta1);  s01 = sin(theta0 + theta1);
    c23 = cos(theta2 + theta3);  s23 = sin(theta2 + theta3);
    
    Set_Value(0, 0) = -s01*(LINK1 + LINK2*c2 + LINK3*c23);
    Set_Value(1, 0) =  c01*(LINK1 + LINK2*c2 + LINK3*c23);
    Set_Value(2, 0) =  0.0;

    Set_Value(0, 1) = -c01*(LINK2*s2 + LINK3*s23);
    Set_Value(1, 1) = -s01*(LINK2*s2 + LINK3*s23); 
    Set_Value(2, 1) = -    (LINK2*c2 + LINK3*c23);
    
    Set_Value(0, 2) = -LINK3*c01*s23;
    Set_Value(1, 2) = -LINK3*s01*s23;
    Set_Value(2, 2) = -LINK3*c23;
  }
  else {
    printf("ERROR: Matrix Dimension Not Match(set J matrix)\n");
  }
}


// **************************************************************************
// Member function: PrintMatrix()
// COMMENT: print out matrix elements
// **************************************************************************
void MatrixMy::PrintMatrix()
{
  int i, j;

  for (i = 0; i < row; i++) {
    for (j = 0; j < column; j++) {
      printf("%f ", m[i][j]);
    }
    printf("\n");
  }
  printf("\n");
}  


// **************************************************************************
void nrerror(char error_text[])
{
  fprintf(stderr, "Numerical Recipes run time error...\n");
  fprintf(stderr, "%s\n", error_text);
  fprintf(stderr, "...now exiting to system...\n");
  exit(1);

} // End of nrerror()


// EOF
