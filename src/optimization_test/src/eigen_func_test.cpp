#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/LU>
 #include <Eigen/Eigenvalues> 

#include "mosek.h" 
//#include "/usr/include/eigen3/Eigen/src/Core/IO.h"

using namespace std;
using namespace Eigen;

typedef double Float;
typedef Matrix<double, Dynamic , 1 > VectorXF;
typedef Matrix<double, Dynamic, Dynamic> MatrixXF;
typedef Matrix<MSKboundkeye, Dynamic, 1> VectorXbk;

/* globals */


// math functions
void getNullSpace(const MatrixXF &A, MatrixXF &C)
{
	JacobiSVD<MatrixXF> svdA(A, ComputeFullU | ComputeFullV);
	MatrixXF V = svdA.matrixV();	
	int n = svdA.nonzeroSingularValues();
	C = V.rightCols(n);

	IOFormat OctaveFmt(FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
	cout<< " NullSpace of A is composed of "<<endl << C.format(OctaveFmt) <<endl;
}

void showDefiniteness(const MatrixXF &A )
{
	if (A.cols() != A.rows())
	{
		cout<< "matrix is not square!!"<<endl;
		exit(1);
	}
	else
	{
		EigenSolver<MatrixXF> es(A);
		typedef Matrix<complex<Float>, Dynamic, 1> ComplexVec;
	
		const ComplexVec& eigenval = es.eigenvalues();
		cout<<es.eigenvalues()<<endl;
		int p_count = 0 , n_count = 0;
		int zero_count = 0;
		double d;
		for (int i=0; i<A.cols();i++)
		{		
			d = real(eigenval(i));//es.eigenvalues()[i]
			cout<<"d_"<<i<<" ="<<d<<endl;
			if (d > 0)
				p_count ++;
			if (d <0 )
				n_count ++;
			if (d == 0)
				zero_count ++;
		}		

		if ( (p_count > 0) && (n_count ==0 ) &&(zero_count == 0)  )
			cout<<"matrix is PD"<<endl;
		else if ( (p_count > 0) && (n_count ==0 ) &&(zero_count > 0)  )
			cout<<"matrix is PSD"<<endl;
		else if ( (p_count == 0) && (n_count > 0 ) &&(zero_count == 0) )
			cout<<"matrix is ND"<<endl;
		else if ( (p_count == 0) && (n_count > 0 ) &&(zero_count > 0)  )
			cout<<"matrix is NSD"<<endl;	
		else
			cout<<"matrix is indefinite"<<endl;
	}
}

int showRank(const MatrixXd &A)
{

	JacobiSVD<MatrixXd> svdA(A, ComputeThinU | ComputeThinV);
	// cout << "The rank of A is " << svdA.nonzeroSingularValues()<<endl;
	VectorXd sing_val_vec = svdA.singularValues() ;

	int count = 0;
	for (int i; i<sing_val_vec.size(); i++)
	{
		//cout<<"singular value "<< i<<" is: "<<sing_val_vec(i)<<endl;
		if (sing_val_vec(i) >1e-14 )
		{
			count ++;
		}
	}

	return count;
}



int main()
{

	//MatrixXf m = MatrixXf::Random(3,2);
	MatrixXd m;
	m.resize(3,4);
	m<<1, 0, 2, 3,
       0, 1, 4, 5,
       0, 0, 0, 0;

/*		m.resize(3,2);
	m<<   0.68,  0.597,
		-0.211,  0.823,
		 0.566, -0.605;*/

	//IOFormat OctaveFmt(12, DontAlignCols, ", ", ";\n", "", "", "[", "]");
	IOFormat OctaveFmt(FullPrecision, 0, ", ", ";\n", "", "", "[", "]");

	cout << "Here is the matrix m:" << endl << m << endl;
	JacobiSVD<MatrixXd> svd(m, ComputeFullU | ComputeFullV);
	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	cout << "U----- " << endl << svd.matrixU().format(OctaveFmt) << endl;
	cout << "V----- " << endl << svd.matrixV().format(OctaveFmt) << endl;
	cout << "number of non-zero singular values: "<< endl << svd.nonzeroSingularValues()<<endl;

	cout<<"Machine epsilon: "<<NumTraits<MatrixXd::Scalar>::epsilon()<<endl;

	MatrixXF C;
	getNullSpace(m,C);
	
	cout<<"MSK_MAX_STR_LEN == "<<MSK_MAX_STR_LEN<<endl;

	MatrixXF x(2,4);
	x<<1,2,3,4,5,6,7,8;
	cout<<"x ="<<x<<endl;	
	double* xptr = x.data();
	cout<<"x[5]="<<xptr[5]<<endl;
	VectorXF y(6);
	y<<1.2,1.3,1.4,0,0,0;
	cout<<y<<endl;
	cout<<"y[2]="<<setw(20)<<y[2]<<endl;

	double xx[5]={1,3,5,7,10};
	Map<VectorXF> v(xx, 5);
	cout<<"c =" << v.transpose()<<endl; 

	//MatrixXF z(3,3);
	//z<<    -1, 2, 0,
    // 		2, -4,  0,
    // 		0,  0, -8;

	MatrixXF z(3,3);
	z<<    1, 2, 0,
     		2, -18,  0,
     		0,  0, 8;
	cout<< "Now we have matrix z:"<<endl<< z <<endl;

	/*EigenSolver<MatrixXF> es(z);
	typedef Matrix<complex<double>, Dynamic, 1> ComplexVec;
	
	const ComplexVec& eigenval = es.eigenvalues();
	cout<<es.eigenvalues()<<endl;
	*/

	cout<<"showDefiniteness function test:"<<endl;
	showDefiniteness(z);

	MatrixXF J = MatrixXF::Constant(6,8,1);

	cout<<"Matrix J is "<<endl<<J<<endl;

	J.setZero(10, 10);
	cout<<"after setZero(), Matrix J now is "<<endl<<J<<endl;
	
	J = - MatrixXF::Constant(4,3,7);
	cout<<"J has changed again, J now is "<<endl<<J<<endl;

	//VectorXF d0(10);
	VectorXF d0 = VectorXF::Zero(10);
	d0.segment(0, 2) = VectorXF::Zero(2);
	d0.segment(2, 3) = 3*VectorXF::Ones(3);
	d0.segment(5, 5) = VectorXF::Ones(5);

	cout<<" d0 = "<<endl;
	cout<<d0<<endl;


   	// rank-revealing: please pay attention to the precision: svd result accuracy depends on the machine precision. float and double make a big difference.

	//MatrixXd AA(3,3); 
   	//AA << 1, 2, 5,
    //    2, 1, 4,
     //   3, 0, 3;

	//AA << 1, 2, 1,
    //     -2, -3, 1,
    //      3, 5, 0;

	MatrixXd AA(4,4);
	AA<<2, 4, 1, 3, -1, -2, 1, 0, 0, 0, 2, 2, 3, 6, 2, 5;

	// Another application of the SVD is that it provides an explicit representation of the range and null space of a matrix M. 
	// The right-singular vectors corresponding to vanishing singular values of M span the null space of M. 
	// The left-singular vectors corresponding to the non-zero singular values of M span the range of M. 
    // => the rank of M equals the number of non-zero singular values 
   	cout << "Here is the matrix A:\n" << AA << endl;
  	//FullPivLU<Matrix3f> lu_decomp(AA);
   	//cout << "The rank of A is " << lu_decomp.rank() << endl;
	JacobiSVD<MatrixXd> svdAA(AA, ComputeFullU | ComputeFullV);
	cout << "The rank of A is " << svdAA.nonzeroSingularValues()<<endl;
	cout << "Its singular values are:" << endl << svdAA.singularValues() << endl;
	cout << "U matrix:" << endl << svdAA.matrixU() << endl;
	cout << "V matrix:" << endl << svdAA.matrixV() << endl;

	VectorXd sing_vec = svdAA.singularValues();

	for (int i; i<sing_vec.size(); i++)
	{
		cout<<"singular value "<< i<<" is: "<<sing_vec(i)<<endl;
	}

	cout<<"my function shows the rank of A is: "<<showRank(AA)<<endl;

	//LLT<Matrix3f> llt;
	//llt.compute(z);
	//cout << llt.isPositiveDefinite() << endl;
	//cout << "done " << endl;
	
	//if (z.ldlt().isPositiveDefinite())
	//cout<<"It's PD."<<endl ;
	//else
	//	cout<<"It's NOT PD."<<endl;*/

	return 1;
}




