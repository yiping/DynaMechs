//  math_funcs.cpp
//  Aug 23, 2012
//  YL
#include "math_funcs.h"

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
		cout<<"eigenvalues are:"<<endl<<es.eigenvalues()<<endl;
		int p_count = 0 , n_count = 0;
		int zero_count = 0;
		double d;
		for (int i=0; i<A.cols();i++)
		{		
			d = real(eigenval(i));//es.eigenvalues()[i]
			//cout<<"d_"<<i<<" ="<<d<<endl;
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


// A is square matrix
void solveInverse(const MatrixXF & A, const VectorXF & rhs, VectorXF & x)
{
	// a positive definite matrix is always nonsingular.
	x = A.partialPivLu().solve( rhs );
}

void solvePseudoInverse(const MatrixXF & A, const VectorXF & rhs, VectorXF & x)
{
	x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(rhs);
}
