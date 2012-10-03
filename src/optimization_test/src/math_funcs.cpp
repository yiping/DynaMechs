//  math_funcs.cpp
//  Aug 23, 2012
//  YL
#include "math_funcs.h"

// math functions
void getNullSpace(const MatrixXd &A, MatrixXd &C)
{
	JacobiSVD<MatrixXd> svdA(A, ComputeFullU | ComputeFullV);
	MatrixXd V = svdA.matrixV();	
	int m = svdA.nonzeroSingularValues();
	int n = V.cols();
	C = V.rightCols(n - m);

	

	IOFormat OctaveFmt(FullPrecision, 0, ", ", ";\n", "", "", "[", "]\n");
	//cout<<"U = "<<endl<<svdA.matrixU().format(OctaveFmt) <<endl;
	MatrixXd S = MatrixXd(svdA.singularValues().asDiagonal());
	MatrixXd Sfull = MatrixXd::Zero(svdA.matrixU().cols(), svdA.matrixV().rows());
	Sfull.block(0,0, S.rows(), S.cols()) = S;
	//cout<<"S = "<<endl<<Sfull.format(OctaveFmt)<<endl;
	//cout<<"V = "<<endl<<svdA.matrixV().format(OctaveFmt) <<endl;
	
	//cout<< " NullSpace of A is composed of "<<endl << C.format(OctaveFmt) <<endl;
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


int showRank(const MatrixXd &A)
{
	// Another application of the SVD is that it provides an explicit representation of the range and null space of a matrix M. 
	// The right-singular vectors corresponding to vanishing singular values of M span the null space of M. 
	// The left-singular vectors corresponding to the non-zero singular values of M span the range of M. 
    // => the rank of M equals the number of non-zero singular values 

	// rank-revealing: please pay attention to the precision: 
	// in Eigen3 implementation, JacobiSVD result accuracy depends on the machine precision. float and double make a big difference.

	JacobiSVD<MatrixXd> svdA(A, ComputeThinU | ComputeThinV);
	// cout << "The rank of A is " << svdA.nonzeroSingularValues()<<endl;
	VectorXd sing_val_vec = svdA.singularValues() ;

	int count = 0;
	for (int i=0; i<sing_val_vec.size(); i++)
	{
		//cout<<"singular value "<< i<<" is: "<<sing_val_vec(i)<<endl;
		if (sing_val_vec(i) >1e-14 )
		{
			count ++;
		}
	}

	return count;
}

bool isFullRank(const MatrixXd &A)
{
	int r = showRank(A);
	bool isFullRank;
	if ( r==A.cols() || r==A.rows() )
	{
		isFullRank = true;
	}	
	else
	{
		isFullRank = false;
	}

	return isFullRank;

}

bool isFullColumnRank(const MatrixXd &A)
{
	bool isFullColRank = false;
	int c = A.cols();
	int r = showRank(A);
	if (c==r)
	{
		isFullColRank = true;
	}

	return isFullColRank;
}



