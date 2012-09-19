
#include "QPsolver.h"



int main()
{
	cout<<"hello!";
	QPsolver s1;
	
	MatrixXF Q(2,2);
	Q<<1, -1, -1, 2;
	VectorXF c(2);
	c << -2, -6;
	double cfix = 0;
	MatrixXF A(3,2);
	A << 1, 1, -1, 2, 2, 1;
	VectorXF blv = VectorXF::Zero(2);
	VectorXF buv(2);
	buv<<  +MSK_INFINITY, +MSK_INFINITY;
	VectorXbk bkv(2);
	bkv << MSK_BK_LO, MSK_BK_LO;


	VectorXF buc(3);
	buc<<2,2,3;
	VectorXF blc(3);
	blc<<-MSK_INFINITY, -MSK_INFINITY, -MSK_INFINITY;
	VectorXbk bkc(3);
	bkc<< MSK_BK_UP, MSK_BK_UP, MSK_BK_UP;

	s1.UpdateObjective(Q, c, cfix) ;
	s1.UpdateConstraintMatrix(A);
	s1.UpdateConstraintBounds(bkc, blc, buc);
	s1.UpdateVariableBounds(bkv, blv, buv );

	s1.InspectQPproblem();

	s1.Optimize();

	// cout<<"+MSK_INFINITY  = "<<setprecision (35)<< +MSK_INFINITY +1.1 << "    sizeof(MSK_INFINITY) = "<<sizeof(MSK_INFINITY)<<endl;
	return 1;
}
