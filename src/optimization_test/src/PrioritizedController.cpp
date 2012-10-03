
//  PrioritizedController.cpp
//	9/19/2012



#include "PrioritizedController.h"


#define DEBUG_PC





PrioritizedController::PrioritizedController() 
{
#ifdef DEBUG_PC
	cout<<" -- PriorizedController Constructor --"<<endl;
#endif 
	


}

PrioritizedController::~PrioritizedController() 
{
#ifdef DEBUG_PC
	cout<<" -- PriorizedController Destructor --"<<endl;
#endif 


}


void PrioritizedController::UpdateObjective(const MatrixXF &TaskJacobian, const VectorXF &TaskBias) //, const MatrixXF &Qmodifier
{

	MatrixXd JtBar = TaskJacobian * CBar;
	getNullSpace(JtBar, C);

	if ( isFullColumnRank(JtBar))
	{
		//cout<<"\nJtBar is full rank! \n"<<endl;
		JtBarFullColRank = true;
	}


	MatrixXd JtBarT = JtBar.transpose();

	MatrixXd Q = JtBarT * JtBar; 

	//Q += Qmodifier;

	VectorXd btBar = TaskBias - TaskJacobian * dBar; 

	VectorXd c = -JtBarT*btBar; 

	double cfix = 0.5*btBar.dot(btBar);

#ifdef DEBUG_PC
	// check for the rank of JtBar
	cout<<endl<<"Objectives: "<<endl;

	IOFormat DisplayFmt(FullPrecision, 0, "  ","\n", "    ", " ", "", "\n");
	cout<<"JtBarColFullRank: "<<JtBarFullColRank<<endl;
	cout<<"CBar =  "<<endl<<CBar.format(DisplayFmt)<<endl;
	cout<<"JtBar = "<<endl<<JtBar.format(DisplayFmt)<<endl;
	cout<<"btBar = "<<endl<<btBar.transpose().format(DisplayFmt)<<endl;
	cout<<"Q = "<<endl<<Q.format(DisplayFmt)<<endl;
	cout<<"Null space of JtBar: "<<endl<<C.format(DisplayFmt)<<endl;
	cout<<"Current JtBar has rank "<<showRank(JtBar)<<endl<<endl;
#endif 

	solver.UpdateObjective(Q, c, cfix);

}


void PrioritizedController::UpdateConstraintMatrix()
{
	solver.UpdateConstraintMatrix(CBar);
}




void PrioritizedController::UpdateConstraintBounds()
{
	solver.UpdateConstraintBounds(c_bk, c_bl - dBar, c_bu - dBar    );
}


void PrioritizedController::UpdateVariableBounds() 
{

	// every variable is free, but you still need to tell mosek that

	solver.UpdateVariableBounds(v_bk, v_bl, v_bu);



}




void PrioritizedController::OptimizeSingleTier() 
{
#ifdef DEBUG_PC
	cout<<" -- Optimizing ... --"<<endl;
#endif 
	solver.Optimize();

	if (solver.solnStatus == 0 )
	{
		dBar = dBar + CBar * solver.xx;
		isFeasible = true;
	}
	else
	{
		isFeasible = false;
		cout<<"Infeasible solution ... exiting "<<endl;
		exit(1);
	}

	if (!JtBarFullColRank )
	{
		
	}

	CBar = CBar * C; 

	solver.Reset();
}





