
#include "control_globals.h"
#include "QPsolver.h"
#include "PrioritizedController.h"
#include "mosek.h"

#define DEBUG_2

class SimpleTestController: public PrioritizedController
{
public:
	SimpleTestController();
	virtual ~SimpleTestController();
	virtual void Reset();
	virtual void SetInitialConstraintBounds();
	virtual void SetInitialVariableBounds();
	virtual void UpdateConstraintMatrix();
	virtual void UpdateConstraintBounds();
	virtual void UpdateVariableBounds();

	
};

SimpleTestController::SimpleTestController()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController Constructor --"<<endl;
#endif 
}

SimpleTestController::~SimpleTestController()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController Destructor --"<<endl;
#endif 
}

void SimpleTestController::Reset()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::Reset() --"<<endl;
#endif 
	D0.resize(1,3);
	c_bk.resize(1);
	c_bl.resize(1);
	c_bu.resize(1);

	dBar.resize(3);
	dBar.setZero();
	CBar= MatrixXd::Identity(3,3);

	TaskJacobians.resize(2);
	TaskBiases.resize(2);

	MatrixXF A1(1,3);
	A1<< 0, 0, 1;
	VectorXF b1(1);
	b1<<0;
	TaskJacobians[0] = A1;
	TaskBiases[0]=b1;

	MatrixXF A2(2,3);
	A2<< 1, 0, 0,
         0, 1, 0;
	TaskJacobians[1] = A2;
	VectorXF b2(2);
	b2 <<3,1;
	TaskBiases[1]= b2;

}

void SimpleTestController::SetInitialConstraintBounds()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::SetInitialConstraintBounds() --"<<endl;
#endif 
	c_bk(0) = MSK_BK_FX;
	c_bl(0) = 1;
	c_bu(0) = 1;

}


void SimpleTestController::SetInitialVariableBounds()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::SetInitialVariableBounds() --"<<endl;
#endif 


}


void SimpleTestController::UpdateConstraintMatrix()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::UpdateConstraintMatrix() --"<<endl;
#endif 
	D0<< 1,1,1;
	solver.UpdateConstraintMatrix(D0*CBar);
#ifdef DEBUG_2
	cout<<" D0 = "<<endl<<D0<<endl;
#endif 
}

void SimpleTestController::UpdateConstraintBounds()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::UpdateConstraintBounds() --"<<endl;
#endif 
	solver.UpdateConstraintBounds(c_bk, c_bl-D0*dBar, c_bu-D0*dBar    );
}

void SimpleTestController::UpdateVariableBounds()
{
#ifdef DEBUG_2
	cout<<" -- SimpleTestController::UpdateVariableBounds() --"<<endl;
#endif 
	int a = CBar.cols();
	v_bk = VectorXbk::Constant(a, MSK_BK_FR);
	v_bl = VectorXF::Constant(a, 0);
	v_bu = VectorXF::Constant(a, 0);

	solver.UpdateVariableBounds(v_bk, v_bl, v_bu    );
}



int main()
{
	cout<<"Simple Test Case for my prioritized controller!";

	
	SimpleTestController stc;


	stc.Reset();
	stc.SetInitialConstraintBounds();
	stc.SetInitialVariableBounds();

	//MatXFVector Qm(2);
	//Qm[0] = MatrixXF::Zero(3,3);
	//Qm[1] = MatrixXF::Zero(2,2);

	for(int i = 0; i<stc.TaskJacobians.size();i++)
	{
		cout<<"\n ------- Task "<<i<<" -------"<<endl;
		stc.UpdateObjective(stc.TaskJacobians[i], stc.TaskBiases[i]); //, Qm[i]	
		stc.UpdateConstraintMatrix();
		stc.UpdateConstraintBounds();
		stc.UpdateVariableBounds();
		stc.solver.InspectQPproblem();
		stc.OptimizeSingleTier();

		cout<<" === "<<stc.dBar.transpose()<<endl;
	}


	return 1;
}
