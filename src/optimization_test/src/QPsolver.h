// QPsolver.h
// Aug 23, 2012
// YL

//! A wrap-up of Mosek functions
#ifndef __QPSOLVER_H__
#define __QPSOLVER_H__



#include "mosek.h" 
#include "dm.h"
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include "control_globals.h"

using namespace std;
using namespace Eigen;

typedef Matrix<MSKboundkeye, Dynamic, 1> VectorXbk;

class QPsolver
{
public:
	QPsolver();
	~QPsolver();
	void Optimize();

	// QP problem inspection
	void InspectQPproblem();

	VectorXF xx; //optimal solution

	// Changing QP problem specification
	void UpdateObjective(const MatrixXF & Q, const VectorXF &c, double cfix) ;
	void UpdateConstraintMatrix(const MatrixXF &A);
	void UpdateConstraintBounds(const VectorXbk &bkc, const VectorXF &blc, const VectorXF &buc);
	void UpdateVariableBounds(const VectorXbk &boundkey_var, const VectorXF &lowerbound_var, const VectorXF &upperbound_var);

	void ModifySingleConstraintBound(int index, MSKboundkeye bkey, double new_lb, double new_ub);
	void ModifySingleVariableBound(int index, MSKboundkeye bkey, double new_lb, double new_ub);
	void Reset();

	int solnStatus; 
protected:


	MSKenv_t env; 
	MSKtask_t task; 
	MSKrescodee r; 

	MSKidxt m_num_con;
	MSKidxt m_num_var;
	MSKidxt m_num_qnz;
};

#endif
