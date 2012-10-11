// QPsolver.cpp
// Aug 23, 2012
// YL

#include "QPsolver.h"
#include "math_funcs.h"




#define MAXNUMCON (NJ+6 + 6*NS )   // # of constraints           
#define MAXNUMVAR (NJ + NJ+6 +  NS*NP*NF)  // # of variables  // 6*NS +
#define MAXNUMANZ (MAXNUMCON * MAXNUMVAR )					  
#define MAXNUMQNZ (MAXNUMVAR * MAXNUMVAR)            


//#define DEBUG_MOSEK
//#define DEBUG_QP

static void MSKAPI printstr(void *handle, char str[]) 
{ 
#ifdef DEBUG_MOSEK
	printf("%s",str);
#endif 
} 


//------------------------------------------------------------
QPsolver::QPsolver()
{
#ifdef DEBUG_QP
	cout<<" -- initializing QP solver --"<<endl;
#endif
	solnStatus = -1;

	// Create the mosek environment. 
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL); 
	
	// Check whether the return code is ok. 
	if ( r==MSK_RES_OK )
	{
		// Directs the log stream to the 'printstr' function. 
		MSK_linkfunctoenvstream(env, MSK_STREAM_LOG,NULL,printstr);
	}
	
	// Initialize the environment.   
	r = MSK_initenv(env);
	if ( r==MSK_RES_OK )
	{ 
		// Create an empty optimization task for later use 
		r = MSK_maketask(env,MAXNUMCON,MAXNUMVAR,&task); // MAXNUMVAR (MAXNUMCON) can be 0 if no such estimate is known.											
		
		if ( r==MSK_RES_OK )
		{
			// connect a call-back function to the task log stream 
			r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
			
			// Give MOSEK an estimate of the size of the input data. 
			// This is done to increase the speed of inputting data. 
			// However, it is optional. 
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumvar(task,MAXNUMVAR);
			
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumcon(task,MAXNUMCON);
			
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumanz(task,MAXNUMANZ);// size of the preallocated storage for A (non-zero)

			if (r == MSK_RES_OK) 
				r = MSK_putmaxnumqnz(task,MAXNUMQNZ);
			
		}
	}

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 by default
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while initializing the QP solver.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
		exit(1);
	} 
}

//------------------------------------------------------------
QPsolver::~QPsolver() 
{
	cout<<" -- QP solver destructor --"<<endl;
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
}




//------------------------------------------------------------
void QPsolver::InspectQPproblem()
{
	MSKintt ncon_[1];
	MSKintt nvar_[1]; 
	MSKintt nqonz_[1];
	MSKintt ncon, nvar, nqonz;

	r = MSK_getnumcon(task, ncon_);
	ncon= ncon_[0];
	if ( r==MSK_RES_OK )
	{ 
		cout<<endl<<"============================================="<<endl;
		cout<<"bound key code:"<<endl;
		cout<<"MSK_BK_FR: "<<MSK_BK_FR<<endl;
		cout<<"MSK_BK_RA: "<<MSK_BK_RA<<endl;
		cout<<"MSK_BK_LO: "<<MSK_BK_LO<<endl;
		cout<<"MSK_BK_UP: "<<MSK_BK_UP<<endl;
		cout<<"MSK_BK_FX: "<<MSK_BK_FX<<endl;

		cout<<endl<<"Current QP problem : "<<endl;
		cout<<"# of constraints: "<< ncon<<endl;

		r =	MSK_getnumvar(task, nvar_);
		nvar = nvar_[0];
		if ( r==MSK_RES_OK )
		{ 
			cout<<"# of variables: "<<nvar<<endl;
		}
	}

	// show objective (Q and c)
	if ( r==MSK_RES_OK )
	{ 
		r = MSK_getnumqobjnz(task, nqonz_);
		nqonz = nqonz_[0];
		MSKintt qosub_i[nqonz];
		MSKintt qosub_j[nqonz];
		double qo_val[nqonz];
		double co[nvar];

		if ( r==MSK_RES_OK )
		{
			MSKintt n1[1];
			MSKintt n2[1];	
			n1[0] = nqonz;
			n2[0] = nqonz;
			r = MSK_getqobj(task, nqonz, n1, n2, qosub_i, qosub_j, qo_val);
			MatrixXF m = MatrixXF::Zero(nvar,nvar);
			for (int k =0; k<nqonz; k++)
			{
				m(qosub_i[k], qosub_j[k]) = qo_val[k];
				if (qosub_i[k] != qosub_j[k])
				{
					m(qosub_j[k], qosub_i[k] ) = qo_val[k]; // rebuild symmetric
				}
			}

			cout<<endl<<"[objective]"<<endl <<"Quadratic coefficient matrix (Q) = "<<endl<<m<<endl;
			showDefiniteness(m);

			r = MSK_getc(task, co);
			if ( r==MSK_RES_OK )
			{
				Map<VectorXF> v(co, nvar);
				cout<<"Linear coefficient vector c =" << v.transpose()<<endl; 
			}
		}
	}

	// show Constraints (A and bounds)
	if ( r==MSK_RES_OK )
	{
		MSKintt i;
		MatrixXF A = MatrixXF::Zero(ncon, nvar);
		VectorXbk bkc_ = VectorXbk::Zero(ncon);
		VectorXF blc_ = VectorXF::Zero(ncon);
		VectorXF buc_ = VectorXF::Zero(ncon);
		for (i =0; (i< ncon) && (r==MSK_RES_OK); i++)
		{	
			MSKintt nzi[1];
			MSKintt subi[nvar];
			double vali[nvar];
			
		    MSKboundkeye bk_con[ncon];
    		double  bl_con[ncon];
    		double  bu_con[ncon];			

			r = MSK_getavec(task, MSK_ACC_CON, i, nzi, subi, vali);

			if ( r==MSK_RES_OK )
			{
				r = MSK_getbound(task, MSK_ACC_CON, i, bk_con+i, bl_con+i, bu_con+i); 
				if ( r==MSK_RES_OK )
				{
					for (int j =0; j< nzi[0];j ++)
						A(i,subi[j]) = vali[j];

					bkc_(i) = bk_con[i];
					blc_(i) = bl_con[i];
					buc_(i) = bu_con[i];					
				}
			}
		}

		if (r==MSK_RES_OK)
		{
			cout<<endl;
			cout<<"[Constraints]"<<endl;
			cout<<"Linear coefficient matrix (A) = "<<endl<< A<<endl;
			cout<<"Constraint bounds:"<<endl;
			cout<<setw(15)<<"LO_C"<<setw(15)<<"UP_C"<<setw(15)<<"KEY_C"<<endl;
			for (int i = 0; i<ncon; i++)
				cout<<setw(15)<<blc_(i)<<setw(15)<<buc_(i)<<setw(15)<<bkc_(i)<<endl;
		}
	}
	

	// show Variable bounds
	if ( r==MSK_RES_OK )
	{
	    MSKboundkeye bk_var[nvar];
		double  bl_var[nvar];
		double  bu_var[nvar];	

		MSKintt i;
		for (i =0; (i< nvar) && (r==MSK_RES_OK); i++)
		{	
			r = MSK_getbound(task, MSK_ACC_VAR, i, bk_var+i, bl_var+i, bu_var+i); 
		}
		
		if ( r==MSK_RES_OK )
		{
			Map<VectorXbk> bkv_(bk_var,nvar);
			Map<VectorXF>  blv_(bl_var,nvar);
			Map<VectorXF>  buv_(bu_var,nvar);	
			cout<<endl<<"[Variables]"<<endl;
			cout<<"Variable bounds: "<<endl;
			cout<<setw(15)<<"LO_V"<<setw(15)<<"UP_V"<<setw(15)<<"KEY_V"<<endl;
			for (int i = 0; i<nvar; i++)
				cout<<setw(15)<<blv_(i)<<setw(15)<<buv_(i)<<setw(15)<<bkv_(i)<<endl;	
			
			cout<<endl<<"============================================="<<endl<<endl;	
		}
		
	}


	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 by default
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while inspecting the current QP problem.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
		exit(1);
	} 
}







//------------------------------------------------------------

//! Update the optimization objective \n
//! \f$ \frac{1}{2} x^T Q x + c^T x + c_{\mbox{\tiny fix}} \f$  \n
//! \li Mosek assumes 1/2 in front of Q. Only lower triangular part of Q should be specified\n
//! \li Q is assumed symmetric. Only non-zero elements are specified. \n
//! \li However, in general case, we don't know which entries of Q are zeros, so we specify the entire lower triangle.\n  
//! This function will figure out the actual number of variables based on the size of supplied \f$ Q \f$ \n
void QPsolver::UpdateObjective(const MatrixXF &Q, const VectorXF & c, double cfix) 
{
	MSKidxt       qsubi[MAXNUMQNZ];
	MSKidxt       qsubj[MAXNUMQNZ];
	double        qval[MAXNUMQNZ];
	MSKidxt       i,j;

	// 
	if (Q.rows() != Q.cols())
	{
		cout<<"Supplied Q is not square!"<<endl;
		exit(1);
	}	

	if (Q.rows() != c.size())
	{
		cout<<"Supplied Q and c :  dimension mismatch!"<<endl;
		exit(1);
	}
	
	m_num_var = Q.rows();
#ifdef DEBUG_QP
	cout<<"m_num_var: "<<m_num_var<<endl;
#endif

	// Append variables. 
	// The variables will initially be fixed at zero (x=0)
	r = MSK_append(task,MSK_ACC_VAR,m_num_var);

	// Add a constant Term to the objective
	if(r == MSK_RES_OK) 
	{
		r = MSK_putcfix(task, cfix );
	}

	// Set the linear term c_j in the objective
	const double* cc = c.data();
	for(j=0; j<m_num_var && r == MSK_RES_OK; ++j) 
	{	
		// cout<< j <<endl;
		r = MSK_putcj(task,j,cc[j]);
	}

	if(r == MSK_RES_OK) 
	{
		MSKidxt k = 0;
		for (i = 0; i < m_num_var; i++) 
		{
			for (j = 0; j<=i; j++) 
			{
				//cout<<"Q("<<i<<","<<j<<")"<<Q(i,j)<<endl;
				if (abs(Q(i,j)) >= 1e-10) 
				{
					qsubi[k]=i; 
					qsubj[k]=j; 
					qval[k]=Q(i,j);
					k++;
				}
			}
		}
		m_num_qnz = k;
		//cout<<"m_num_qnz = "<<m_num_qnz<<endl;
		r = MSK_putqobj(task,m_num_qnz,qsubi,qsubj,qval);
#ifdef DEBUG_QP
		cout<<"done putting qobj"<<endl;
#endif
	}

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while updating the objective.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	} 

}




//------------------------------------------------------------

//! Update the linear constraint coefficient matrix (\f$ A \f$) row by row \n
//! \f$  l_k^c \leq a_k^T x \leq u_k^c  \;\;\;\; k=0,1,\cdots,m-1 (\mbox{row index})\f$ \n
//! This function will figure out the actual number of cosntraints based on the number of rows of supplied \f$ A \f$ \n
void QPsolver::UpdateConstraintMatrix(const MatrixXF & A) 
{
	MSKidxt	asub[m_num_var];
	double  aval[m_num_var];

	if (A.cols() != m_num_var)
	{
		cout<<"The number of columns of A needs to be equal to the number of variables."<<endl;
		exit(1);
	}

	m_num_con = A.rows();
#ifdef DEBUG_QP
	cout<<"m_num_con: "<<m_num_con <<endl;
#endif

	// Append 'NUMCON' empty constraints. The constraints will initially have no bounds. 
	r = MSK_append(task,MSK_ACC_CON,m_num_con);

	for(MSKidxt i=0; i< m_num_con && r == MSK_RES_OK; ++i) // # of CON 
	{
		MSKidxt k=0;
		for (MSKidxt j=0; j<m_num_var; j++) // # of VAR
		{
			if (abs(A(i,j)) > 1e-10) 
			{
				asub[k]=j;
				aval[k]=A(i,j);
				k++;
			}
		}
		
		r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		// cout<<"done putting a row of A"<<endl;
	}

#ifdef DEBUG_QP
	if (r == MSK_RES_OK)
	{
		cout<<"done putting all rows of A"<<endl;
	}
#endif

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while updating the linear constraint matrix.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	} 
}



//------------------------------------------------------------
//! Update the bounds for constraints.
void QPsolver::UpdateConstraintBounds(const VectorXbk & bkc, const VectorXF & blc, const VectorXF & buc )
{
	if (   ! ( (bkc.size()==blc.size()) && (buc.size()==blc.size()) )    )
	{
		cout<<"Constraint bound: vectors of boundkey, lowerbound and upperbound should have the same size."<<endl;
		exit(1);
	}

	if ( bkc.size()!= m_num_con )
	{
		cout<<"Constraint bounds: dimension mismatch."<<endl;
		exit(1);
	}

	MSKidxt i;

	// Bounds on Constraints
	for (i=0; i<m_num_con && r == MSK_RES_OK; i++) // # of constraints 
	{
		r = MSK_putbound(task, MSK_ACC_CON, i, bkc[i], blc[i], buc[i]);
		// cout<<"done putting a row of constraint bounds"<<endl;
	}

#ifdef DEBUG_QP
	if (r == MSK_RES_OK)
	{
		cout<<"done putting all rows of constraint bounds"<<endl;
	}
#endif

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while setting up the constraint bounds.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	} 

} 


//------------------------------------------------------------
//! Update the bounds for variables.
void QPsolver::UpdateVariableBounds(const VectorXbk &boundkey_var, const VectorXF &lowerbound_var, const VectorXF &upperbound_var ) 
{

	if (   ! ( (boundkey_var.size()==lowerbound_var.size()) && (boundkey_var.size()==lowerbound_var.size()) )    )
	{
		cout<<"Variable bounds: vectors of boundkey, lowerbound and upperbound should have the same size."<<endl;
		exit(1);
	}

	if ( boundkey_var.size()!= m_num_var )
	{
		cout<<"Variable bounds: dimension mismatch."<<endl;
		exit(1);
	}

	MSKboundkeye   bkx[m_num_var];
	double        blx[m_num_var];
	double        bux[m_num_var];
	
	for (int i =0; i<m_num_var; i++)
	{
		bkx[i] = boundkey_var(i);
		blx[i] = lowerbound_var(i);
		bux[i] = upperbound_var(i);
	}

	
	r =MSK_putboundslice(task, MSK_ACC_VAR, 0, m_num_var, bkx, blx, bux);
#ifdef DEBUG_QP
	cout<<"done putting variable bounds"<<endl;
#endif
	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while setting up the variable bounds.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	} 
}

//------------------------------------------------------------
//! Run optimization.

void QPsolver::Optimize()
{
	xx.resize(m_num_var);
	// MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 10e-14);
	MSKrescodee trmcode; // Relay information about the conditions under which the optimizer terminated

	// Run optimizer 
	r = MSK_optimizetrm(task,&trmcode);

#ifdef DEBUG_MOSEK
	// Print a summary containing information about the solution 
	MSK_solutionsummary (task,MSK_STREAM_LOG);

	int iter;
	MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter);
	cout<<"Number of interior-point iterations since invoking the interior-point optimizer: "<< iter<<endl;
#endif

	if ( r==MSK_RES_OK ) 
	{
		MSKsolstae soln_status;
		
		MSK_getsolutionstatus (task, MSK_SOL_ITR, NULL, &soln_status);

		switch(soln_status)
		{
			case MSK_SOL_STA_OPTIMAL:   
			case MSK_SOL_STA_NEAR_OPTIMAL:
				MSK_getsolutionslice(task,
									 MSK_SOL_ITR,    // Request the interior solution.
									 MSK_SOL_ITEM_XX,// return x in corresponding dual problem
									 0,              // Index of first variable.    
									 m_num_var,      // Index of last variable+1.   
									 xx.data());
				solnStatus = 0;
				// cout<<"The optimal solution is "<<endl<<xx<<endl;
				break;
			case MSK_SOL_STA_DUAL_INFEAS_CER:
			case MSK_SOL_STA_PRIM_INFEAS_CER:
			case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
			case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
				solnStatus = 1;  
				printf("Primal or dual infeasibility certificate found.\n");
				//cout<<"time"<<endl;
				break;
				
			case MSK_SOL_STA_UNKNOWN:
				printf("The status of the solution could not be determined.\n");
				solnStatus = 2;
				break;
			default:
				solnStatus = 3;
				printf("Other solution status.");
				break;
		}

	}

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while optimizing.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	} 

}


void QPsolver::ModifySingleConstraintBound(int index, MSKboundkeye bkey, double new_lb, double new_ub)
{
	r =  MSK_putbound(task, MSK_ACC_CON, index, bkey, new_lb, new_ub);	

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while optimizing.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	}
}

void QPsolver::ModifySingleVariableBound(int index, MSKboundkeye bkey, double new_lb, double new_ub)
{
	r =  MSK_putbound(task, MSK_ACC_VAR, index, bkey, new_lb, new_ub);	

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while optimizing.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	}
}

//! Should only be called after QP problem setup
void QPsolver::Reset( )
{
	MSKintt	subv[m_num_var];
	for (int k = 0; k<m_num_var;k++)
	{
		subv[k] = k;
	}

	MSKintt	subc[m_num_con];
	for (int k = 0; k<m_num_con;k++)
	{
		subc[k] = k;
	}
	r = MSK_remove(task, MSK_ACC_VAR, m_num_var, subv);

	if (r == MSK_RES_OK) 
	{	
		r = MSK_remove(task, MSK_ACC_CON, m_num_con, subc);
	}

	if (r != MSK_RES_OK) 
	{	
		// In case of an error, print error code and description.  
		char symname[MSK_MAX_STR_LEN]; //1024 
		char desc[MSK_MAX_STR_LEN]; 
		printf("An error occurred while resetting.\n"); 
		MSK_getcodedesc (r, symname, desc); 
		printf("Error %s - '%s'\n",symname,desc); 
	}

}


