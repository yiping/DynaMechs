// TaskSpaceControllerA.cpp
// Nov 27, 2012
// YL

#include "TaskSpaceControllerA.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
#include "mosek.h"
#include "control_defs.h"
#include "globalVariables.h"

//#define OPTIM_DEBUG

#define MAXNUMTASKS (NJ+6 +6 + 6*NS)

#define MAXNUMCON (NJ+6 + 6*NS + MAXNUMTASKS)               /* Number of constraints.*/
#define MAXNUMVAR (NJ + NJ+6 + 6*NS + NS*NP*NF)        /* Number of variables.*/
#define MAXNUMANZ (MAXNUMCON * MAXNUMVAR)		   /* Number of non-zeros in A.*/
#define MAXNUMQNZ (MAXNUMVAR * MAXNUMVAR)          /* Max Number of non-zeros in Q.*/

//#define OPTIM_DEBUG


const int TaskSpaceControllerA::tauStart    = 0;
const int TaskSpaceControllerA::tauEnd      = NJ-1;
const int TaskSpaceControllerA::qddStart    = NJ;
const int TaskSpaceControllerA::qddEnd      = 2*NJ+5;
const int TaskSpaceControllerA::fStart      = 2*NJ+6;
const int TaskSpaceControllerA::fEnd        = 2*NJ+5 + 6*NS;
const int TaskSpaceControllerA::lambdaStart = 2*NJ+6 + 6*NS;
const int TaskSpaceControllerA::lambdaEnd   = 2*NJ+5 + 6*NS + NS*NP*NF;

const int TaskSpaceControllerA::dynConstrStart = 0;
const int TaskSpaceControllerA::dynConstrEnd   = NJ+5;
const int TaskSpaceControllerA::fConstrStart   = NJ+6;
const int TaskSpaceControllerA::fConstrEnd     = NJ+5+6*NS;
const int TaskSpaceControllerA::hptConstrStart = fConstrEnd +1;



static void MSKAPI printstr(void *handle,
                            char str[])
{
#ifdef OPTIM_DEBUG
	printf("%s",str);
#endif
	
} /* printstr */


TaskSpaceControllerA::TaskSpaceControllerA(dmArticulation * art) : TaskSpaceController(art) 
{
	///
	artic = art;




	

	/// Mosek Initialization

	/* Create the mosek environment. */
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
	
	/* Check whether the return code is ok. */
	if ( r==MSK_RES_OK )
	{
		/* Directs the log stream to the 'printstr' function. */
		MSK_linkfunctoenvstream(env,
								MSK_STREAM_LOG,
								NULL,
								printstr);
	}
	
	/* Initialize the environment. */   
	r = MSK_initenv(env);

	if ( r==MSK_RES_OK )
	{ 
		/* Create the optimization task. */
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

			numCon = MAXNUMCON;
			numVar = MAXNUMVAR;

			// Append 'numCon' empty constraints.
			// The constraints will initially have no bounds.
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_CON,numCon);
			
			// Append 'numVar' variables.
			// The variables will initially be fixed at zero (x=0). 
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_VAR,numVar);
		}
	}
	
	// set names for variables
	int k=0;
	for (int i=tauStart; i<tauEnd; i++) {
		stringstream ss;
		ss << "t" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
	}
	
	k=0;
	for (int i=qddStart; i<qddEnd; i++) {
		stringstream ss;
		ss << "q" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
	}
	k=0;
	for (int i=lambdaStart; i<lambdaEnd; i++) {
		stringstream ss;
		ss << "l" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char*) ss.str().c_str());
	}
	
	// set names for constraints
	k=0;
	for (int i=dynConstrStart; i<dynConstrEnd; i++) {
		stringstream ss;
		ss << "dyn" << k++;
		MSK_putname(task, MSK_PI_CON, i, (char*) ss.str().c_str());
	}
	
	k=0;
	for (int i=fConstrStart; i<fConstrEnd; i++) {
		stringstream ss;
		ss << "f" << k++;
		MSK_putname(task, MSK_PI_CON, i, (char*) ss.str().c_str());
	}
	
}



TaskSpaceControllerA::~TaskSpaceControllerA() 
{
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
}

void TaskSpaceControllerA::ObtainArticulationData() 
{
	artic->computeH();
	artic->computeCandG();
		
	for (int i=0; i<NS; i++) 
	{
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}


//----------------------------------------------------------------------------

void TaskSpaceControllerA::UpdateObjective() 
{
	MSKidxt       qsubi[MAXNUMQNZ];
	MSKidxt       qsubj[MAXNUMQNZ];
	double        qval[MAXNUMQNZ];
	MSKidxt       i,j;
	
	MatrixXd JtT = TaskJacobian.transpose();

	// set inactive task rows to zeros
	for (int i=0; i<taskOptimActive.size(); i++) 
	{
		if (taskOptimActive(i)==0) 
		{
			JtT.col(i).setZero();
		}
		else 
		{
			JtT.col(i)*= TaskWeight(i)*TaskWeight(i);
		}
	}

	MatrixXd Q = JtT*TaskJacobian + MatrixXF::Identity(NJ+6,NJ+6)*.001;
	
	VectorXd c = VectorXF::Zero(numVar);
	c.segment(qddStart,NJ+6) = -JtT*TaskBias;
	
	
	// Add a constant Term to the Objective
	if ( r ==MSK_RES_OK ) 
	{
		r = MSK_putcfix(task,.5*TaskBias.dot(TaskBias));
	}
	
	// Set the linear term c_j in the objective.
	for(j=0; j<numVar && r == MSK_RES_OK; ++j) 
	{
		if(r == MSK_RES_OK) 
		{
			r = MSK_putcj(task,j,c[j]);
		}
	}
	
	// Set Q 
	if ( r==MSK_RES_OK )
	{	
		MSKidxt k=0;
		for (i = 0; i<(NJ+6); i++) 
		{
			for (j = 0; j<=i; j++) 
			{
				if (Q(i,j) != 0) 
				{
					qsubi[k]=i + qddStart;
					qsubj[k]=j + qddStart;
					qval[k]=Q(i,j);
					k++;
				}
			}
		}
		for (i = fStart; i <= (fEnd); i++) 
		{
			qsubi[k]=i; 
			qsubj[k]=i; 
			qval[k]=.1;
			k++;
		}
		
		if (k > MAXNUMQNZ) 
		{
			printf("PROBLEM WITH Q MATRIX, MORE NONZERO ELEMENTS THAN EXPECTED!\n");
			exit(-1);
		}

		if(r == MSK_RES_OK) 
		{
			r = MSK_putqobj(task,k,qsubi,qsubj,qval);
		}
	}
}

void TaskSpaceControllerA::UpdateVariableBounds() 
{
	MSKidxt       i;
	
	MSKboundkeye  bkx[numVar];
	double        blx[numVar];
	double        bux[numVar];
	
	// Initialize Bounds for Tau Variables
	for (i=tauStart; i<=tauEnd; i++) 
	{
		bkx[i] = MSK_BK_RA;
		blx[i] = -500;
		bux[i] = +500;
	}
	
	//Intialize Bounds for Qdd Variables
	for (i=qddStart; i<=qddEnd; i++) 
	{
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	//Intialize Bounds for Force Variables
	for (i=fStart; i<=fEnd; i++) 
	{
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	//Initialize Bounds for Lagrange Multiplies
	for (i=lambdaStart; i<=lambdaEnd; i++) 
	{
		bkx[i] = MSK_BK_LO;
		blx[i] = 0;
		bux[i] = +MSK_INFINITY;
	}
	
	MSK_putboundslice(task, MSK_ACC_VAR, 0, numVar-1, bkx, blx, bux);
}

void TaskSpaceControllerA::AssignFootMaxLoad(int index, double maxLoad) 
{
	
	r =  MSK_putbound(task, MSK_ACC_VAR, fStart+6*index+5, MSK_BK_UP, -MSK_INFINITY, maxLoad);	
}

//----------------------------------------------------------------------------

void TaskSpaceControllerA::UpdateInitialConstraintBounds() 
{
	MSKidxt       i;
	
	MSKboundkeye  bkc[MAXNUMCON];
	double        blc[MAXNUMCON], buc[MAXNUMCON]; 
	
	//Bounds on Dynamics Constraints
	for (i = dynConstrStart; i<=dynConstrEnd; i++) 
	{
		bkc[i] = MSK_BK_FX;
		blc[i] = -artic->CandG(i);
		buc[i] = -artic->CandG(i);
	}
	
	//Bounds on Force
	for (i = fConstrStart; i<=fConstrEnd; i++) 
	{
		bkc[i] = MSK_BK_FX;
		blc[i] = 0;
		buc[i] = 0;
	}
	

	// Putting initial constraint bounds 
	for(i=0; i<=fConstrEnd && r==MSK_RES_OK; ++i) 
	{
		r = MSK_putbound(task, MSK_ACC_CON, i, bkc[i], blc[i], buc[i]);
	}
	
}

void TaskSpaceControllerA::UpdateHPTConstraintBounds()
{
	MSKboundkeye  bkc[MAXNUMCON];
	double        blc[MAXNUMCON], buc[MAXNUMCON]; 
	MSKidxt       i;
	
	// cout << "taskConstrActive:" << endl<<taskConstrActive << endl;
	// cout << "taskOptimActive:" << endl << taskOptimActive << endl;
	
	// cout << "Loading Hpt Constraints Bounds " << endl;
	// cout << TaskBias.rows() << " vs. " << taskConstrActive.rows() << endl;
	int k = 0;

	// Bounds on Higher Priority Task constraints
	for (i=hptConstrStart; i<hptConstrStart+TaskBias.rows(); i++) 
	{
		//cout << k << " of " << TaskBias.rows() << endl;
		
		if (taskConstrActive(k) > 0) 
		{
			bkc[i]=MSK_BK_FX;
		}
		else 
		{
			bkc[i]=MSK_BK_FR;
		}
		
		blc[i]=TaskBias(k);
		buc[i]=TaskBias(k++);
		r = MSK_putbound(task, MSK_ACC_CON, i, bkc[i], blc[i], buc[i]);
	}
}


void TaskSpaceControllerA::UpdateConstraintMatrix() 
{
	
	MSKidxt       asub[numVar],asubtmp[numVar];
	double		  aval[numVar];
	cout << setprecision(2);
	//cout << "H= " << endl << artic->H << endl;
	
	/// Load rows for dynamics Constraints
	for(MSKidxt i=dynConstrStart; i<=dynConstrEnd && r == MSK_RES_OK; ++i) 
	{
		MSKidxt k=0;
		for (MSKidxt j=0; j<NJ+6; j++) 
		{
			if (artic->H(i,j) != 0) 
			{
				asub[k]=j+qddStart;
				aval[k]=artic->H(i,j);
				k++;
			}
		}

		if (i>=6) 
		{
			asub[k] = (i-6)+tauStart;
			aval[k] = -1;
			k++;
		}
		
		for (MSKidxt js=0; js<NS; js++) 
		{
			MSKidxt fsub = 6*js + fStart;
			for (MSKidxt j=0; j<6; j++) 
			{
				if (SupportJacobians[js](j,i) != 0) 
				{
					asub[k] = fsub+j;
					aval[k] = -SupportJacobians[js](j,i);
					k++;
				}
			}
		}
		
		
		if(r == MSK_RES_OK) 
		{
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}

	}
	



	/// Load rows for Force Constraints
	MSKidxt fSub = fStart;
	
	// Loop though support bodies
	for (MSKidxt i = 0; i<NS; i++) 
	{
		//cout << "Building for support body " << i << endl;
		
		Matrix<Float, 6, NP*NF, RowMajor> LocalFrictionBases;	// in support frame
		
		MSKidxt k=0;
		
		//Loop though contact points for support body i
		for (MSKidxt jp =0; jp<NP; jp++) 
		{
			//cout << "Looping through support point " << jp << endl;
			MSKidxt lambdaSub = lambdaStart + NP*NF*i + NF*jp;
			
			LocalFrictionBases.block(0,NF*jp,6,NF) = PointForceXforms[i][jp]*FrictionBasis;	// X' V
			for (MSKidxt jf=0; jf<NF; jf++) 
			{
				asubtmp[k++] = lambdaSub++;
			}
		}
		//cout << "Local Friction Basis Size " << LocalFrictionBases.rows() << " by " << LocalFrictionBases.cols() << endl;
		
		// Process Sparsity of Local Friction basis
		for (int j=0; j<6 ; j++) 
		{			
			int kact = 0;
			for (int cnt=0; cnt <NP*NF; cnt++) 
			{
				if(abs(LocalFrictionBases(j,cnt)) > 10e-10) 
				{
					asub[kact]=asubtmp[cnt];
					aval[kact++]=LocalFrictionBases(j,cnt); 
				}
			}
									  
			asub[kact]   = fSub;
			aval[kact++] = -1;
			
			//cout << "f" << j << endl;
			//for (int cnt =0; cnt<kact; cnt++) {
			//	cout << aval[cnt] << ", "; 
			//}
			//cout <<endl;
			
			if(r == MSK_RES_OK) 
			{
				r = MSK_putavec(task, MSK_ACC_CON,fConstrStart+6*i+j,kact,asub,aval);
			}
			
			fSub++;
		}
		
	}
	



	/// Load rows for Additional HPT Constraints	
	int row=0;
	MSKidxt k=0;
	for (MSKidxt i =hptConstrStart; i< hptConstrStart + TaskBias.rows(); i++) 
	{
		
		k=0;
		for (MSKidxt jj = 0; jj<(NJ+6); jj++) 
		{
			if (TaskJacobian(row,jj) !=0) 
			{
				aval[k]=TaskJacobian(row,jj);
				asub[k]=qddStart+jj;
				//cout << aval[k] << ",";
				k++;
			}
		}
		if(r == MSK_RES_OK) 
		{
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}
		row++;
		
	}
	

}


//----------------------------------------------------------------------------
void TaskSpaceControllerA::Optimize() 
{
	xx.resize(numVar);
	if ( r==MSK_RES_OK ) 
	{
		MSKrescodee trmcode; // Relay information about the conditions under which the optimizer terminated
		
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 10e-14);
		
		/* Run optimizer */
		r = MSK_optimizetrm(task,&trmcode);
		
		/* Print a summary containing information
		 about the solution for debugging purposes*/
		#ifdef OPTIM_DEBUG
			// examine constaint matrix
			MatrixXF A;
			A.resize(numCon,numVar);
			
			for (int i=0; i<numCon; i++) 
			{
				if (i==fConstrStart) 
				{
					cout << endl;
				}
				
				
				for (int j=0; j<numVar; j++) 
				{
					if (j==qddStart || j==fStart || j==lambdaStart) 
					{
						cout << "|";
					}
					double aij;
					MSK_getaij (task,i,j,&aij);
					A(i,j) = aij;
					if (abs(aij) > 10e-10) 
					{
						cout << "X";
					}
					else 
					{
						cout <<" ";
					}
				}
				cout << endl;
			}
		
			MSK_solutionsummary (task,MSK_STREAM_LOG);
			cout << "Optim Complete" << endl;
		#endif
		
		if ( r==MSK_RES_OK ) 
		{
			MSKsolstae solsta;
			//int j;
			
			
			MSK_getsolutionstatus (task, MSK_SOL_ITR, NULL, &solsta);

			#ifdef OPTIM_DEBUG
				MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter);
				cout << "Iter = " << iter << endl;
			#endif

			//MSKrealt primalObj;
			//MSK_getprimalobj(task, MSK_SOL_ITR,&primalObj);
			//cout << "Objective Val " << primalObj << endl;
			switch(solsta)
			{
				case MSK_SOL_STA_OPTIMAL:   
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getsolutionslice(task,
										 MSK_SOL_ITR,    // Request the interior solution.
										 MSK_SOL_ITEM_XX,// Which part of solution: return x in corresponding dual problem
										 0,              // Index of first variable.    
										 numVar,         // Index of last variable+1.   
										 xx.data());
					
					// cout<<"The optimal solution is "<<endl<<xx<<endl;
					break;
				case MSK_SOL_STA_DUAL_INFEAS_CER:
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:  
					printf("Primal or dual infeasibility certificate found.\n");
					break;
					
				case MSK_SOL_STA_UNKNOWN:
					printf("The status of the solution could not be determined.\n");
					break;
				default:
					printf("Other solution status.");
					break;
			}
		}
		else 
		{
			printf("Error while optimizing.\n");
		}
	}
	
	if (r != MSK_RES_OK) 
	{
		// In case of an error print error code and description.       
		char symname[MSK_MAX_STR_LEN];	//1024
		char desc[MSK_MAX_STR_LEN];
		
		printf("An error occurred while optimizing.\n");     
		MSK_getcodedesc (r,	 symname, desc);
		printf("Error %s - '%s'\n",symname,desc);
	}

	tau = xx.segment(tauStart,NJ);
	qdd = xx.segment(qddStart,NJ+6);
	fs = xx.segment(fStart,6*NS);
	lambda = xx.segment(lambdaStart,NS*NP*NF);

	TaskError = TaskJacobian*qdd-TaskBias;	//
	
}
