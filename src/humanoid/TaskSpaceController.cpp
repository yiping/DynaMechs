/*
 *  TaskSpaceController.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "TaskSpaceController.h"
#include "mosek.h"

//#define NUMCON 1   /* Number of constraints.             */
//#define NUMVAR 3   /* Number of variables.               */
//#define NUMANZ 3   /* Number of non-zeros in A.           */
//#define NUMQNZ 4   /* Number of non-zeros in Q.           */

#define NJ  30
#define NF  4
#define NS  2
#define NP  4
#define MU .9

#define MAXSUBTASKS 20

#define MAXNUMCON (NJ+6 + 2*NS + MAXSUBTASKS)              /* Number of constraints.             */
#define NUMVAR (NJ + NJ+6+2*NS+NS*NP*NF)  /* Number of variables.               */
#define MAXNUMANZ (4*NJ*NJ)					  /* Number of non-zeros in A.           */
#define MAXNUMQNZ ((NJ+6)*(NJ+6))            /* Max Number of non-zeros in Q.           */

const int tauStart    = 0;
const int tauEnd      = NJ-1;
const int qddStart    = NJ;
const int qddEnd      = 2*NJ+5;
const int fStart      = 2*NJ+6;
const int fEnd        = 2*NJ+5 + 6*NS;
const int lambdaStart = 2*NJ+6 + 6*NS;
const int lambdaEnd   = 2*NJ+5 + 6*NS + NS*NP*NF;

const int dynConstrStart = 0;
const int dynConstrEnd   = NJ+5;
const int fConstrStart   = NJ+6;
const int fConstrEnd     = NJ+5+6*NS;

static void MSKAPI printstr(void *handle,
                            char str[])
{
	printf("%s",str);
} /* printstr */


TaskSpaceController::TaskSpaceController(dmArticulation * art, IntVector & suppIndices, XformVector & suppXforms) {
	
	artic = art;
	SupportIndices = suppIndices;
	SupportXforms  = suppXforms;
	
	// Create Linearized Friction Cone Basis
	FrictionBasis = MatrixXF::Zero(6,NF);
	for (int j=0; j<NF; j++) {
		double angle = (j * 2 * M_PI) / NF;
		FrictionBasis(3,j) = MU*cos(angle);
		FrictionBasis(4,j) = MU*sin(angle);
		FrictionBasis(5,j) = 1;
	}
	
	// Initialize Support Jacobians
	for (int i=0; i<NS; i++) {
		SupportJacobians[i] = MatrixXF::Zero(6,NJ+6);
	}
	
	/* Create the mosek environment. */
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
	
	
	// Initial Assumption for this round in the library
	numCon = MAXNUMCON - MAXSUBTASKS;
	
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
		r = MSK_maketask(env,MAXNUMCON,NUMVAR,&task);
		
		if ( r==MSK_RES_OK )
		{
			r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
			
			/* Give MOSEK an estimate of the size of the input data. 
			 This is done to increase the speed of inputting data. 
			 However, it is optional. */
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumvar(task,NUMVAR);
			
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumcon(task,MAXNUMCON);
			
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumanz(task,MAXNUMANZ);
			
			/* Append 'numCon' empty constraints.
			 The constraints will initially have no bounds. */
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_CON,numCon);
			
			/* Append 'NUMVAR' variables.
			 The variables will initially be fixed at zero (x=0). */
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_VAR,NUMVAR);
		}
	}
	
}

//----------------------------------------------------------------------------

TaskSpaceController::~TaskSpaceController() {
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
}

void TaskSpaceController::ObtainArticulationData() {
	H = artic->computeH();
	CandG = artic->computeCandG();
	
	for (int i=0; i<NS; i++) {
		SupportJacobians[i] = artic->calculateJacobian(SupportIndices[i], SupportXforms[i]);
	}
	
	
	
	
}

//----------------------------------------------------------------------------
void TaskSpaceController::InitializeProblem()
{
	ObtainArticulationData();
	UpdateVariableBounds();
	UpdateConstraintMatrix();
	UpdateConstraintBounds();
	UpdateObjective();
}

void TaskSpaceController::UpdateObjective() {
	MSKidxt       qsubi[MAXNUMQNZ];
	MSKidxt       qsubj[MAXNUMQNZ];
	double        qval[MAXNUMQNZ];
	//double        c[NUMVAR]   = {0.0,-1.0,0.0};
	
	MSKidxt       i,j;
	
	MatrixXd JtT = TaskJacobian.transpose();
	
	MatrixXd Q = JtT*TaskJacobian;
	VectorXd c = -JtT*TaskBias;
	
	// Add a constant Term to the Objective
	if ( r ==MSK_RES_OK ) {
		r = MSK_putcfix(task,.5*TaskBias.dot(TaskBias));
	}
	
	for(j=0; j<NUMVAR && r == MSK_RES_OK; ++j) {
		// Set the linear term c_j in the objective.
		if(r == MSK_RES_OK)
			r = MSK_putcj(task,j,c[j]);
	}
	
	if ( r==MSK_RES_OK )
	{
		MSKidxt k;
		for (i = 0; i<(NJ+6); i++) {
			for (j = 0; j<=i; j++) {
				if (Q(i,j) != 0) {
					qsubi[k]=i+qddStart; qsubj[k]=j+qddStart; qval[k]=Q(i,j);
					k++;
				}
			}
		}
		if (k > MAXNUMQNZ) {
			printf("PROBLEM WITH Q MATRIX, MORE NONZERO ELEMENTS THAN EXPECTED!\n");
			exit(-1);
		}
		r = MSK_putqobj(task,k,qsubi,qsubj,qval);
	}
}

void TaskSpaceController::UpdateVariableBounds() {
	MSKidxt       i;
	
	MSKboundkeye  bkx[NUMVAR];
	double        blx[NUMVAR];
	double        bux[NUMVAR];
	
	// Initialize Bounds for Tau Variables
	for (i=tauStart; i<=tauEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	//Intialize Bounds for Qdd Variables
	for (i=qddStart; i<=qddEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	//Intialize Bounds for Force Variables
	for (i=fStart; i<=fEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	//Initialize Bounds for Lagrange Multiplies
	for (i=lambdaStart; i<=lambdaEnd; i++) {
		bkx[i] = MSK_BK_LO;
		blx[i] = 0;
		bux[i] = +MSK_INFINITY;
	}
	
	MSK_putboundslice(task, MSK_ACC_VAR, 0, NUMVAR-1, bkx, blx, bux);
}

void TaskSpaceController::UpdateConstraintBounds() {
	MSKidxt       i,j;
	
	MSKboundkeye  bkc[MAXNUMCON];
	double        blc[MAXNUMCON], buc[MAXNUMCON]; 
	
	//Bounds on Dynamics Constraints
	for (i = dynConstrStart; i<=dynConstrEnd; i++) {
		bkc[i] = MSK_BK_FX;
		blc[i] = -CandG(i);
		buc[i] = -CandG(i);
	}
	
	//Bounds on Force
	for (i = fConstrStart; i<=fConstrEnd; i++) {
		bkc[i] = MSK_BK_FX;
		blc[i] = -CandG(i);
		buc[i] = -CandG(i);
	}
	
	
	
	for(i=0; i<numCon && r==MSK_RES_OK; ++i) {
		r = MSK_putbound(task, MSK_ACC_CON, i, bkc[i], blc[i], buc[i]);
	}
	
}

void TaskSpaceController::UpdateConstraintMatrix() {
	
	// Load rows for dynamics Constraints
	
	// Load rows for Force Constraints
	
	// Load rows for Additional Constraints
	
	
	//
	/*
	
	
	for(j=0; j<NUMVAR && r == MSK_RES_OK; ++j) {
		
		// Input column j of A   
		if(r == MSK_RES_OK)
			r = MSK_putavec(task,
							MSK_ACC_VAR,       // Input columns of A.
							j,                 // Variable (column) index.
							aptre[j]-aptrb[j], // Number of non-zeros in column j.
							asub+aptrb[j],     // Pointer to row indexes of column j.
							aval+aptrb[j]);    // Pointer to Values of column j.
	}
	
	 */
}

void TaskSpaceController::Optimize()
{
	double        xx[NUMVAR];
	if ( r==MSK_RES_OK )
	{
		MSKrescodee trmcode;
		
		/* Run optimizer */
		r = MSK_optimizetrm(task,&trmcode);
		
		/* Print a summary containing information
		 about the solution for debugging purposes*/
		MSK_solutionsummary (task,MSK_STREAM_LOG);
		
		if ( r==MSK_RES_OK )
		{
			MSKsolstae solsta;
			int j;
			
			MSK_getsolutionstatus (task,
								   MSK_SOL_ITR,
								   NULL,
								   &solsta);
			
			switch(solsta)
			{
				case MSK_SOL_STA_OPTIMAL:   
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getsolutionslice(task,
										 MSK_SOL_ITR,    /* Request the interior solution. */
										 MSK_SOL_ITEM_XX,/* Which part of solution.     */
										 0,              /* Index of first variable.    */
										 NUMVAR,         /* Index of last variable+1.   */
										 xx);
					
					printf("Optimal primal solution\n");
					for(j=0; j<NUMVAR; ++j)
						printf("x[%d]: %e\n",j,xx[j]);
					
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
		/* In case of an error print error code and description. */      
		char symname[MSK_MAX_STR_LEN];
		char desc[MSK_MAX_STR_LEN];
		
		printf("An error occurred while optimizing.\n");     
		MSK_getcodedesc (r,
						 symname,
						 desc);
		printf("Error %s - '%s'\n",symname,desc);
	}
}