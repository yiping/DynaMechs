/*
 *  TaskSpaceController.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 4/23/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "TaskSpaceControllerB.h"
#include "dmContactModel.hpp"
#include "dmRigidBody.hpp"
#include "mosek.h"
#include "GlobalDefines.h"
#include <Eigen/Cholesky>

#define MAXNUMTASKS (6+20+6+12)

//#define OPTIM_DEBUG

const int TaskSpaceControllerB::tauStart    = 0;
const int TaskSpaceControllerB::tauEnd      = NJ-1;
const int TaskSpaceControllerB::fStart      = TaskSpaceControllerB::tauEnd+1;
const int TaskSpaceControllerB::fEnd        = TaskSpaceControllerB::fStart+ (3*NS*NP-1);
const int TaskSpaceControllerB::zStart      = TaskSpaceControllerB::fEnd + 1;
const int TaskSpaceControllerB::eStart      = TaskSpaceControllerB::zStart + 1;
const int TaskSpaceControllerB::eEnd		= TaskSpaceControllerB::eStart+ MAXNUMTASKS-1;

const int TaskSpaceControllerB::eConstrStart = 0;
const int TaskSpaceControllerB::eConstrEnd   = TaskSpaceControllerB::eConstrStart + MAXNUMTASKS-1;
const int TaskSpaceControllerB::fNormConstrStart = TaskSpaceControllerB::eConstrEnd+1;
const int TaskSpaceControllerB::fNormConstrEnd   = TaskSpaceControllerB::fNormConstrStart+NS-1;

#define MAXNUMCON (MAXNUMTASKS+NS)						/* Number of constraints.             */
#define NUMVAR (NJ+3*NS*NP+1+MAXNUMTASKS)			/* Number of variables.               */
#define MAXNUMANZ (NUMVAR*MAXNUMCON)				/* Number of non-zeros in A.           */
#define MAXNUMQNZ (MAXNUMVAR*MAXNUMVAR)				/* Max Number of non-zeros in Q.           */

static void MSKAPI printstr(void *handle,
                            char str[])
{
#ifdef OPTIM_DEBUG
	printf("%s",str);
#endif
	
} /* printstr */


TaskSpaceControllerB::TaskSpaceControllerB(dmArticulation * art) : TaskSpaceController(art) {
	minfz = -MSK_INFINITY;
	SupportJacobians.resize(NS);
	
	// Initialize Support Jacobians
	for (int i=0; i<NS; i++) {
		SupportJacobians[i] = MatrixXF::Zero(6,NP+6);;
	}
	
	
	/* Create the mosek environment. */
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
	
	
	// Initial Assumption for this round in the library
	numCon = MAXNUMCON;
	
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
			
			if (r == MSK_RES_OK) {
				r =  MSK_putmaxnumcone (task,NS*NP+1);
			}
			
			/* Append 'numCon' empty constraints.
			 The constraints will initially have no bounds. */
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_CON,numCon);
			
			/* Append 'NUMVAR' variables.
			 The variables will initially be fixed at zero (x=0). */
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_VAR,NUMVAR);
			
			// Error Cone
			int k=0;
			MSKidxt csub[MAXNUMTASKS+1];
			csub[k++] = zStart;
			for (int i=0; i<MAXNUMTASKS; i++) {
				csub[k++]=eStart+i;
			}
			MSK_appendcone(task, MSK_CT_QUAD, 0.0, MAXNUMTASKS+1, csub);
			
			// Append force cones!
			for (int i=0; i<NS*NP; i++) {
				MSKidxt csub[3];
				csub[0] = fStart+3*i+2;
				csub[1] = fStart+3*i;
				csub[2] = fStart+3*i+1;
				
				MSK_appendcone(task, MSK_CT_QUAD, 0.0, 3, csub);
			}
			
			
			
		}
	}
	
	//cout << "LambdaEnd " << lambdaEnd << " NUMVAR " << NUMVAR << endl;
	
	int k=0;
	for (int i=tauStart; i<tauEnd; i++) {
		stringstream ss;
		ss << "t" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
	}
	
	k=0;
	for (int i=eConstrStart; i<= eConstrEnd; i++) {
		stringstream ss;
		ss << "eConst" << k++;
		MSK_putname(task, MSK_PI_CON, i, (char*) ss.str().c_str());
	}
}

//----------------------------------------------------------------------------

TaskSpaceControllerB::~TaskSpaceControllerB() {
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
}

void TaskSpaceControllerB::ObtainArticulationData() {
	artic->computeH();
	artic->computeCandG();
	
	for (int i=0; i<NS; i++) {
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}

void TaskSpaceControllerB::UpdateTauObjective() {
	
}

void TaskSpaceControllerB::UpdateObjective() {
	
	VectorXd c;
	c.setZero(NUMVAR);
	c(zStart) = 1;
	MSKidxt j;
	
	for(j=0; j<NUMVAR && r == MSK_RES_OK; ++j) {
		// Set the linear term c_j in the objective.
		if(r == MSK_RES_OK) {
			r = MSK_putcj(task,j,c(j));
		}
	}
}

void TaskSpaceControllerB::UpdateVariableBounds() {
	MSKidxt       i;
	
	MSKboundkeye  bkx[NUMVAR];
	double        blx[NUMVAR];
	double        bux[NUMVAR];
	
	// Initialize Bounds for Tau Variables
	for (i=tauStart; i<=tauEnd; i++) {
		bkx[i] = MSK_BK_RA;
		blx[i] = -1000;
		bux[i] = +1000;
	}
	
	//Intialize Bounds for Force Variables
	for (i=fStart; i<=fEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	for (i=fStart+2; i<=fEnd; i+=3) {
		bkx[i] = MSK_BK_LO;
		blx[i] = minfz;
		bux[i] = +MSK_INFINITY;
	}
	
	
	//Initialize Bounds for Error Variables
	for (i=eStart; i<=eEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}
	
	bkx[zStart] = MSK_BK_FR;
	blx[zStart] = -MSK_INFINITY;
	bux[zStart] = +MSK_INFINITY;
	
	MSK_putboundslice(task, MSK_ACC_VAR, 0, NUMVAR-1, bkx, blx, bux);
}

void TaskSpaceControllerB::AssignFootMaxLoad(int index, double maxLoad) {
	
	r = MSK_putbound(task, MSK_ACC_CON, fNormConstrStart+index, MSK_BK_UP, -MSK_INFINITY, maxLoad);	
	//for (int i=fStart+3*NP*index+2; i<(fStart+3*NP*(index+1)+2); i+=3) {
	//	MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
	//}
	if (maxLoad == 0) {
		for (int i=fStart+3*NP*index; i<(fStart+3*NP*(index+1)); i++) {
			MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FX, 0, 0);
		}
	}
}



void TaskSpaceControllerB::UpdateInitialConstraintBounds() {
	for (MSKidxt i=0; i<NS; i++) {
		if(r == MSK_RES_OK) {
			r = MSK_putbound(task, MSK_ACC_CON, fNormConstrStart+i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
		}
	}	
}

void TaskSpaceControllerB::UpdateHPTConstraintBounds() {
	int k=0;
	MSKidxt csub[MAXNUMTASKS+1];
	csub[k++] = zStart;
	
	MSKboundkeye bk;
	double bl, bu;
	for (int i=0; i<MAXNUMTASKS; i++) {
		
		if (taskConstrActive(i) > 0) {
			bk = MSK_BK_FX;
			bl = TaskWeight(i)*(TaskBias(i)+eBiasCandG(i));
			bu = bl;
			MSK_putbound(task, MSK_ACC_VAR, eStart+i,MSK_BK_FX, 0, 0);
		}
		else if (taskOptimActive(i) >0) {
			bk = MSK_BK_FX;
			bl = TaskWeight(i)*(TaskBias(i)+eBiasCandG(i));
			bu = bl;
			csub[k++] = eStart+i;
		}
		else {
			bk = MSK_BK_FR;
			bl = -MSK_INFINITY;
			bu = +MSK_INFINITY;
		}

		MSK_putbound(task, MSK_ACC_CON, i, bk, bl, bu);
	}
	MSK_putcone(task, 0, MSK_CT_QUAD, 0.0, k, csub);
}


void TaskSpaceControllerB::UpdateConstraintMatrix() {
	
	//dmTimespec tv1,tv2;
	//dmGetSysTime(&tv1);
	MSKidxt       asub[NUMVAR],asubtmp[NUMVAR];
	double		  aval[NUMVAR];
	//cout << setprecision(2);
	//cout << "H= " << endl << artic->H << endl;
	
	Hdecomp.compute(artic->H);
	MatrixXF invHJt = Hdecomp.solve(TaskJacobian.transpose());
	MatrixXF JinvH = invHJt.transpose();
	int m = TaskJacobian.rows();
	
	LambdaInvTau = JinvH.block(0,6,m,NJ);
	LambdaInvTau.topRows(6).setZero();
	
	LambdaInvF.setZero(m,3*NS*NP);
	
	eBiasCandG.resize(m);
	eBiasCandG = TaskJacobian*Hdecomp.solve(artic->CandG);
	
	// Compute the LamndaInvF using one leg at a time, exploiting common structure.
	for (int i = 0; i<NS; i++) {
		MatrixXF LambdaInvFLoc(m,6);
		LambdaInvFLoc = JinvH * SupportJacobians[i].transpose();
			
		for (int j=0; j<NP; j++) {
			
			
			//int rightrows = PointForceXforms[i][j].bottomRows(3).rows();
			//int rightcols = PointForceXforms[i][j].bottomRows(3).cols();
			
			//cout << rightrows << "," << rightcols << endl;
			LambdaInvF.block(0,3*(NP*i+j),m,3) = LambdaInvFLoc*PointForceXforms[i][j].rightCols(3);
			// Mu scaling
			LambdaInvF.col(3*(NP*i+j)+2) /= MU;
		}
	} 
	
	//dmGetSysTime(&tv2);
	//cout << "Loaded " << timeDiff(tv1, tv2) << endl;
	
	// Load rows for dynamics Constraints
	for(MSKidxt i=eConstrStart; i<=eConstrEnd && r == MSK_RES_OK; ++i) {
		MSKidxt k=0;
		for (MSKidxt j=0; j<NJ; j++) {
			if (LambdaInvTau(i,j) != 0) {
				asub[k]=j+tauStart;
				aval[k]=TaskWeight(i)*LambdaInvTau(i,j);
				k++;
			}
		}
		
		for (MSKidxt j=0; j<3*NS*NP; j++) {
			if (LambdaInvF(i,j) != 0) {
				asub[k]=j+fStart;
				aval[k]=TaskWeight(i)*LambdaInvF(i,j);
				k++;
			}
		}
		
		asub[k] = eStart+i;
		aval[k] = -1;
		k++;
		
		//cout << "Loading Constraint " << i << " k=" << endl;
		if(r == MSK_RES_OK) {
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}
	}
	
	// Load rows for foot normal force constraints
	for (MSKidxt i=0; i<NS; i++) {
		MSKidxt asub[NP];
		MSKrealt aval[NP];
		
		for (int j=0; j<NP; j++) {
			asub[j] = fStart+(3*(NP*i+j)+2);
			aval[j] = 1;
		}
		if(r == MSK_RES_OK) {
			r = MSK_putavec(task, MSK_ACC_CON,i+fNormConstrStart,NP,asub,aval);
		}
	}
}
wxCommandEvent dummyPauseEvent;
void TaskSpaceControllerB::Optimize() {
	static VectorXF f(NS*NP);
	
	xx.resize(NUMVAR);
	if ( r==MSK_RES_OK ) {
		MSKrescodee trmcode;
		

		/*for (int i=0; i<taskConstrActive.size(); i++) {
			if (taskConstrActive(i) > 0) {
				Float val = LambdaInvF.row(i).dot(f) + LambdaInvTau.row(i).dot(tau);
				cout << "Const " << i << " " << val << " =? " << TaskBias(i)+eBiasCandG(i) << " (" << TaskBias(i) << " + " << eBiasCandG(i) << ") " << endl;
				
			}
			else if (taskOptimActive(i) > 0)
			{
				cout << "Optim " << i << " bias " << TaskBias(i)+eBiasCandG(i) << " (" << TaskBias(i) << " + " << eBiasCandG(i) << endl;
				
			}
		}*/
		
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 1e-04);
		MSK_putintparam(task, MSK_IPAR_PRESOLVE_USE, MSK_PRESOLVE_MODE_OFF);
		//MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, 10e-04);
		//MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, 10e-08);
		//MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, 10e-08);
		//MSK_putintparam(task, MSK_IPAR_INTPNT_NUM_THREADS, 1);
		
		//MSK_putintparam(task, MSK_IPAR_DATA_CHECK, MSK_OFF);
		//MSK_putintparam(task, MSK_IPAR_CONCURRENT_NUM_OPTIMIZERS, 0);
		//MSK_putintparam(task, MSK_IPAR_LOG, 0);
		//MSK_putintparam(task, MSK_IPAR_INTPNT_BASIS, MSK_BI_NEVER);
		
		/* Run optimizer */
		
		r = MSK_optimizetrm(task,&trmcode);
		/* Print a summary containing information
		 about the solution for debugging purposes*/
#ifdef OPTIM_DEBUG
		MatrixXF A;
		A.resize(numCon,NUMVAR);
		
		for (int i=0; i<numCon; i++) {
			
			for (int j=0; j<NUMVAR; j++) {
				if (j==tauStart || j==fStart || j==eStart) {
					cout << "|";
				}
				double aij;
				MSK_getaij (task,i,j,&aij);
				A(i,j) = aij;
				if (abs(aij) > 10e-10) {
					cout << "X";
				}
				else {
					cout <<" ";
				}
			}
			cout << endl;
		}
		
		MSK_solutionsummary (task,MSK_STREAM_LOG);
		cout << "Optim Complete" << endl;
#endif
		
		if ( r==MSK_RES_OK ) {
			MSKsolstae solsta;
			//int j;
			
			
			MSK_getsolutionstatus (task,
								   MSK_SOL_ITR,
								   NULL,
								   &solsta);
			
			MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter);
			//cout << "Iter = " << iter << endl;
			
			switch(solsta)
			{
				case MSK_SOL_STA_OPTIMAL:   
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getsolutionslice(task,
										 MSK_SOL_ITR,    /* Request the interior solution. */
										 MSK_SOL_ITEM_XX,/* Which part of solution.     */
										 0,              /* Index of first variable.    */
										 NUMVAR,         /* Index of last variable+1.   */
										 xx.data());
					
					//printf("Optimal primal solution\n");
					//for(j=0; j<NUMVAR; ++j)
					//	printf("x[%d]: %e\n",j,xx[j]);
					
					break;
				case MSK_SOL_STA_DUAL_INFEAS_CER:
					printf("d");
				case MSK_SOL_STA_PRIM_INFEAS_CER:
					printf("P");
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					printf("nd");
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:  
					printf("Primal or dual infeasibility certificate found.\n");
					cout << "time = " <<setprecision(5) << simThread->sim_time << endl;
					
					simThread->paused_flag = true;
					break;
					
				case MSK_SOL_STA_UNKNOWN:
					printf("The status of the solution could not be determined.\n");
					break;
				default:
					printf("Other solution status.");
					break;
			}
		}
		else {
			printf("Error while optimizing.\n");
		}
	}
	
	if (r != MSK_RES_OK) {
		/* In case of an error print error code and description. */      
		char symname[MSK_MAX_STR_LEN];
		char desc[MSK_MAX_STR_LEN];
		
		printf("An error occurred while optimizing.\n");     
		MSK_getcodedesc (r,
						 symname,
						 desc);
		printf("Error %s - '%s'\n",symname,desc);
	}
	
	//MSKrealt a;
	//MSK_getdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, &a);
	//cout << a << endl;
	//exit(-1);
	
	tau = xx.segment(tauStart,NJ);
	qdd = VectorXF::Zero(NJ+6);
	fs = VectorXF::Zero(6*NS);
	lambda = VectorXF::Zero(1);
	
	
	f = xx.segment(fStart,3*NS*NP);
	//cout << "f = " << f.transpose() << endl;
	TaskError = xx.segment(eStart,(eEnd-eStart)+1);
	for (int i=0; i<TaskError.size(); i++) {
		TaskError(i)/=TaskWeight(i);
	}
	
	for (int i=0; i<NS; i++) {
		for (int j=0; j<NP ; j++) {
			Vector3F floc = f.segment(3*(NP*i+j),3);
			floc(2)/=MU;
			fs.segment(6*i,6) += PointForceXforms[i][j].rightCols(3)*floc;
		}
	}
	
	//cout << "fs " << fs.transpose() << endl;
	MatrixXF H = artic->H;
	VectorXF CandG = artic->CandG;
	
	MatrixXF S = MatrixXF::Zero(NJ,NJ+6);
	S.block(0,6,NJ,NJ) = MatrixXF::Identity(NJ,NJ);
	
	VectorXF generalizedContactForce = VectorXF::Zero(NJ+6);
	for (int i=0; i<NS; i++) {
		generalizedContactForce += SupportJacobians[i].transpose()*fs.segment(6*i,6);
	}
	
	qdd = Hdecomp.solve(S.transpose() * tau + generalizedContactForce- CandG);
	
	//cout << "f = " << endl << f << endl;
	
	/*for (int i=0; i<taskConstrActive.size(); i++) {
		if (taskOptimActive(i) > 0) {
			Float val = LambdaInvF.row(i).dot(f) + LambdaInvTau.row(i).dot(tau);
			cout << "Post " << i << " " << val << " =? " << TaskBias(i)+eBiasCandG(i) << " e=(" << TaskError(i) << " vs " << -val+TaskBias(i)+eBiasCandG(i) << ") " << endl;
			
		}
	}*/
	//cout << fs.transpose()<< endl;
}