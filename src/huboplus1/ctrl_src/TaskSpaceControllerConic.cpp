/*
 * TaskSpaceControllerConic.cpp
 *
 *  Created on: Mar 9, 2014
 *      Author: yiping
 */


#include "TaskSpaceControllerConic.h"
#include "mosek.h"
#include <Eigen/Cholesky>
#include "globalVariables.h"

#define MAXNUMTASKS (6+NJ+6+6*NS)
// NJ+6 - Pose (actuated dof NJ)
// 6 - centroid momentum
// 6*NS - feet orientation/position


const int TaskSpaceControllerConic::tauStart    = 0;
const int TaskSpaceControllerConic::tauEnd      = NJ-1;
const int TaskSpaceControllerConic::fStart      = TaskSpaceControllerConic::tauEnd+1;
const int TaskSpaceControllerConic::fEnd        = TaskSpaceControllerConic::fStart+ (3*NS*NP-1);
const int TaskSpaceControllerConic::zStart      = TaskSpaceControllerConic::fEnd + 1;
const int TaskSpaceControllerConic::eStart      = TaskSpaceControllerConic::zStart + 1;
const int TaskSpaceControllerConic::eEnd		= TaskSpaceControllerConic::eStart+ MAXNUMTASKS-1;

const int TaskSpaceControllerConic::eConstrStart = 0;
const int TaskSpaceControllerConic::eConstrEnd   = TaskSpaceControllerConic::eConstrStart + MAXNUMTASKS-1;
const int TaskSpaceControllerConic::fNormConstrStart = TaskSpaceControllerConic::eConstrEnd+1;
const int TaskSpaceControllerConic::fNormConstrEnd   = TaskSpaceControllerConic::fNormConstrStart+NS-1;



#define MAXNUMCON (MAXNUMTASKS+NS)					// Number of constraints.
#define MAXNUMVAR (NJ+3*NS*NP+1+MAXNUMTASKS)			// Number of variables.

#define MAXNUMANZ (MAXNUMVAR*MAXNUMCON)				// Number of non-zeros in A.


#define OPTIM_DEBUG

static void MSKAPI printstr(void *handle, char str[]) // 'static' tells compiler this function is only seen in this source file
{
#ifdef OPTIM_DEBUG
	printf("%s",str);
#endif

} /* printstr */

TaskSpaceControllerConic::TaskSpaceControllerConic(dmArticulation * artic) : TaskSpaceController(artic)
{
	// minfz = -MSK_INFINITY;
	SupportJacobians.resize(NS);

	// Initialize Support Jacobians
	for (int i=0; i<NS; i++)
	{
		SupportJacobians[i] = MatrixXF::Zero(6,NP+6); // ??
	}


	/* Create the mosek environment. */
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);


	// Initial Assumption for this round in the library
	numCon = MAXNUMCON;
	numVar = MAXNUMVAR;

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
		r = MSK_maketask(env,MAXNUMCON,MAXNUMVAR,&task);

		if ( r==MSK_RES_OK )
		{
			r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

			/* Give MOSEK an estimate of the size of the input data.
			 This is done to increase the speed of inputting data.
			 However, it is optional. */
			if (r == MSK_RES_OK)
				r = MSK_putmaxnumvar(task,MAXNUMVAR);

			if (r == MSK_RES_OK)
				r = MSK_putmaxnumcon(task,MAXNUMCON);

			if (r == MSK_RES_OK)
				r = MSK_putmaxnumanz(task,MAXNUMANZ);

			if (r == MSK_RES_OK)
				r =  MSK_putmaxnumcone (task,NS*NP+1); // 1 is for the error cone, others for force cones

			// Append 'numCon' empty constraints.
			// The constraints will initially have no bounds.
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_CON,numCon);

			/* Append 'numVar' variables.
			 The variables will initially be fixed at zero (x=0). */
			if ( r == MSK_RES_OK )
				r = MSK_append(task,MSK_ACC_VAR,numVar);

			// Append Error Cone ?
			int k=0;
			MSKidxt csub[MAXNUMTASKS+1];
			csub[k++] = zStart;

			for (int i=0; i<MAXNUMTASKS; i++)
			{
				csub[k++]=eStart+i;
			}
			MSK_appendcone(task, MSK_CT_QUAD, 0.0, MAXNUMTASKS+1, csub); // cone 0

			// Append force cones!
			for (int i=0; i<NS*NP; i++)
			{
				MSKidxt csub[3];
				csub[0] = fStart+3*i+2;
				csub[1] = fStart+3*i;
				csub[2] = fStart+3*i+1;

				MSK_appendcone(task, MSK_CT_QUAD, 0.0, 3, csub); // cone 1, 2, ...
			}


		}
	}



	int k=0;
	for (int i=tauStart; i<=tauEnd; i++)
	{
		stringstream ss;
		ss << "t" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
	}

	k=0;
	for (int i=eStart; i<=eEnd; i++)
	{
		stringstream ss;
		ss << "e" << k++;
		MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
	}


	k=0;
	int k1=0;
	int k2=0;
	for (int i=fStart; i<=fEnd; )
	{
		for (k2 = 0; k2<3; k2++)
		{
			stringstream ss;
			ss << "f" << k1 <<"," << k2;
			MSK_putname(task, MSK_PI_VAR, i, (char *) ss.str().c_str());
			i++;
		}
		k1++;
	}
	MSK_putname(task, MSK_PI_VAR, zStart, "z");


	k=0;
	for (int i=eConstrStart; i<= eConstrEnd; i++)
	{
		stringstream ss;
		ss << "eConstraint" << k++;
		MSK_putname(task, MSK_PI_CON, i, (char*) ss.str().c_str());
	}



}

TaskSpaceControllerConic::~TaskSpaceControllerConic()
{
	MSK_deletetask(&task);
	MSK_deleteenv(&env);
}


void TaskSpaceControllerConic::ObtainArticulationData()
{
	artic->computeH();
	artic->computeCandG();

	for (int i=0; i<NS; i++)
	{
		artic->computeJacobian(SupportIndices[i], SupportXforms[i],SupportJacobians[i]);
	}
}

void TaskSpaceControllerConic::UpdateObjective()
{
	// c^T x
	VectorXf c;
	c.setZero(numVar);
	c(zStart) = 1.;
	MSKidxt j;
	for(j=0; j<numVar && r == MSK_RES_OK; ++j)
	{
		// Set the linear term c_j in the conic objective.
		if(r == MSK_RES_OK)
		{
			r = MSK_putcj(task,j,c(j));
		}
	}
	//MSK_putcfix(task,0.0);
}

void TaskSpaceControllerConic::UpdateConstraintMatrix()
{

	MSKidxt asub[numVar],asubtmp[numVar];
	double aval[numVar];

	// A -- alias for General Taskjacobian
	// H -- Joint-Space Initia M

	Hdecomp.compute(artic->H);
	MatrixXF invH_At = Hdecomp.solve(TaskJacobian.transpose()); //H.ldlt().solve(...)
	MatrixXF A_invH = invH_At.transpose();
	int m = TaskJacobian.rows();

	LambdaInvTau = A_invH.block(0,6,m,NJ);
	LambdaInvTau.topRows(6).setZero();//? The top 6 rows (cent mom) should come out zero if not for numerical errors, since only the external forces impact the centroidal momentum

	LambdaInvF.setZero(m,3*NS*NP); //3 -- linear force, 3 components each

	eBiasCandG.resize(m);
	eBiasCandG = TaskJacobian*Hdecomp.solve(artic->CandG);

	// Compute the LamndaInvF using one leg at a time, exploiting common structure.
	for (int i = 0; i<NS; i++)
	{
		MatrixXF LambdaInvF_SupportBody(m,6);
		LambdaInvF_SupportBody = A_invH * SupportJacobians[i].transpose();

		for (int j=0; j<NP; j++)
		{
			LambdaInvF.block(0,3*(NP*i+j),m,3) = LambdaInvF_SupportBody*PointForceXforms[i][j].rightCols(3); // right most 3 columns
			// Mu scaling
			LambdaInvF.col(3*(NP*i+j)+2) /= MU; //? to facilitate formulating cone constraints
		}
	}

	// MSK specific
	// Load row by row for dynamics constraints, a.k.a. constraints on e vector (Equality constraints)
	for(MSKidxt i=eConstrStart; i<=eConstrEnd && r == MSK_RES_OK; ++i)
	{
		MSKidxt k=0;
		for (MSKidxt j=0; j<NJ; j++)
		{
			if (LambdaInvTau(i,j) != 0)
			{
				asub[k]=j+tauStart;
				aval[k]=TaskWeight(i)*LambdaInvTau(i,j);
				k++;
			}
		}

		for (MSKidxt j=0; j<3*NS*NP; j++)
		{
			if (LambdaInvF(i,j) != 0)
			{
				asub[k]=j+fStart;
				aval[k]=TaskWeight(i)*LambdaInvF(i,j);
				k++;
			}
		}

		asub[k] = eStart+i;
		aval[k] = -1; //?
		k++;

		//cout << "Loading Constraint " << i << " k=" << endl;
		if(r == MSK_RES_OK)
		{
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}
	}

	// Load rows for foot normal force constraints
	for (MSKidxt i=0; i<NS; i++)
	{
		MSKidxt asub[NP];
		MSKrealt aval[NP];

		for (int j=0; j<NP; j++)
		{
			asub[j] = fStart+(3*(NP*i+j)+2);
			aval[j] = 1;
		}
		if(r == MSK_RES_OK)
		{
			r = MSK_putavec(task, MSK_ACC_CON,i+fNormConstrStart,NP,asub,aval);
		}
	}
}

void TaskSpaceControllerConic::UpdateInitialConstraintBounds()
{
	for (MSKidxt i=0; i<NS; i++)
	{
		if(r == MSK_RES_OK)
		{
			r = MSK_putbound(task, MSK_ACC_CON, fNormConstrStart+i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);// constraint is free
		} // forces need satisfy cone constraints
	}
}



void TaskSpaceControllerConic::UpdateHPTConstraintBounds()
{
	int k=0;
	MSKidxt csub[MAXNUMTASKS+1];

	csub[k++] = zStart; // first variable is the z variable!

	MSKboundkeye bk;
	double bl, bu;
	for (int i=0; i<MAXNUMTASKS; i++)
	{

		if (taskConstrActive(i) > 0)
		{
			// for already optimized (higher priority task) errors
			bk = MSK_BK_FX;  //fixed
			bl = TaskWeight(i)*(TaskBias(i)+eBiasCandG(i)); // TaskBias ?
			bu = bl;
			MSK_putbound(task, MSK_ACC_VAR, eStart+i,MSK_BK_FX, 0, 0);//Put bounds on variables
		}
		else if (taskOptimActive(i) >0)
		{
			// errors being optimized
			bk = MSK_BK_FX;
			bl = TaskWeight(i)*(TaskBias(i)+eBiasCandG(i));
			bu = bl;
			csub[k++] = eStart+i;
		}
		else
		{
			bk = MSK_BK_FR;
			bl = -MSK_INFINITY;
			bu = +MSK_INFINITY;
		}

		MSK_putbound(task, MSK_ACC_CON, i, bk, bl, bu);
	}

	// update error cone
	MSK_putcone(task, 0, MSK_CT_QUAD, 0.0, k, csub);//Replaces a conic constraint. (quadratic cone)
	//In MOSEK all variables belong to the set of reals, unless they are explicitly declared as belonging to a cone. Each variable may belong to one cone at most.
}


void TaskSpaceControllerConic::UpdateVariableBounds()
{
	MSKidxt       i;

	MSKboundkeye  bkx[numVar];
	double        blx[numVar];
	double        bux[numVar];

	// Initialize Bounds for Tau Variables
	for (i=tauStart; i<=tauEnd; i++)
	{
		bkx[i] = MSK_BK_RA; // range
		blx[i] = -1000;
		bux[i] = +1000;
	}

	//Initialize Bounds for Force Variables
	for (i=fStart; i<=fEnd; i++)
	{
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}

//	for (i=fStart+2; i<=fEnd; i+=3) // z component
//	{
//		bkx[i] = MSK_BK_LO;
//		blx[i] = minfz;
//		bux[i] = +MSK_INFINITY;
//	}


	//Initialize Bounds for Error Variables
	for (i=eStart; i<=eEnd; i++)
	{
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
		bux[i] = +MSK_INFINITY;
	}

	bkx[zStart] = MSK_BK_FR;
	blx[zStart] = -MSK_INFINITY;
	bux[zStart] = +MSK_INFINITY;

	MSK_putboundslice(task, MSK_ACC_VAR, 0, numVar, bkx, blx, bux);
}

void TaskSpaceControllerConic::AssignFootMaxLoad(int index, double maxLoad) // index selects a specific support body
{
	r = MSK_putbound(task, MSK_ACC_CON, fNormConstrStart+index, MSK_BK_UP, -MSK_INFINITY, maxLoad);

	if (maxLoad == 0)
	{
		for (int i=fStart+3*NP*index; i<(fStart+3*NP*(index+1)); i++)
		{
			MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FX, 0, 0);
		}
	} // forces need satisfy cone constraints
}

void TaskSpaceControllerConic::Optimize()
{
	static VectorXF f(NS*NP);
	xx.resize(numVar);

	if ( r==MSK_RES_OK )
	{
		MSKrescodee trmcode;

		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 1e-04);
		MSK_putintparam(task, MSK_IPAR_PRESOLVE_USE, MSK_PRESOLVE_MODE_OFF);


		/* Run optimizer */
		cout <<"inside_Optimize_1"<<endl;
		r = MSK_optimizetrm(task,&trmcode);
		cout <<r<<endl;

#ifdef OPTIM_DEBUG
		/* Print a summary containing information
		 about the solution for debugging purposes*/
		MatrixXF A;
		A.resize(numCon,numVar);

		for (int i=0; i<numCon; i++)
		{
			for (int j=0; j<numVar; j++)
			{
				if (j==tauStart || j==fStart || j==eStart)
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

			MSK_getsolutionstatus (task,
								   MSK_SOL_ITR,
								   NULL,
								   &solsta);

			//MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter);
			//cout << "Iter = " << iter << endl;

			switch(solsta)
			{
				case MSK_SOL_STA_OPTIMAL:
				case MSK_SOL_STA_NEAR_OPTIMAL:
					MSK_getsolutionslice(task,
										 MSK_SOL_ITR,    /* Request the interior solution. */
										 MSK_SOL_ITEM_XX,/* Which part of solution.     */
										 0,              /* Index of first variable.    */
										 numVar,         /* Index of last variable+1.   */
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


					r = MSK_printdata(task, MSK_STREAM_LOG, 0, numCon, 0, numVar, 0, NS*NP+1, 0, 0, 0, 0, 1, 1, 1, 1);
					//r = MSK_printdata(task, MSK_STREAM_LOG	, 0, NUMVAR, 0, NUMCON, 0, NUMCONE, 0,0, 0, 0, 0, 0, 0, 1);
					exit(-1);
					simThread->paused_flag = true;
					break;

				case MSK_SOL_STA_UNKNOWN:
					printf("The status of the solution could not be determined.\n");
				    float a;
					cin >> a;

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


	/////
	tau = xx.segment(tauStart,NJ);
	qdd = VectorXF::Zero(NJ+6);
	fs = VectorXF::Zero(6*NS);



	f = xx.segment(fStart,3*NS*NP);
	//cout << "f = " << f.transpose() << endl;
	TaskError = xx.segment(eStart,(eEnd-eStart)+1);
	for (int i=0; i<TaskError.size(); i++)
	{
		TaskError(i)/=TaskWeight(i);
	}

	for (int i=0; i<NS; i++)
	{
		for (int j=0; j<NP ; j++)
		{
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
	for (int i=0; i<NS; i++)
	{
		generalizedContactForce += SupportJacobians[i].transpose()*fs.segment(6*i,6);
	}

	qdd = Hdecomp.solve(S.transpose() * tau + generalizedContactForce- CandG); // compute qdd


}
