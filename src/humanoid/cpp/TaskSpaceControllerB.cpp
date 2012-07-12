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

#define MAXNUMTASKS (20+6+12)


const int TaskSpaceControllerB::tauStart    = 0;
const int TaskSpaceControllerB::tauEnd      = NJ-1;
const int TaskSpaceControllerB::fStart      = TaskSpaceControllerB::tauEnd+1;
const int TaskSpaceControllerB::fEnd        = TaskSpaceControllerB::fStart+ (3*NS*NP-1);
const int TaskSpaceControllerB::zStart      = TaskSpaceControllerB::fEnd + 1;
const int TaskSpaceControllerB::eStart      = TaskSpaceControllerB::zStart + 1;
const int TaskSpaceControllerB::eEnd		= TaskSpaceControllerB::eStart+ MAXNUMTASKS-1;

const int TaskSpaceControllerB::eConstrStart = 0;
const int TaskSpaceControllerB::eConstrEnd   = TaskSpaceControllerB::eConstrStart + MAXNUMTASKS-1;


#define MAXNUMCON (MAXNUMTASKS)						/* Number of constraints.             */
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
	
	VectorXd c = VectorXF::Zero(NUMVAR);
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
		blx[i] = -500;
		bux[i] = +500;
	}
	
	//Intialize Bounds for Force Variables
	for (i=fStart; i<=fEnd; i++) {
		bkx[i] = MSK_BK_FR;
		blx[i] = -MSK_INFINITY;
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
	
	cout << "Needs implemented" << endl;
	exit(-1);
	//r =  MSK_putbound(task, MSK_ACC_VAR, fStart+6*index+5, MSK_BK_UP, -MSK_INFINITY, maxLoad);	
}



void TaskSpaceControllerB::UpdateInitialConstraintBounds() {
	// None	
}

void TaskSpaceControllerB::UpdateHPTConstraintBounds()
{
	MSKboundkeye  bkc[MAXNUMCON];
	double        blc[MAXNUMCON], buc[MAXNUMCON]; 
	MSKidxt       i;
	
	//cout << "Loading Hpt Constraints Bounds " << endl;
	int k = 0;
	// Bounds on Constraints
	for (i=hptConstrStart; i<hptConstrStart+TaskBias.rows(); i++) {
		//cout << k << " of " << TaskBias.rows() << endl;
		
		if (taskConstrActive(k) > 0) {
			bkc[i]=MSK_BK_FX;
		}
		else {
			bkc[i]=MSK_BK_FR;
		}
		
		blc[i]=TaskBias(k);
		buc[i]=TaskBias(k++);
		r = MSK_putbound(task, MSK_ACC_CON, i, bkc[i], blc[i], buc[i]);
	}
}


void TaskSpaceControllerB::UpdateConstraintMatrix() {
	
	MSKidxt       asub[NUMVAR],asubtmp[NUMVAR];
	double		  aval[NUMVAR];
	cout << setprecision(2);
	//cout << "H= " << endl << artic->H << endl;
	
	// Load rows for dynamics Constraints
	for(MSKidxt i=dynConstrStart; i<=dynConstrEnd && r == MSK_RES_OK; ++i) {
		MSKidxt k=0;
		for (MSKidxt j=0; j<NJ+6; j++) {
			if (artic->H(i,j) != 0) {
				asub[k]=j+qddStart;
				aval[k]=artic->H(i,j);
				k++;
			}
		}
		if (i>=6) {
			asub[k] = (i-6)+tauStart;
			aval[k] = -1;
			k++;
		}
		
		for (MSKidxt js=0; js<NS; js++) {
			MSKidxt fsub = 6*js + fStart;
			for (MSKidxt j=0; j<6; j++) {
				if (SupportJacobians[js](j,i) != 0) {
					asub[k] = fsub+j;
					aval[k] = -SupportJacobians[js](j,i);
					k++;
				}
			}
		}
		
		//cout << "Loading Constraint " << i << " k=" << endl;
		if(r == MSK_RES_OK) {
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}
		//cout << "d" << i << " = ";
		//for (int j=0; j<k; j++) {
		//	cout << aval[j] << " ";
		//}
		//cout << endl;
	}
	
	
	// Load rows for Force Constraints
	MSKidxt fSub = fStart;
	
	
	// Loop though support bodies
	for (MSKidxt i = 0; i<NS; i++) {
		//cout << "Building for support body " << i << endl;
		
		Matrix<Float, 6, NP*NF, RowMajor> LocalFrictionBases;
		
		MSKidxt k=0;
		Matrix6F PointXForm, TotalXForm;
		
		//cout << "Assigning PointXform blocks " << endl;
		//cout << "Block size " << PointXForm.block(0,0,3,3).size() << endl;
		//cout << "Assign size " << Matrix3F::Identity().size() << endl;
		PointXForm.block(0,0,3,3) = Matrix3F::Identity();
		PointXForm.block(3,3,3,3) = Matrix3F::Identity();
		PointXForm.block(0,3,3,3) = Matrix3F::Zero();
		
		//cout << "Getting forces for link " << SupportIndices[i] << endl;
		dmRigidBody * linki = (dmRigidBody*) artic->m_link_list[SupportIndices[i]]->link;
		dmContactModel * dmContactLattice = (dmContactModel *) linki->getForce(0); 
		
		//cout << "Assigning Temporary Matricies" << endl;
		Matrix3F RSup = SupportXforms[i].block(0,0,3,3);
		Matrix3F tmpMat = SupportXforms[i].block(3,0,3,3)*RSup.transpose();
		Vector3F piRelSup;
		crossExtract(tmpMat,piRelSup);
		
		//cout << "pirelsup " << endl << piRelSup << endl;
		
		//Loop thought points for support i
		for (MSKidxt jp =0; jp<NP; jp++) {
			
			//cout << "Looping through support point " << jp << endl;
			MSKidxt lambdaSub = lambdaStart + NP*NF*i + NF*jp;
			Vector3F pRel, tmp;
			
			//Tmp is now the contact point location relative to the body coordinate
			dmContactLattice->getContactPoint(jp,tmp.data());
			
			// Point of contact (relative to support origin) in support coordinates
			pRel = RSup*tmp + piRelSup;
			
			//cout << "Point of contact in body coordinates" << endl << tmp << endl;
			//cout << "Relative Point of contact in Support Coordinates " << endl << pRel << endl;
			
			PointXForm.block(0,3,3,3) = cr3(pRel);
			
			LocalFrictionBases.block(0,NF*jp,6,NF) = PointXForm*FrictionBasis;
			for (MSKidxt jf=0; jf<NF; jf++) {
				asubtmp[k++] = lambdaSub++;
			}
		}
		//cout << "Local Friction Basis Size " << LocalFrictionBases.rows() << " by " << LocalFrictionBases.cols() << endl;
		
		// Process Sparsity of Local Friction basis
		for (int j=0; j<6 ; j++) {
			
			int kact = 0;
			for (int cnt=0; cnt <NP*NF; cnt++) {
				if(abs(LocalFrictionBases(j,cnt)) > 10e-10) {
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
			
			if(r == MSK_RES_OK) {
				r = MSK_putavec(task, MSK_ACC_CON,fConstrStart+6*i+j,kact,asub,aval);
			}
			
			fSub++;
		}
		//cout << "Constraints Added " << endl;
		
	}
	
	// Load rows for Additional Constraints	
	int row=0;
	MSKidxt k=0;
	for (MSKidxt i =hptConstrStart; i< hptConstrStart + TaskBias.rows(); i++) {
		//cout << "CJ row " << row << endl;
		k=0;
		for (MSKidxt jj = 0; jj<(NJ+6); jj++) {
			if (TaskJacobian(row,jj) !=0) {
				aval[k]=TaskJacobian(row,jj);
				asub[k]=qddStart+jj;
				//cout << aval[k] << ",";
				k++;
			}
		}
		if(r == MSK_RES_OK) {
			r = MSK_putavec(task, MSK_ACC_CON,i,k,asub,aval);
		}
		row++;
		//cout << endl << " nnz " << k << endl;
		//cout << ConstraintJacobian.row(row) << endl;
		
	}
	//cout << "additional Constraints added" << endl;
	
}

void TaskSpaceControllerB::Optimize() {
	xx.resize(NUMVAR);
	if ( r==MSK_RES_OK ) {
		MSKrescodee trmcode;
		
		/* Run optimizer */
		r = MSK_optimizetrm(task,&trmcode);
		
		/* Print a summary containing information
		 about the solution for debugging purposes*/
#ifdef OPTIM_DEBUG
		MatrixXF A;
		A.resize(numCon,NUMVAR);
		
		for (int i=0; i<numCon; i++) {
			if (i==fConstrStart) {
				cout << endl;
			}
			
			
			for (int j=0; j<NUMVAR; j++) {
				if (j==qddStart || j==fStart || j==lambdaStart) {
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
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:  
					printf("Primal or dual infeasibility certificate found.\n");
					cout << "time = " <<setprecision(5) << simThread->sim_time << endl;
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
}