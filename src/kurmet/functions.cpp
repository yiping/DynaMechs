
#include <dm.h>
#include "functions.h"

// A is 6 by 6 square matrix
Vector6F solveInverse(const Matrix6F & A, const Vector6F & b)
{
	Vector6F x;
	x = A.partialPivLu().solve( b );
	return x;
}

VectorXF solveJacobianPseudoInverse(const Matrix6XF & J, const Vector6F & rhs)
{
	VectorXF x;
	x = J.jacobiSvd(ComputeThinU | ComputeThinV).solve(rhs);
	return x;
}

void simDataOutput(const DataRecVector & MyVec)
{
	cout<<"outputing simulation data..."<<endl;
    string OutputFileString = "sim_data.txt";
    FILE *fPtr = fopen(OutputFileString.c_str(),"w");

    // "w" Create an empty file for writing.
	// If a file with the same name already exists
    // its content is erased and the file is treated as a new empty file.

	for( int u = 0; u<MyVec.size();u++)
	{
		fprintf(fPtr ,"%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t  %lf\t"
				" %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t", //
				MyVec[u]->sim_time,
				MyVec[u]->actual_torso_p_ICS[0],
				MyVec[u]->actual_torso_p_ICS[1],
				MyVec[u]->actual_torso_p_ICS[2],
				MyVec[u]->ref_torso_p_ICS[0],
				MyVec[u]->ref_torso_p_ICS[1],
				MyVec[u]->ref_torso_p_ICS[2],
				MyVec[u]->desired_torso_acc[2],
				MyVec[u]->desired_torso_acc[3],
				MyVec[u]->desired_torso_acc[4],
				MyVec[u]->JointTorque[5],
				MyVec[u]->JointTorque[6],
				MyVec[u]->JointTorque[7],
				MyVec[u]->JointTorque[8],
				MyVec[u]->ZMPx_ICS,
		        MyVec[u]->Rfx_ICS,
		        MyVec[u]->Lfx_ICS,
		        MyVec[u]->q2ss,
		        MyVec[u]->qd2ss,
		        MyVec[u]->q1ss,
		        MyVec[u]->qd1ss,
		        MyVec[u]->q0ss,
		        MyVec[u]->qd0ss,
		        MyVec[u]->q[2],
		        MyVec[u]->qd[2],
		        MyVec[u]->tr[2][0],
		        MyVec[u]->q[1]
		        );
		fprintf(fPtr,"\n");
	}


    fclose(fPtr);
    cout<<"done."<<endl<<endl;
}
