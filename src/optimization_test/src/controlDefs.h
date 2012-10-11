// controlDefs.h
// 

//  Sep 6, 2012
//  Project: Optimization Test
//  marcos, typedefs that are specific to robot controller
//  YL




#ifndef __CONTROL_DEFS__
#define __CONTROL_DEFS__


#include <dm.h>
#include <vector>

typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;
typedef vector<string > StringVector;
typedef vector<MatrixXF> MatXFVector;
typedef vector<VectorXF> VecXFVector;

typedef struct GRFInfoStruct 
{
	Vector3F pZMP;
	Vector3F fZMP;
	Float nZMP;
	
	int localContacts;
	vector<Vector3F > pCoPs;
	vector<Vector3F > fCoPs;
	vector<Float > nCoPs;
	
	vector<Vector6F> footWrenches;
	vector<MatrixXF> footJacs;
} GRFInfo;


typedef struct ControlInfoStruct   
{
	int iter;
	double calcTime;
	double setupTime;
	double optimTime;
	double totalTime;
}  ControlInfo ;

#define COPY_P_TO_VEC(p,pvec) pvec << p[0], p[1], p[2]; 

#define NJ 20 //actuated DOF
#define NS 2
#define NP 4
#define NF 4
#define MU 1

#endif

