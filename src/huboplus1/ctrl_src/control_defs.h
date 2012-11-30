

#ifndef __CONTROL_DEFS__
#define __CONTROL_DEFS__
#include <dm.h>


#define NJ 26
#define NS 2
#define NP 4
#define NF 4

#define MU .6

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

typedef vector<int> IntVector;
typedef vector<MatrixX6F> XformVector;

typedef vector<string > StringVector;

#endif
