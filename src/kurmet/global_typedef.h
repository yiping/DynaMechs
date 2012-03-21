
#ifndef GLOBAL_H // header guards 
#define GLOBAL_H 

#include <dm.h>  

struct DataRecord{  
	Float sim_time;

	//system state
	Float q[9];
	Float qd[9];

	// torso orientation/position error (in local coordinate)
	//Float error[6];



	Float q2ss;
	Float qd2ss;
	Float q1ss;
	Float qd1ss;
	Float q0ss;
	Float qd0ss;
	
	// desired torso acc
	Float desired_torso_acc[6];

	// computed leg joint torques 
	Float JointTorque[9];

	// desired torso position ICS
	Float ref_torso_p_ICS[3];
	// actual torso position ICS
	Float actual_torso_p_ICS[3];

	Float ZMPx_ICS;
	Float actual_ZMPx_ICS;
	Float Rfx_ICS;
	Float Lfx_ICS;

	// computed torso torque
	Float computed_tq2;

	Float tr[9][1];

};

typedef vector<DataRecord *> DataRecVector;  
#endif
