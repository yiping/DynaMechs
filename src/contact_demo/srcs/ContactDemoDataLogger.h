// ContactDemoDataLogger.h
// July 12, 2012
// YL

#ifndef __CONTACT_DEMO_DATA_LOGGER_H__
#define __CONTACT_DEMO_DATA_LOGGER_H__


#include "DataLogger.h"
#include <string>
using namespace std;

class ContactDemoDataLogger : public DataLogger
{
	
public:	
	ContactDemoDataLogger();
	void logData();
	void saveToFile();
	string dataSaveDirectory;
private:
	int SIM_TIME, NORMAL_PENETRATION, NORMAL_VELOCITY;  // designed for only one contact point
	int G_FE_TERRAIN,
		G_FE_PLANAR_DAMPER_TERRAIN, 
		G_EXT_F, 
		G_BOX_POS_ICS, 
		G_BOX_VEL, 
		G_SLIDING_FLAGS, 
		G_CONTACT_FLAGS;
		//G_ANCHOR_0_ICS,
		//G_ANCHOR_1_ICS,
		//G_ANCHOR_2_ICS,
		//G_ANCHOR_3_ICS;
};
#endif
