// OTDataLogger.h
// Sep 3, 2012
// Project: Optimization Test
// YL

#ifndef __OT_DATA_LOGGER_H__
#define __OT_DATA_LOGGER_H__


#include "DataLogger.h"
#include <string>
using namespace std;

class OTDataLogger : public DataLogger
{
	
public:	
	OTDataLogger();
	void logData();
	void saveToFile();
	string dataSaveDirectory;
private:
	//
	int SIM_TIME;  // 

	// group
};
#endif
