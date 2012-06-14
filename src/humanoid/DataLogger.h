/*
 *  DataLogger.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/11/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __DATALOGGERH__
#define __DATALOGGERH__

#include <dm.h>
#include "GlobalTypes.h"
#include <string>
#include <deque> 
#include <vector>

typedef vector<Float> FloatVector;

class DataLogger {
	public:
		DataLogger();
		void newRecord();
	
		void assignItem(int code, Float value);
		void assignGroup(int groupCode, const VectorXF & value);
		
		void writeRecords();
		void setFile(const string & fName);
	
		void setMaxGroups(int);
		void setMaxItems(int);
		void declareGroup(int groupCode, IntVector & itemCodes);
		void setItemName(int itemCode, string &);
	
	private:
		FloatVector * curr;
		deque<FloatVector *> data;
		string fileName;
		vector<string> itemNames;
		vector<IntVector> groups; 
		int maxItems, maxGroups;
};




#endif