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
#include "wx/wx.h"

typedef vector<Float> FloatVector;
typedef vector<int> IntVector;

class DataLogger {
public:
	DataLogger();
	void newRecord();
	
	
	void writeRecords();
	void setFile(const string & fName);
	
	
	void assignItem(int code, Float value);
	void assignGroup(int groupCode, const VectorXF & value);
	void assignMatrixGroup(int groupCode, const MatrixXF & value);
	
	int  addItem(const string & displayName, const string & matlabName);
	int  declareGroup(const string & displayName, IntVector & itemCodes); //declare group from existing items
	int	 addGroup(const string & displayName, const string & matlabName, int size);
	int  addMatrixGroup(const string & displayName, const string & matlabName, int rowSize, int colSize);
	
	wxMutex dataMutex;
	
private:
	FloatVector * curr;
	deque<FloatVector *> data;
	string fileName;
	vector<string> itemNames;
	vector<string> matlabItemNames;
	vector<IntVector> groups; 
	vector<string> groupNames;
	
	int maxItems, maxGroups;
};




#endif