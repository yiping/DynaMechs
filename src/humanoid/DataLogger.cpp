/*
 *  untitled.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/11/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "DataLogger.h"


DataLogger::DataLogger() {
	curr = NULL;
}
void DataLogger::newRecord() {
	curr = new FloatVector(maxItems);
	data.push_back(curr);
}

void DataLogger::assignItem(int code, Float value) {
	#ifdef DATA_LOG_DEBUG
		if (curr == NULL) {
			cout << "ASSERTION FAILED: No current record! " << endl;
			exit(-1);
		}
	#endif
	curr->at(code) = value;
	
}
void DataLogger::assignGroup(int groupCode, const VectorXF & value) {
	#ifdef DATA_LOG_DEBUG
		if (curr == NULL) {
			cout << "ASSERTION FAILED: No current record! " << endl;
			exit(-1);
		}
	#endif
	const unsigned int groupSize = groups[groupCode].size();
	IntVector * g = &(groups[groupCode]);
	for (unsigned int i=0; i<groupSize; i++) {
		curr->at(g->at(i)) = value(i);
	}
}

void DataLogger::writeRecords(){
	FILE * fPtr = fopen(fileName.c_str(), "a");
	while (data.size() > 0) {
		curr = data[0];
		for (int i=0; i < maxItems; i++) {
			fprintf(fPtr, "%lf\t",(double) curr->at(i) );
		}
		fprintf(fPtr,"\n");
		data.pop_front();
	}
	curr = NULL;
	fclose(fPtr);
}

void DataLogger::setFile(const string & fName) {
	fileName = fName;
	FILE * fPtr = fopen(fileName.c_str(),"w");
	if (fPtr == NULL) {
		cout << "Problem Opening File " << fName << endl;
		exit(-1);
	}
	for (int i = 0; i<maxItems; i++) {
		fprintf(fPtr, "%s\t",itemNames[i].c_str());
	}
	fprintf(fPtr,"\n");
	fclose(fPtr);
	
}

int DataLogger::addMatrixGroup(const string & displayName, const string & matlabName, int rowSize, int colSize) {
	IntVector itemCodes(rowSize*colSize);
	int k=0;
	for (int i=0; i<rowSize; i++) {
		for (int j=0; j<colSize; j++) {
			stringstream ss;
			ss << displayName << "[" << i << ", " << j << "]"; 
			itemNames.push_back(ss.str());
			
			ss.clear();
			ss << matlabName << "(" << i+1 << ", " << j+1 << ")"; 
			matlabItemNames.push_back(ss.str());
			itemCodes[k++] = itemNames.size();
			maxItems++;
		}
	}
	
	groups.push_back(itemCodes);
	groupNames.push_back(displayName);
	maxGroups++;
	return maxGroups;
}

int DataLogger::addGroup(const string & displayName, const string & matlabName, int size) {
	IntVector itemCodes(size);
	for (int i=0; i<size; i++) {
		stringstream ss;
		ss << displayName << "[" << i << "]"; 
		itemNames.push_back(ss.str());
		
		ss.clear();
		ss << matlabName << "(" << i+1 << ")"; 
		matlabItemNames.push_back(ss.str());
		
		itemCodes[i] = itemNames.size();
		maxItems++;
	}
	
	groups.push_back(itemCodes);
	groupNames.push_back(displayName);
	maxGroups++;
	return maxGroups;
}

void DataLogger::setMaxGroups(int maxG) {
	maxGroups = maxG;
	groups.resize(maxGroups);
	groupNames.resize(maxGroups);
}

void DataLogger::setMaxItems(int maxI) {
	maxItems = maxI;
	itemNames.resize(maxItems);
	matlabItemNames.resize(maxItems);
}

void DataLogger::declareGroup(int groupCode, const string & name, IntVector & itemCodes) {
	groups[groupCode] = itemCodes;
	groupNames[groupCode] = name;
}

void DataLogger::setItemName(int itemCode, const string & s, const string & s2) {
	itemNames[itemCode] = s;
	matlabItemNames[itemCode] = s2;
}