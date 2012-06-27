/*
 *  untitled.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/11/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "DataLogger.h"
#include "GlobalDefines.h"
#include <stdio.h>

#define DATA_LOG_DEBUG

DataLogger::DataLogger() {
	curr = NULL;
}
void DataLogger::newRecord() {
	curr = new FloatVector(maxItems);
	data.push_back(curr);
	
	/*for (int i = 0 ; i< maxGroups; i++) {
		cout << "Group  " << i << " ='" << groupNames[i] << "' Size " << groups[i].size() << endl;
	}
	
	for (int i=0; i<maxItems; i++) {
		cout << "item " << i << " = " << itemNames[i] << endl;
	}*/
	return;
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
	//cout << "Group " << groupCode  << " (" << groupNames[groupCode] << ") Size " << groupSize << " vs. " << value.size() << endl;
	
	IntVector * g = &(groups[groupCode]);
	for (unsigned int i=0; i<groupSize; i++) {
		curr->at(g->at(i)) = value(i);
	}
}

void DataLogger::assignMatrixGroup(int groupCode, const MatrixXF & value) {
#ifdef DATA_LOG_DEBUG
	if (curr == NULL) {
		cout << "ASSERTION FAILED: No current record! " << endl;
		exit(-1);
	}
	const unsigned int groupSize = groups[groupCode].size();
	if (groupSize != value.size()) {
		cout << "ASSERTION FAILED: Matrix Group Size does not match Matrix Data Size!" <<endl;
		exit(-1);
	}
#endif
	
	
	//cout << "Group " << groupCode  << " (" << groupNames[groupCode] << ") Size " << groupSize << " vs. " << value.size() << endl;
	
	IntVector * g = &(groups[groupCode]);
	int k=0;
	for (unsigned int j=0; j<value.cols(); j++) {
		for (unsigned int i=0; i<value.rows(); i++) {
			//cout << "Assigning item " << g->at(k) << " '" << itemNames[g->at(k)] << "' = " << value(i,j) << endl;
			curr->at(g->at(k)) = value(i,j);
			k++;
		}
	}
	
}

void DataLogger::writeRecords(){
	dataMutex.Lock();
	FILE * fPtr = fopen(fileName.c_str(), "a");
	int items = data.size();
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
	cout << "Wrote " << items << " records to " << fileName << endl;
	dataMutex.Unlock();
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
		if (i !=(maxItems-1)) {
			fprintf(fPtr, "; ");
		}
	}
	fprintf(fPtr,"\n");
	for (int i = 0; i<maxItems; i++) {
		fprintf(fPtr, "%s\t",matlabItemNames[i].c_str());
		if (i !=(maxItems-1)) {
			fprintf(fPtr, "; ");
		}
	}
	fprintf(fPtr,"\n");
	fclose(fPtr);
	
}

int DataLogger::addMatrixGroup(const string & displayName, const string & matlabName, int rowSize, int colSize) {
	IntVector itemCodes(rowSize*colSize);
	int k=0;
	for (int j=0; j<colSize; j++) {
		for (int i=0; i<rowSize; i++) {
		
			stringstream ss;
			ss << displayName << "[" << i << ", " << j << "]"; 
			itemNames.push_back(ss.str());
			
			ss.str(std::string());
			ss << matlabName << "(" << i+1 << ", " << j+1 << ")"; 
			matlabItemNames.push_back(ss.str());
			itemCodes[k++] = itemNames.size()-1;
			maxItems++;
		}
	}
	
	groups.push_back(itemCodes);
	groupNames.push_back(displayName);
	maxGroups++;
	return maxGroups-1;
}

int DataLogger::addGroup(const string & displayName, const string & matlabName, int size) {
	IntVector itemCodes(size);
	for (int i=0; i<size; i++) {
		stringstream ss;
		ss << displayName << "[" << i << "]"; 
		itemNames.push_back(ss.str());
		
		ss.str(std::string());
		ss << matlabName << "(" << i+1 << ")"; 
		matlabItemNames.push_back(ss.str());
		
		itemCodes[i] = itemNames.size()-1;
		maxItems++;
	}
	
	groups.push_back(itemCodes);
	groupNames.push_back(displayName);
	maxGroups++;
	return maxGroups-1;
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