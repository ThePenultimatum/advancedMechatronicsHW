#ifndef FILTERS_H__
#define FILTERS_H__
#include<xc.h>
// Header file for filters.c

// MAF

//void initMAFBuffer(int *buffer, int value, int numNums);

int getNextInd(int ind, int numNums);

void replaceIndMAF(int *buf, int ind, int val);

int getAvgFilter(int *buf, int numNums);

// IIR



// FIR

float getResFilterFIR(int *movingAvgFilterFIRBuf, float *firArr, int numFIRVals);

#endif