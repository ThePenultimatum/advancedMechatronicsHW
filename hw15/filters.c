#include<xc.h>
#include "filters.h"

// MAF

/*void initMAFBuffer(int *buffer, int value, int numNums) {
    int i = 0;
    for (i = 0; i < numNums; i++) {
        buffer[i] = value;
    }
}*/

int getNextInd(int ind, int numNums) {
    return ((ind+1) % numNums);
}

void replaceIndMAF(int *buf, int ind, int val) {
    buf[ind] = val;
}

int getAvgFilter(int *buf, int numNums) {
    int i = 0;
    int sum = 0;
    for (i=0; i < numNums; i++) {
        sum += buf[i];
    }
    return (sum / numNums);
}

// IIR



// FIR

float getResFilterFIR(int *movingAvgFilterFIRBuf, float *firArr, int numFIRVals) {
    int i = 0;
    float sum = 0;
    for (i = 0; i < numFIRVals; i++) {
        sum += movingAvgFilterFIRBuf[i] * firArr[i];
    }
    return sum;
}