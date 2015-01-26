/*
 * PID.c
 *
 *  Created on: 2013-4-22
 *      Author: appleseed
 */
#include "PID.h"

#pragma CODE_SECTION(IncPIControl, "ramfuncs");
#pragma CODE_SECTION(IncPIDControl, "ramfuncs");

void IncPIDControlInit(sPIDParams *pParams){
	pParams->ProportionalGain = 0;
	pParams->IntergralGain = 0;
	pParams->DerivativeGain = 0;
	//pParams->PositivePIDLimit = 0;
	//pParams->NegativePIDLimit = 0;
	pParams->LastError = 0;
	pParams->PrevError = 0;
	pParams->MeasuredValue = 0;
	pParams->DesiredValue = 0;
}

float IncPIDControl(sPIDParams * pParams){
	float iError,iIncpid;
	iError = pParams->DesiredValue - pParams->MeasuredValue;
	iIncpid = (pParams->ProportionalGain * (iError - pParams->LastError)
			+ pParams->IntergralGain * iError
			+ pParams->DerivativeGain * (iError - 2 * pParams->LastError + pParams->PrevError));
	pParams->PrevError = pParams->LastError;
	pParams->LastError = iError;
	return iIncpid;
}

float IncPIControl(sPIDParams * pParams){
	float iError,iIncpid;
	iError = pParams->DesiredValue - pParams->MeasuredValue;
	iIncpid = (pParams->ProportionalGain * (iError - pParams->LastError)
			+ pParams->IntergralGain * iError);
	//pParams->PrevError = pParams->LastError;
	pParams->LastError = iError;
	return iIncpid;
}

