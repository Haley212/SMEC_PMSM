/*
 * PID.h
 *
 *  Created on: 2013-4-22
 *      Author: appleseed
 */

#ifndef PID_H_
#define PID_H_

typedef struct{
	float ProportionalGain;
	float IntergralGain;
	float DerivativeGain;
	//float PositivePIDLimit;
	//float NegativePIDLimit;
	float LastError;
	float PrevError;
	float MeasuredValue;
	float DesiredValue;
}sPIDParams;
//PID calculate
extern void IncPIDControlInit(sPIDParams *pParams);
extern float IncPIDControl(sPIDParams *pParams);
extern float IncPIControl(sPIDParams *pParams);


#endif /* PID_H_ */
