/*
 * SVM.h
 *
 *  Created on: 2013-3-19
 *      Author: appleseed
 */
#ifndef SVM_H
#define SVM_H
#include "PID.h"

#define SVM_ARG_L		13
#define	VOLTAGE_A		0
#define VOLTAGE_B		1
#define VOLTAGE_C		2
#define CURRENT_A		3
#define CURRENT_B		4
#define CURRENT_C		5
#define FREQUENCE		6
#define	SVM_ANGLE		7
#define DC_VOLTAGE		8		//单个电容上的电压值
#define VOLTAGE_OUT		9		//期望电压
#define VOLTAGE_ACT 	10		//实际输出电压
#define VOLTAGE_OPI 	11
#define	VOLTAGE_SHIFT	12		//中点电位偏差

#define CONVERSION_OF_ARC	57.29578f
#define HALF_SQRT_3 		0.866025403f
#define SQRT_3				1.732050808f
#define ONE_THIRD_SQRT_3	0.577350269f

/*
typedef struct{
#define DQ_BUFFER_LENGTH 2
	float Ua;
	float Ub;
	float Uc;
	float Ia;
	float Ib;
	float Ic;
	float Ud;
	float Uq;
	float Ud_back;
	float Uq_back;
	float Id;
	float Iq;
	float Id_back;
	float Iq_back;
	float Theta;
	float Sin_Theta;
	float Cos_Theta;
	float IdBuffer[DQ_BUFFER_LENGTH];
	float IqBuffer[DQ_BUFFER_LENGTH];
	float IdAvg;
	float IqAvg;
	unsigned char IdqBufferPointer;
}sDQ;
*/
extern float SVMArguments[SVM_ARG_L];
/*
extern sPIDParams DCurrentPI;
extern sPIDParams QCurrentPI;
extern sPIDParams MidVoltagePI;
extern sDQ DQ;
*/
extern float UrefLength;
extern unsigned int REC_PWM[3];
extern unsigned int INV_PWM[3];

void SVMInit(void);
void SVPWM(float * Arg, unsigned int *PWM);//SVPWM Algo
static void SVPWMCalcuSector(unsigned char *Sector);
/*
void  DQInit(void);
void  ABC2DQ(sDQ * v);
void  DQ2ABC(sDQ * v);
void  DQStoreAndAvg(sDQ *v);
*/
typedef struct{
#define DQ_BUFFER_LENGTH 2
	float U;
	float V;
	float W;
	float cosine_value;
	float sine_value;
	float D;
	float Q;
}DQ;

void  DQInit(DQ * v);
void  ABC2DQ(DQ * v);
void  DQ2ABC(DQ * v);

#endif
