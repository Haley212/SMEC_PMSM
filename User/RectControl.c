/*
 * RectControl.c
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 */
#include "RectControl.h"

volatile struct RectSampleRawDataStruct RectSampleRawData;
volatile struct RectRunningDataStruct RectRunningData;

DQ GridVoltageDQ;
DQ GridCurrentDQ;
DQ GridRectOutDQ;			//控制量输出
sPIDParams GridCurrentDPI;
sPIDParams GridCurrentQPI;
sPIDParams DCBUSVoltagePI;

void RectControlInit(void){
	RectRunningData.GridAngle = 0;
	RectRunningData.ReactorCompensate = 2 * 3.141592654f * 50 * 2 * 0.001;
	RectRunningData.CurrentDPI = 0.0f;
	RectRunningData.CurrentQPI = 0.0f;
	RectRunningData.RectProtection.all = 0;
	RectRunningData.RectUOCCounter = 0;
	RectRunningData.RectVOCCounter = 0;
	RectRunningData.RectWOCCounter = 0;
	RectRunningData.RectDCSumErrCounter = 0;
	RectRunningData.RectDCDelErrCounter = 0;

	SOGI_PLL_Init(&SOGIPLLData);
	DQInit(&GridVoltageDQ);
	DQInit(&GridCurrentDQ);
	DQInit(&GridRectOutDQ);
	IncPIDControlInit(&GridCurrentDPI);
	IncPIDControlInit(&GridCurrentQPI);
	IncPIDControlInit(&DCBUSVoltagePI);
	GridCurrentDPI.ProportionalGain = 6.0f;
	GridCurrentDPI.IntergralGain = 0.016f;
	GridCurrentQPI.ProportionalGain = 6.0f;
	GridCurrentQPI.IntergralGain = 0.016f;
}

void ProcessRectSampleDataRoutine(void){
/*
	DCVoltage_POS_SUM = DCVoltage_POS_SUM + (unsigned int)RectSampleRawData.P_Voltage - (unsigned int)DCVoltage_POS[DCVoltage_POS_Index];
	DCVoltage_POS[DCVoltage_POS_Index++] = RectSampleRawData.P_Voltage;
	if(DCVoltage_POS_Index > 19){DCVoltage_POS_Index = 0;}
	DCVoltage_NEG_SUM = DCVoltage_NEG_SUM + (unsigned int)RectSampleRawData.N_Voltage - (unsigned int)DCVoltage_NEG[DCVoltage_NEG_Index];
	DCVoltage_NEG[DCVoltage_NEG_Index++] = RectSampleRawData.N_Voltage;
	if(DCVoltage_NEG_Index > 19){DCVoltage_NEG_Index = 0;}

	RectRunningData.POS_DC_Voltage = (float)DCVoltage_POS_SUM * 0.09214867614f * 0.05;
	RectRunningData.NEG_DC_Voltage = (float)DCVoltage_NEG_SUM * 0.09337287535f * 0.05;
*/
	RectRunningData.POS_DC_Voltage = (float)RectSampleRawData.P_Voltage * 0.093867595f * 0.5f;
	RectRunningData.NEG_DC_Voltage = (float)RectSampleRawData.N_Voltage * 0.094074521f * 0.5f;
	RectRunningData.DC_Voltage = RectRunningData.POS_DC_Voltage + RectRunningData.NEG_DC_Voltage;
	RectRunningData.DC_Delta = RectRunningData.POS_DC_Voltage - RectRunningData.NEG_DC_Voltage;
	RectRunningData.U_Voltage = ((float)RECT_U_VOLTAGE_OFFSET - (float)RectSampleRawData.U_Voltage) * RECT_U_VOLTAGE_GAIN;
	RectRunningData.V_Voltage = ((float)RECT_V_VOLTAGE_OFFSET - (float)RectSampleRawData.V_Voltage) * RECT_V_VOLTAGE_GAIN;
	RectRunningData.W_Voltage = ((float)RECT_W_VOLTAGE_OFFSET - (float)RectSampleRawData.W_Voltage) * RECT_W_VOLTAGE_GAIN;
//	RectRunningData.U_Current = ((float)RECT_U_CURRENT_OFFSET - (float)RectSampleRawData.U_Current) * RECT_U_CURRENT_GAIN;
//	RectRunningData.V_Current = ((float)RECT_V_CURRENT_OFFSET - (float)RectSampleRawData.V_Current) * RECT_V_CURRENT_GAIN;
//	RectRunningData.W_Current = ((float)RECT_W_CURRENT_OFFSET - (float)RectSampleRawData.W_Current) * RECT_W_CURRENT_GAIN;
//	这里反向了
	RectRunningData.U_Current = ((float)RectSampleRawData.U_Current - (float)RECT_U_CURRENT_OFFSET) * RECT_U_CURRENT_GAIN;
	RectRunningData.V_Current = ((float)RectSampleRawData.V_Current - (float)RECT_V_CURRENT_OFFSET) * RECT_V_CURRENT_GAIN;
	RectRunningData.W_Current = ((float)RectSampleRawData.W_Current - (float)RECT_W_CURRENT_OFFSET) * RECT_W_CURRENT_GAIN;
}

void GridPLLRoutine(void){
	/*PLL*/
	SOGIPLLData.U = RectRunningData.U_Voltage;
	SOGIPLLData.V = RectRunningData.V_Voltage;
	SOGIPLLData.W = RectRunningData.W_Voltage;
	SOGI_PLL_Update(&SOGIPLLData);
	RectRunningData.GridFreq = SOGIPLLData.GridFrequency;

//	RectRunningData.GridAngle = SOGIPLLData.theta;
//	GridVoltageDQ.sine_value = sin(SOGIPLLData.theta);
//	GridVoltageDQ.cosine_value = cos(SOGIPLLData.theta);

	GridVoltageDQ.sine_value = SOGIPLLData.theta_sin;
	GridVoltageDQ.cosine_value = SOGIPLLData.theta_cos;
	GridVoltageDQ.U = RectRunningData.U_Voltage;
	GridVoltageDQ.V = RectRunningData.V_Voltage;
	GridVoltageDQ.W = RectRunningData.W_Voltage;
	ABC2DQ(&GridVoltageDQ);
	RectRunningData.VoltageD = GridVoltageDQ.D;
	RectRunningData.VoltageQ = GridVoltageDQ.Q;

	GridCurrentDQ.sine_value = SOGIPLLData.theta_sin;
	GridCurrentDQ.cosine_value = SOGIPLLData.theta_cos;
	GridCurrentDQ.U = RectRunningData.U_Current;
	GridCurrentDQ.V = RectRunningData.V_Current;
	GridCurrentDQ.W = RectRunningData.W_Current;
	ABC2DQ(&GridCurrentDQ);
	RectRunningData.CurrentD = GridCurrentDQ.D;
	RectRunningData.CurrentQ = GridCurrentDQ.Q;

	GridRectOutDQ.sine_value = SOGIPLLData.theta_sin;
	GridRectOutDQ.cosine_value = SOGIPLLData.theta_cos;
}

void ProcessRectifierControlInit(void){
	RectRunningData.CurrentDPI = 0.0f;
	RectRunningData.CurrentQPI = 0.0f;
	RectRunningData.RectProtection.all = 0;
	RectRunningData.DesiredDCurrent = 0.0f;

//	SOGI_PLL_Init(&SOGIPLLData);
	DQInit(&GridVoltageDQ);
	DQInit(&GridCurrentDQ);
	DQInit(&GridRectOutDQ);
	IncPIDControlInit(&GridCurrentDPI);
	IncPIDControlInit(&GridCurrentQPI);
	IncPIDControlInit(&DCBUSVoltagePI);
	GridCurrentDPI.ProportionalGain = 6.0f * 1.0f;
	GridCurrentDPI.IntergralGain = 0.16f * 1.0f;
	GridCurrentQPI.ProportionalGain = 6.0f * 1.0f;
	GridCurrentQPI.IntergralGain = 0.16f * 1.0f;
	DCBUSVoltagePI.ProportionalGain = 0.3f * 1.0f;
	DCBUSVoltagePI.IntergralGain = 0.003f * 1.0f;
	SVMInit();
}

void ProcessRectifierPREP(void){
	DQ2ABC(&GridVoltageDQ);
	SVMArguments[DC_VOLTAGE] = RectRunningData.DC_Voltage * 0.5f;
	SVMArguments[VOLTAGE_SHIFT] = 0;
	SVMArguments[VOLTAGE_A] = GridVoltageDQ.U;
	SVMArguments[VOLTAGE_B] = GridVoltageDQ.V;
	SVMArguments[VOLTAGE_C] = GridVoltageDQ.W;
/*
	SVMArguments[VOLTAGE_A] =  15 * cos(RectRunningData.GridAngle);
	SVMArguments[VOLTAGE_B] =  15 * cos(RectRunningData.GridAngle + 4.188790205f);
	SVMArguments[VOLTAGE_C] =  15 * cos(RectRunningData.GridAngle + 2.094395102f);
*/
	SVPWM(SVMArguments,REC_PWM);
}

void ProcessRectifierControl(void){

	/*直流电压环*/
	if(MachineState == STATE_ONGRID){					//正常工作时
		DCBUSVoltagePI.MeasuredValue = RectRunningData.DC_Voltage;
		DCBUSVoltagePI.DesiredValue = 700.0f;
		RectRunningData.DesiredDCurrent += IncPIControl(&DCBUSVoltagePI);
		if(RectRunningData.DesiredDCurrent > MAX_DESIRED_D_CURRENT){RectRunningData.DesiredDCurrent = MAX_DESIRED_D_CURRENT;}
		if(RectRunningData.DesiredDCurrent < MIN_DESIRED_D_CURRENT){RectRunningData.DesiredDCurrent = MIN_DESIRED_D_CURRENT;}
	}else if(MachineState == STATE_DC_UNSTABLE){		//电压还在上升过程中
		DCBUSVoltagePI.MeasuredValue = RectRunningData.DC_Voltage;
		DCBUSVoltagePI.DesiredValue = 700.0f;
		RectRunningData.DesiredDCurrent += IncPIControl(&DCBUSVoltagePI);
		if(RectRunningData.DesiredDCurrent > MAX_CHARGING_D_CURRENT){RectRunningData.DesiredDCurrent = MAX_CHARGING_D_CURRENT;}
		if(RectRunningData.DesiredDCurrent < MIN_CHARGING_D_CURRENT){RectRunningData.DesiredDCurrent = MIN_CHARGING_D_CURRENT;}
	}

	/*D轴电流控制*/
	GridCurrentDPI.MeasuredValue = GridCurrentDQ.D;
	GridCurrentDPI.DesiredValue = RectRunningData.DesiredDCurrent;
	RectRunningData.CurrentDPI += IncPIControl(&GridCurrentDPI);
	if(RectRunningData.CurrentDPI > MAX_CURRENTDPI){RectRunningData.CurrentDPI = MAX_CURRENTDPI;}
	if(RectRunningData.CurrentDPI < MIN_CURRENTDPI){RectRunningData.CurrentDPI = MIN_CURRENTDPI;}
	GridRectOutDQ.D = GridVoltageDQ.D + RectRunningData.ReactorCompensate * GridCurrentDQ.Q - RectRunningData.CurrentDPI;
//	GridRectOutDQ.D = RectRunningData.ReactorCompensate * GridCurrentDQ.Q - RectRunningData.CurrentDPI;

	/*Q轴电流控制*/
	GridCurrentQPI.MeasuredValue = GridCurrentDQ.Q;
	GridCurrentQPI.DesiredValue = 0.0f;
	RectRunningData.CurrentQPI += IncPIControl(&GridCurrentQPI);
	if(RectRunningData.CurrentQPI > MAX_CURRENTQPI){RectRunningData.CurrentQPI = MAX_CURRENTQPI;}
	if(RectRunningData.CurrentQPI < MIN_CURRENTQPI){RectRunningData.CurrentQPI = MIN_CURRENTQPI;}
	GridRectOutDQ.Q = GridVoltageDQ.Q - RectRunningData.ReactorCompensate * GridCurrentDQ.D - RectRunningData.CurrentQPI;
//	GridRectOutDQ.Q = RectRunningData.ReactorCompensate * GridCurrentDQ.D - RectRunningData.CurrentQPI;

	DQ2ABC(&GridRectOutDQ);
	SVMArguments[DC_VOLTAGE] = RectRunningData.DC_Voltage * 0.5f;
	SVMArguments[VOLTAGE_SHIFT] = 0.0f;
#ifdef	NVC_ENABLE
	SVMArguments[VOLTAGE_SHIFT] = RectRunningData.DC_Delta * 0.1f;
#endif

	SVMArguments[CURRENT_A] = RectRunningData.U_Current;
	SVMArguments[CURRENT_B] = RectRunningData.V_Current;
	SVMArguments[CURRENT_C] = RectRunningData.W_Current;
	SVMArguments[VOLTAGE_A] = GridRectOutDQ.U;
	SVMArguments[VOLTAGE_B] = GridRectOutDQ.V;
	SVMArguments[VOLTAGE_C] = GridRectOutDQ.W;

/*	Open-Loop
	SVMArguments[VOLTAGE_OUT] = 220.0f;
	RectRunningData.GridAngle += 1.8f;	//50Hz*360degree/10KHz = 1.8degree Per Interrupt
	if(RectRunningData.GridAngle >= 360.0f){ RectRunningData.GridAngle -= 360.0f;}
	SVMArguments[VOLTAGE_A] =  SVMArguments[VOLTAGE_OUT] * cos((RectRunningData.GridAngle)/CONVERSION_OF_ARC);
	SVMArguments[VOLTAGE_B] =  SVMArguments[VOLTAGE_OUT] * cos((RectRunningData.GridAngle + 240.0f) / CONVERSION_OF_ARC);
	SVMArguments[VOLTAGE_C] =  SVMArguments[VOLTAGE_OUT] * cos((RectRunningData.GridAngle + 120.0f) / CONVERSION_OF_ARC);
*/
	SVPWM(SVMArguments,REC_PWM);
}

unsigned int RectProtectionRoutine(unsigned int fpga_fault){
#define RECT_OVER_CURRENT 25.0f
	//700V
#define RECT_DC_HIGH	370.0f
#define RECT_DC_LOW		330.0f
#define RECT_DCSUM_HIGH	750.0f
#define RECT_DCSUM_LOW	650.0f
#define RECT_DC_DELTA	60.0f

	if(RectRunningData.U_Current > RECT_OVER_CURRENT || RectRunningData.U_Current < -RECT_OVER_CURRENT){
		RectRunningData.RectUOCCounter ++;
	}else{
		RectRunningData.RectUOCCounter = 0;
	}

	if(RectRunningData.V_Current > RECT_OVER_CURRENT || RectRunningData.V_Current < -RECT_OVER_CURRENT){
		RectRunningData.RectVOCCounter ++;
	}else{
		RectRunningData.RectVOCCounter = 0;
	}

	if(RectRunningData.W_Current > RECT_OVER_CURRENT || RectRunningData.W_Current < -RECT_OVER_CURRENT){
		RectRunningData.RectWOCCounter ++;
	}else{
		RectRunningData.RectWOCCounter = 0;
	}

	if(RectRunningData.RectUOCCounter > 2){
		RectRunningData.RectProtection.bit.U_OVERCURRENT = 1;
		RectRunningData.RectErrMark.U_OVERCURRENT = RectRunningData.U_Current;
	}
	if(RectRunningData.RectVOCCounter > 2){
		RectRunningData.RectProtection.bit.V_OVERCURRENT = 1;
		RectRunningData.RectErrMark.V_OVERCURRENT = RectRunningData.V_Current;
	}
	if(RectRunningData.RectWOCCounter > 2){
		RectRunningData.RectProtection.bit.W_OVERCURRENT = 1;
		RectRunningData.RectErrMark.W_OVERCURRENT = RectRunningData.W_Current;
	}

	//下面是容错保护机制
	if(MachineState == STATE_ONGRID){				//直流电容充电完成
		if(RectRunningData.DC_Voltage > RECT_DCSUM_HIGH || RectRunningData.DC_Voltage < RECT_DCSUM_LOW){
			RectRunningData.RectDCSumErrCounter ++;
			RectRunningData.DC_Voltage = RectRunningData.DC_Voltage_old;
			RectRunningData.DC_Delta = RectRunningData.DC_Delta_old;
		}else{RectRunningData.RectDCSumErrCounter = 0;RectRunningData.DC_Voltage_old = RectRunningData.DC_Voltage;}
		if(RectRunningData.DC_Delta > RECT_DC_DELTA || RectRunningData.DC_Delta < -RECT_DC_DELTA){
			RectRunningData.RectDCDelErrCounter ++;
			RectRunningData.DC_Voltage = RectRunningData.DC_Voltage_old;
			RectRunningData.DC_Delta = RectRunningData.DC_Delta_old;
		}else{RectRunningData.RectDCDelErrCounter = 0;RectRunningData.DC_Delta_old = RectRunningData.DC_Delta;}
	}else if(MachineState == STATE_DC_UNSTABLE){	//直流电容还在充电
		if(RectRunningData.DC_Voltage > 730 || RectRunningData.DC_Voltage < 520){
			RectRunningData.RectDCSumErrCounter ++;
			RectRunningData.DC_Voltage = RectRunningData.DC_Voltage_old;
			RectRunningData.DC_Delta = RectRunningData.DC_Delta_old;
		}else{RectRunningData.RectDCSumErrCounter = 0;RectRunningData.DC_Voltage_old = RectRunningData.DC_Voltage;}
		if(RectRunningData.DC_Delta > RECT_DC_DELTA || RectRunningData.DC_Delta < -RECT_DC_DELTA){
			RectRunningData.RectDCDelErrCounter ++;
			RectRunningData.DC_Voltage = RectRunningData.DC_Voltage_old;
			RectRunningData.DC_Delta = RectRunningData.DC_Delta_old;
		}else{RectRunningData.RectDCDelErrCounter = 0;RectRunningData.DC_Delta_old = RectRunningData.DC_Delta;}
	}

	if(RectRunningData.RectDCSumErrCounter > 2){
		RectRunningData.RectProtection.bit.SUM_DC_ERR = 1;
		RectRunningData.RectErrMark.SUM_DC_ERR = RectRunningData.DC_Voltage;
	}
	if(RectRunningData.RectDCDelErrCounter > 2){
		RectRunningData.RectProtection.bit.DEL_DC_ERR = 1;
		RectRunningData.RectErrMark.DEL_DC_ERR = RectRunningData.DC_Delta;
	}
	//下面是原先的直流电压保护
/*	if(MachineState == STATE_ONGRID){
		if(RectRunningData.POS_DC_Voltage > RECT_DC_HIGH || RectRunningData.POS_DC_Voltage < RECT_DC_LOW){RectRunningData.RectProtection.bit.POS_DC_ERR = 1;RectRunningData.RectErrMark.POS_DC_ERR = RectRunningData.POS_DC_Voltage;}
		if(RectRunningData.NEG_DC_Voltage > RECT_DC_HIGH || RectRunningData.NEG_DC_Voltage < RECT_DC_LOW){RectRunningData.RectProtection.bit.NEG_DC_ERR = 1;RectRunningData.RectErrMark.NEG_DC_ERR = RectRunningData.NEG_DC_Voltage;}
		if(RectRunningData.DC_Voltage > RECT_DCSUM_HIGH || RectRunningData.DC_Voltage < RECT_DCSUM_LOW){RectRunningData.RectProtection.bit.SUM_DC_ERR = 1;RectRunningData.RectErrMark.SUM_DC_ERR = RectRunningData.DC_Voltage;}
		if(RectRunningData.DC_Delta > RECT_DC_DELTA || RectRunningData.DC_Delta < -RECT_DC_DELTA){RectRunningData.RectProtection.bit.DEL_DC_ERR = 1;RectRunningData.RectErrMark.DEL_DC_ERR = RectRunningData.DC_Delta;}
	}else if(MachineState == STATE_DC_UNSTABLE){
		if(RectRunningData.POS_DC_Voltage > 370 || RectRunningData.POS_DC_Voltage < 260){RectRunningData.RectProtection.bit.POS_DC_ERR = 1;RectRunningData.RectErrMark.POS_DC_ERR = RectRunningData.POS_DC_Voltage;}
		if(RectRunningData.NEG_DC_Voltage > 370 || RectRunningData.NEG_DC_Voltage < 260){RectRunningData.RectProtection.bit.NEG_DC_ERR = 1;RectRunningData.RectErrMark.NEG_DC_ERR = RectRunningData.NEG_DC_Voltage;}
		if(RectRunningData.DC_Voltage > 730 || RectRunningData.DC_Voltage < 520){RectRunningData.RectProtection.bit.SUM_DC_ERR = 1;RectRunningData.RectErrMark.SUM_DC_ERR = RectRunningData.DC_Voltage;}
		if(RectRunningData.DC_Delta > RECT_DC_DELTA || RectRunningData.DC_Delta < -RECT_DC_DELTA){RectRunningData.RectProtection.bit.DEL_DC_ERR = 1;RectRunningData.RectErrMark.DEL_DC_ERR = RectRunningData.DC_Delta;}
	}*/
	if(RectRunningData.GridFreq > 52.0f || RectRunningData.GridFreq < 48.0f){RectRunningData.RectProtection.bit.GRID_FREQ_ERR = 1;RectRunningData.RectErrMark.GRID_FREQ_ERR = RectRunningData.GridFreq;}

	if(RectRunningData.VoltageD < 250.0f || RectRunningData.VoltageD > 370.0f){
		GridVoltageDQ.D = 310.0f;
		GridVoltageDQ.Q = 0.0f;
/*		RectRunningData.RectProtection.bit.GRID_VOLTAGE_ERR = 1;
		RectRunningData.RectErrMark.GRID_VOLTAGE_ERR = RectRunningData.VoltageD;
		RectRunningData.RectErrMark.GRID_VOLTAGE_ERRQ = RectRunningData.VoltageQ;
		RectRunningData.RectErrMark.GRID_VOLTAGE_U = RectRunningData.U_Voltage;
		RectRunningData.RectErrMark.GRID_VOLTAGE_V = RectRunningData.V_Voltage;
		RectRunningData.RectErrMark.GRID_VOLTAGE_W = RectRunningData.W_Voltage;
		RectRunningData.RectErrMark.GRID_ANGLE = SOGIPLLData.theta;
*/
	}

//	if(fpga_fault & 0x02){RectRunningData.RectProtection.bit.FPGA_OVERHEAT = 1;}
	if(fpga_fault & 0x04){RectRunningData.RectProtection.bit.FPGA_OVERCURRENT = 1;}
	return RectRunningData.RectProtection.all;
}
