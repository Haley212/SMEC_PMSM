/*
 * RectControl.h
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 *      整流器控制
 */

#ifndef RECTCONTROL_H_
#define RECTCONTROL_H_

#include "Global.h"

struct RectSampleRawDataStruct{		//采样获得的原始数据
	unsigned int U_Voltage;			//U相电压
	unsigned int V_Voltage;			//V相电压
	unsigned int W_Voltage;			//W相电压
	unsigned int U_Current;			//U相电流
	unsigned int V_Current;			//V相电流
	unsigned int W_Current;			//W相电流
	unsigned int P_Voltage;			//上电容电压
	unsigned int N_Voltage;			//下电容电压
};
extern volatile struct RectSampleRawDataStruct RectSampleRawData;

struct RectProtectionBits{
	unsigned int U_OVERCURRENT:1;
	unsigned int V_OVERCURRENT:1;
	unsigned int W_OVERCURRENT:1;
	unsigned int POS_DC_ERR:1;
	unsigned int NEG_DC_ERR:1;
	unsigned int SUM_DC_ERR:1;
	unsigned int DEL_DC_ERR:1;
	unsigned int GRID_FREQ_ERR:1;
	unsigned int GRID_VOLTAGE_ERR:1;
	unsigned int FPGA_OVERCURRENT:1;
	unsigned int FPGA_OVERHEAT:1;
	unsigned int RSVD:5;
};

union RectProtectionStruct{
	unsigned int all;
	struct RectProtectionBits bit;
};

struct RectProtectionMark{
	float U_OVERCURRENT;
	float V_OVERCURRENT;
	float W_OVERCURRENT;
	float POS_DC_ERR;
	float NEG_DC_ERR;
	float SUM_DC_ERR;
	float DEL_DC_ERR;
	float GRID_FREQ_ERR;
	float GRID_VOLTAGE_ERR;
	float GRID_VOLTAGE_ERRQ;
	float GRID_VOLTAGE_U;
	float GRID_VOLTAGE_V;
	float GRID_VOLTAGE_W;
	float GRID_ANGLE;
	float FPGA_OVERCURRENT;
	float FPGA_OVERHEAT;
};

struct RectRunningDataStruct{
	float GridAngle;				//电网角度
	float GridFreq;					//电网频率
	float POS_DC_Voltage;			//上电容电压
	float NEG_DC_Voltage;			//下电容电压
	float DC_Voltage;				//直流电压
	float DC_Delta;					//直流电压
	float DC_Voltage_old;
	float DC_Delta_old;
	float U_Voltage;
	float V_Voltage;
	float W_Voltage;
	float VoltageD;
	float VoltageQ;
	float U_Current;
	float V_Current;
	float W_Current;
	float CurrentD;					//D轴电流
	float CurrentQ;					//Q轴电流
	float DesiredDCurrent;			//D轴直流电流，Q轴为0
#define MAX_DESIRED_D_CURRENT	40
#define MIN_DESIRED_D_CURRENT	-40
#define MAX_CHARGING_D_CURRENT	5
#define MIN_CHARGING_D_CURRENT	-5
	float CurrentDPI;				//PI后电流直流
#define	MAX_CURRENTDPI	100
#define MIN_CURRENTDPI	-100
	float CurrentQPI;				//PI后电流直流
#define MAX_CURRENTQPI	100
#define MIN_CURRENTQPI	-100
	float ReactorCompensate;

	union RectProtectionStruct RectProtection;
	struct RectProtectionMark RectErrMark;
	unsigned int RectUOCCounter;
	unsigned int RectVOCCounter;
	unsigned int RectWOCCounter;
	unsigned int RectDCSumErrCounter;
	unsigned int RectDCDelErrCounter;
};
extern volatile struct RectRunningDataStruct RectRunningData;
// 整流器上电时初始化，在程序上电时调用
void RectControlInit(void);
// 整流器采样数据处理
void ProcessRectSampleDataRoutine(void);
// 电网锁相环，每个中断周期必须都执行
void GridPLLRoutine(void);

// 整流器控制前初始化，并网前必须调用，锁相环不在这里初始化
void ProcessRectifierControlInit(void);
// STATE_PREP 状态下，FPGA发出PWM波，但RPWM_DISABLE
// LCL充电
void ProcessRectifierPREP(void);

// 整流器控制
void ProcessRectifierControl(void);
// 整流器正常工作时的保护
unsigned int RectProtectionRoutine(unsigned int fpga_fault);	//软件保护，返回1表示有问题

#endif /* RECTCONTROL_H_ */
