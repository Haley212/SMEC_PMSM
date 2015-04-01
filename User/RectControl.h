/*
 * RectControl.h
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 *      ����������
 */

#ifndef RECTCONTROL_H_
#define RECTCONTROL_H_

#include "Global.h"

struct RectSampleRawDataStruct{		//������õ�ԭʼ����
	unsigned int U_Voltage;			//U���ѹ
	unsigned int V_Voltage;			//V���ѹ
	unsigned int W_Voltage;			//W���ѹ
	unsigned int U_Current;			//U�����
	unsigned int V_Current;			//V�����
	unsigned int W_Current;			//W�����
	unsigned int P_Voltage;			//�ϵ��ݵ�ѹ
	unsigned int N_Voltage;			//�µ��ݵ�ѹ
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
	float GridAngle;				//�����Ƕ�
	float GridFreq;					//����Ƶ��
	float POS_DC_Voltage;			//�ϵ��ݵ�ѹ
	float NEG_DC_Voltage;			//�µ��ݵ�ѹ
	float DC_Voltage;				//ֱ����ѹ
	float DC_Delta;					//ֱ����ѹ
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
	float CurrentD;					//D�����
	float CurrentQ;					//Q�����
	float DesiredDCurrent;			//D��ֱ��������Q��Ϊ0
#define MAX_DESIRED_D_CURRENT	40
#define MIN_DESIRED_D_CURRENT	-40
#define MAX_CHARGING_D_CURRENT	5
#define MIN_CHARGING_D_CURRENT	-5
	float CurrentDPI;				//PI�����ֱ��
#define	MAX_CURRENTDPI	100
#define MIN_CURRENTDPI	-100
	float CurrentQPI;				//PI�����ֱ��
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
// �������ϵ�ʱ��ʼ�����ڳ����ϵ�ʱ����
void RectControlInit(void);
// �������������ݴ���
void ProcessRectSampleDataRoutine(void);
// �������໷��ÿ���ж����ڱ��붼ִ��
void GridPLLRoutine(void);

// ����������ǰ��ʼ��������ǰ������ã����໷���������ʼ��
void ProcessRectifierControlInit(void);
// STATE_PREP ״̬�£�FPGA����PWM������RPWM_DISABLE
// LCL���
void ProcessRectifierPREP(void);

// ����������
void ProcessRectifierControl(void);
// ��������������ʱ�ı���
unsigned int RectProtectionRoutine(unsigned int fpga_fault);	//�������������1��ʾ������

#endif /* RECTCONTROL_H_ */
