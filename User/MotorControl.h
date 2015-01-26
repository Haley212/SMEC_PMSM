/*
 * InvControl.h
 *
 *  Created on: 2014-10-27
 *      Author: appleseed
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "Global.h"

extern volatile float GivenSpeed;
extern volatile unsigned int OpenLoopBoolean;
extern volatile float SpeedRef;

struct InvSampleRawDataStruct{		//������õ�ԭʼ����
	unsigned int U_Current;			//U�����
	unsigned int V_Current;			//V�����
	unsigned int W_Current;			//W�����
	unsigned int P_Voltage;			//�ϵ��ݵ�ѹ
	unsigned int N_Voltage;			//�µ��ݵ�ѹ
};
extern volatile struct InvSampleRawDataStruct InvSampleRawData;

struct InvProtectionBits{
	unsigned int U_OVERCURRENT:1;
	unsigned int V_OVERCURRENT:1;
	unsigned int W_OVERCURRENT:1;
	unsigned int OVER_SPEED:1;
	unsigned int RSVD:12;
};

union InvProtectionStruct{
	unsigned int all;
	struct InvProtectionBits bit;
};

struct InvProtectionMark{
	float U_OVERCURRENT;
	float V_OVERCURRENT;
	float W_OVERCURRENT;
	float OVER_SPEED;
	float OVER_SPEED_PR;
	float U_D;
	float U_Q;
};

struct InvRunningDataStruct{
	unsigned char DirectionQep;		//�������
	float MotorAngle;				//�����Ƕ�
	float MotorSpeed;				//����ٶ�
	float POS_DC_Voltage;			//�ϵ��ݵ�ѹ
	float NEG_DC_Voltage;			//�µ��ݵ�ѹ
	float DC_Voltage;				//ֱ����ѹ
	float DC_Delta;					//ֱ����ѹ

	float U_Voltage;
	float V_Voltage;
	float W_Voltage;

	float U_Current;
	float V_Current;
	float W_Current;

	float CurrentD;					//D�����
	float CurrentQ;					//Q�����

	float TorqueQ;					//Q��ָ�����
#define MAX_TORQUE_CURRENT		5
#define MIN_TORQUE_CURRENT		-5
	float CurrentDPI;				//PI�����ֱ��
#define	MAX_MOTOR_CURRENTDPI	200
#define MIN_MOTOR_CURRENTDPI	-200
	float CurrentQPI;				//PI�����ֱ��
#define MAX_MOTOR_CURRENTQPI	200
#define MIN_MOTOR_CURRENTQPI	-200
	float ReactorCompensate;
	union InvProtectionStruct InvProtection;
	struct InvProtectionMark InvErrMark;

	unsigned int MotorUOCCounter;
	unsigned int MotorVOCCounter;
	unsigned int MotorWOCCounter;

	unsigned int MotorPWMPREP;
	float MotorVOutD;
	float MotorVOutQ;
};
extern volatile struct InvRunningDataStruct InvRunningData;

void InvControlInit(void);

void ProcessInvSampleDataRoutine(void);

void ProcessInverterControl(void);

void ProcessMotorSpeedLoop(void);			//�ٶȻ�

unsigned int InvProtectionRoutine(void);	//�������������1��ʾ������

struct MotorSpeed{
	float theta_elec;		//Output: Motor Electrical angle
	float theta_mech;		//Output: Motor Mechanical angle
	int	DirectionQeq;		//Output: Motor rotation direction
	float theta_raw;		//Variable
	float mech_scaler;		//Parameter: mech angle convert
	float elec_scaler;		//Parameter:
	int pole_pairs;			//Parameter: Number of pole pairs
	float cal_angle;		//Parameter: elec angular offset between encoder and phase a
	int	index_sync_flag;	//Ouptut: Motor Z Index sync
	int speed_sync_flag;	//Output: MotorSpeed detect sync, �����ٶȻ�PI����
//	float old_position;
//	float new_position;
//	float del_position;
	signed long	old_position_uint;
	signed long	new_position_uint;
	signed long	del_position_uint;
	float SpeedRpm_fr;		//Output: MotorSpeed(rpm)
	float BaseRpm;			//Paramter: index to rpm
	float SpeedRpm_pr;		//Output: MotorSpeed(rpm)
	float SpeedScaler;
	float omega;			//w,����ٶ�

	unsigned int F[4];		//���������صĶ�λ��
	unsigned int F_Index;
	float F_theta;
	unsigned int F_Index_last;
	unsigned int F_Count;
};
extern volatile struct MotorSpeed MotorSpeedData;

void EncoderInit(int init_rotor_position);			//��������ʼ��������1Ϊ�����ת��λ�ã�0Ϊ�������ת��λ��
void ProcessEncoder(void);							//������ת�٣���õ��λ��

#endif /* INVTCONTROL_H_ */
