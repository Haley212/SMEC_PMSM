/*
 * Global.h
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "Menu.h"
#include "Printf_sci.h"
#include "RectControl.h"
#include "MotorControl.h"
#include "SVM.h"
#include "math.h"
#include "PID.h"
#include "SOGI_PLL.h"
#include "stdio.h"

#define DSP_VERSION	3112

//	������״̬
#define STATE_IDLE			0		//	����״̬
#define STATE_RESIS_CHARGE	1		//	Ԥ�������У��ȴ���絽һ����ѹֵ
#define STATE_PREP			2		//	Ԥ�����������̵����պϣ��ȴ���ѹ�ȶ���FPGA��ʼ����PWM����������IGBT
#define STATE_DC_UNSTABLE	3		//	PWM����������ֱ��ĸ�ߵ�ѹ700V���˽׶εĵ�ѹ��������
#define STATE_ONGRID		4
#define	STATE_ERR			255

//	����״̬
#define MOTOR_IDLE			0
#define MOTOR_PREP			3
#define MOTOR_ON			6
#define MOTOR_ERR			255

//	ʹ���е��λƽ��
#define	NVC_ENABLE

#define DC_POS_GAIN 0.01105957
#define DC_NEG_GAIN 0.01105957

#define RECT_U_VOLTAGE_OFFSET	2011.2f
//#define RECT_U_VOLTAGE_GAIN		0.247442789f
#define RECT_V_VOLTAGE_OFFSET	2018.0f
//#define RECT_V_VOLTAGE_GAIN		0.24653766f
#define RECT_W_VOLTAGE_OFFSET	2015.7f
//#define RECT_W_VOLTAGE_GAIN		0.246987544f

#define RECT_U_VOLTAGE_GAIN		0.3662109375f
#define RECT_V_VOLTAGE_GAIN		0.3662109375f
#define RECT_W_VOLTAGE_GAIN		0.3662109375f

// �ڶ���PCB�������ֵ�������������
#define RECT_U_CURRENT_OFFSET	2030.5
#define RECT_U_CURRENT_GAIN		0.019921875f
#define RECT_V_CURRENT_OFFSET	2016.9			//2026.4
#define RECT_V_CURRENT_GAIN		0.019921875f
#define RECT_W_CURRENT_OFFSET	2025.4
#define RECT_W_CURRENT_GAIN		0.019921875f

#define INV_U_CURRENT_OFFSET	2023.2
#define INV_U_CURRENT_GAIN		0.03984375f
#define INV_V_CURRENT_OFFSET	2029.2
#define INV_V_CURRENT_GAIN		0.03984375f
#define INV_W_CURRENT_OFFSET	2028.2
#define INV_W_CURRENT_GAIN		0.03984375f

// �������
#define MOTOR_INDUCTOR			12.7e-3
#define MOTOR_PHI				1.066052513
#define MOTOR_POLE				12

#endif /* GLOBAL_H_ */
