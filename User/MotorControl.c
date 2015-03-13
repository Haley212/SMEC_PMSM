/*
 * InvControl.c
 *
 *  Created on: 2014-10-27
 *      Author: appleseed
 */
#include "MotorControl.h"
#include "DSP28x_Project.h"

volatile float GivenSpeed;
volatile unsigned int OpenLoopBoolean;
volatile float SpeedRef;

volatile struct InvSampleRawDataStruct InvSampleRawData;
volatile struct InvRunningDataStruct InvRunningData;
volatile struct MotorSpeed MotorSpeedData;

volatile float QC;
DQ MotorCurrentDQ;
DQ MotorVOutDQ;
sPIDParams MotorCurrentDPI;
sPIDParams MotorCurrentQPI;
sPIDParams MotorSpeedPI;

#define SpeedArrayMax 4
float SpeedArray[SpeedArrayMax];
unsigned int SpeedArrayIndex;
float SpeedAverage;

void InvControlInit(void){
	SpeedRef = 0.0f;
	OpenLoopBoolean = 0;		//默认闭环

	SpeedArrayIndex = 0;

	InvRunningData.MotorAngle = 0.0f;
	InvRunningData.CurrentDPI = 0;
	InvRunningData.CurrentQPI = 0;
	InvRunningData.TorqueQ = 0;
	InvRunningData.ReactorCompensate = 1.5 * MOTOR_INDUCTOR;
	InvRunningData.InvProtection.all = 0;
	InvRunningData.MotorUOCCounter = 0;
	InvRunningData.MotorVOCCounter = 0;
	InvRunningData.MotorWOCCounter = 0;
	InvRunningData.MotorPWMPREP = 0;

	DQInit(&MotorCurrentDQ);
	DQInit(&MotorVOutDQ);
	IncPIDControlInit(&MotorCurrentDPI);
	IncPIDControlInit(&MotorCurrentQPI);
	IncPIDControlInit(&MotorSpeedPI);
/*	MotorSpeedPI.ProportionalGain = 0.2f;
	MotorSpeedPI.IntergralGain = 0.01f;
	MotorCurrentDPI.ProportionalGain = 5.0f;
	MotorCurrentDPI.IntergralGain = 0.05f;
	MotorCurrentQPI.ProportionalGain = 5.0f;
	MotorCurrentQPI.IntergralGain = 0.05f;
*/
	MotorSpeedPI.ProportionalGain = 0.2f;
	MotorSpeedPI.IntergralGain = 0.01f;
	MotorCurrentDPI.ProportionalGain = 2.0f;
	MotorCurrentDPI.IntergralGain = 0.05f;
	MotorCurrentQPI.ProportionalGain = 2.0f;
	MotorCurrentQPI.IntergralGain = 0.05f;

	QC = 0.0f;
}

void ProcessInvSampleDataRoutine(void){

	InvRunningData.POS_DC_Voltage = RectRunningData.POS_DC_Voltage;
	InvRunningData.NEG_DC_Voltage = RectRunningData.NEG_DC_Voltage;
	InvRunningData.DC_Voltage = RectRunningData.DC_Voltage;
	InvRunningData.DC_Delta = RectRunningData.DC_Delta;
/*
	InvRunningData.U_Current = ((float)InvSampleRawData.U_Current - (float)INV_U_CURRENT_OFFSET) * INV_U_CURRENT_GAIN;
	InvRunningData.V_Current = ((float)InvSampleRawData.V_Current - (float)INV_V_CURRENT_OFFSET) * INV_V_CURRENT_GAIN;
	InvRunningData.W_Current = ((float)InvSampleRawData.W_Current - (float)INV_W_CURRENT_OFFSET) * INV_W_CURRENT_GAIN;
*/
	InvRunningData.U_Current = ((float)INV_U_CURRENT_OFFSET - (float)InvSampleRawData.U_Current) * INV_U_CURRENT_GAIN;
	InvRunningData.V_Current = ((float)INV_V_CURRENT_OFFSET - (float)InvSampleRawData.V_Current) * INV_V_CURRENT_GAIN;
	InvRunningData.W_Current = ((float)INV_W_CURRENT_OFFSET - (float)InvSampleRawData.W_Current) * INV_W_CURRENT_GAIN;

	//E theta acquire
//	InvRunningData.MotorAngle += 0.03141592654;		//50Hz*2pi/10KHz = 1.8degree Per Interrupt
//	if(InvRunningData.MotorAngle >= 6.283185307f){ InvRunningData.MotorAngle -= 6.283185307f;}
	InvRunningData.MotorAngle = MotorSpeedData.theta_elec;

	MotorCurrentDQ.sine_value = sin(InvRunningData.MotorAngle);
	MotorCurrentDQ.cosine_value = cos(InvRunningData.MotorAngle);
	MotorCurrentDQ.U = InvRunningData.U_Current;
	MotorCurrentDQ.V = InvRunningData.V_Current;
	MotorCurrentDQ.W = InvRunningData.W_Current;

	ABC2DQ(&MotorCurrentDQ);
	MotorVOutDQ.sine_value = MotorCurrentDQ.sine_value;
	MotorVOutDQ.cosine_value = MotorCurrentDQ.cosine_value;
	InvRunningData.CurrentD = MotorCurrentDQ.D;
	InvRunningData.CurrentQ = MotorCurrentDQ.Q;
}

void ProcessInverterControl(void){

	//	D轴电流控制
	MotorCurrentDPI.MeasuredValue = MotorCurrentDQ.D;
	MotorCurrentDPI.DesiredValue = 0.0f;
	InvRunningData.CurrentDPI += IncPIControl(&MotorCurrentDPI);
	if(InvRunningData.CurrentDPI > MAX_MOTOR_CURRENTDPI){InvRunningData.CurrentDPI = MAX_MOTOR_CURRENTDPI;}
	if(InvRunningData.CurrentDPI < MIN_MOTOR_CURRENTDPI){InvRunningData.CurrentDPI = MIN_MOTOR_CURRENTDPI;}
/*	if(QC < 30.0f){
		QC += 3e-4;
	}else{
		QC = 30.0f;
	}
	MotorVOutDQ.D = - MotorSpeedData.omega * InvRunningData.ReactorCompensate * QC;*/
	MotorVOutDQ.D = InvRunningData.CurrentDPI - MotorSpeedData.omega * InvRunningData.ReactorCompensate * MotorCurrentDQ.Q;
	//	Q轴电流控制
	MotorCurrentQPI.MeasuredValue = MotorCurrentDQ.Q;
	if(OpenLoopBoolean){
		MotorCurrentQPI.DesiredValue = 5.0f;
	}else{
		MotorCurrentQPI.DesiredValue = InvRunningData.TorqueQ;
	}
	InvRunningData.CurrentQPI += IncPIControl(&MotorCurrentQPI);
	if(InvRunningData.CurrentQPI > MAX_MOTOR_CURRENTQPI){InvRunningData.CurrentQPI = MAX_MOTOR_CURRENTQPI;}
	if(InvRunningData.CurrentQPI < MIN_MOTOR_CURRENTQPI){InvRunningData.CurrentQPI = MIN_MOTOR_CURRENTQPI;}
//	MotorVOutDQ.Q = QC * 0.359 + MotorSpeedData.omega * MOTOR_PHI;
	MotorVOutDQ.Q = InvRunningData.CurrentQPI + MotorSpeedData.omega * InvRunningData.ReactorCompensate * MotorCurrentDQ.D + MotorSpeedData.omega * MOTOR_PHI;

	InvRunningData.MotorVOutD = MotorVOutDQ.D;
	InvRunningData.MotorVOutQ = MotorVOutDQ.Q;

	DQ2ABC(&MotorVOutDQ);
	InvRunningData.U_Voltage = MotorVOutDQ.U;
	InvRunningData.V_Voltage = MotorVOutDQ.V;
	InvRunningData.W_Voltage = MotorVOutDQ.W;

	SVMArguments[DC_VOLTAGE] = InvRunningData.DC_Voltage * 0.5f;
	SVMArguments[VOLTAGE_SHIFT] = 0;
#ifdef	NVC_ENABLE
	SVMArguments[VOLTAGE_SHIFT] = -RectRunningData.DC_Delta * 0.1f;
#endif

	SVMArguments[CURRENT_A] = InvRunningData.U_Current;
	SVMArguments[CURRENT_B] = InvRunningData.V_Current;
	SVMArguments[CURRENT_C] = InvRunningData.W_Current;
//	SVMArguments[VOLTAGE_OUT] = 310.0f;
//	SVMArguments[VOLTAGE_A] =  SVMArguments[VOLTAGE_OUT] * cos(InvRunningData.MotorAngle);
//	SVMArguments[VOLTAGE_B] =  SVMArguments[VOLTAGE_OUT] * cos(InvRunningData.MotorAngle + 4.188790205f);
//	SVMArguments[VOLTAGE_C] =  SVMArguments[VOLTAGE_OUT] * cos(InvRunningData.MotorAngle + 2.094395102f);

	SVMArguments[VOLTAGE_A] =  MotorVOutDQ.U;
	SVMArguments[VOLTAGE_B] =  MotorVOutDQ.V;
	SVMArguments[VOLTAGE_C] =  MotorVOutDQ.W;
	SVPWM(SVMArguments,INV_PWM);
}

void ProcessMotorSpeedLoop(void){
	float err;
#define ERR_SLOPE 1.0f
	if(MotorSpeedData.speed_sync_flag == 1){
		err = GivenSpeed - SpeedRef;
		if(err > ERR_SLOPE){
			SpeedRef += ERR_SLOPE;
		}else if(err < -ERR_SLOPE){
			SpeedRef -= ERR_SLOPE;
		}else{
			SpeedRef = GivenSpeed;
		}
		MotorSpeedPI.DesiredValue = SpeedRef;

//		GivenSpeed += 0.2f;
//		if(GivenSpeed > 140.0f){GivenSpeed = 140.0f;}
//		MotorSpeedPI.DesiredValue = GivenSpeed;

/*		if(MotorSpeedData.SpeedRpm_pr > 30.0f || MotorSpeedData.SpeedRpm_pr < -30.0f){
			MotorSpeedPI.MeasuredValue = MotorSpeedData.SpeedRpm_fr;
//			MotorSpeedData.omega = MotorSpeedData.SpeedRpm_fr * 1.256637061f;	// 12 * 2pi / 60
			MotorSpeedData.omega = 0.0f;
		}else{
			MotorSpeedPI.MeasuredValue = MotorSpeedData.SpeedRpm_pr;
//			MotorSpeedData.omega = MotorSpeedData.SpeedRpm_pr * 1.256637061f;	// 12 * 2pi / 60
			MotorSpeedData.omega = 0.0f;
		}*/

		MotorSpeedPI.MeasuredValue = SpeedAverage;
		MotorSpeedData.omega = SpeedAverage * 1.256637061f;	// 12 * 2pi / 60

//		MotorSpeedPI.MeasuredValue = MotorSpeedData.SpeedRpm_fr;
//		MotorSpeedData.omega = MotorSpeedData.SpeedRpm_fr * 1.256637061f;	// 12 * 2pi / 60
//		MotorSpeedData.omega = 0.0f;
		InvRunningData.TorqueQ += IncPIControl(&MotorSpeedPI);
		if(InvRunningData.TorqueQ > MAX_TORQUE_CURRENT){InvRunningData.TorqueQ = MAX_TORQUE_CURRENT;}
		if(InvRunningData.TorqueQ < MIN_TORQUE_CURRENT){InvRunningData.TorqueQ = MIN_TORQUE_CURRENT;}
		MotorSpeedData.speed_sync_flag = 0;
		// End of Speed PI
	}else{
	}
}

unsigned int InvProtectionRoutine(void){	//软件保护，返回1表示有问题
	float t;
#define INV_OVER_CURRENT	50
#define MOTOR_OVERSPEED		110

	if(InvRunningData.U_Current > INV_OVER_CURRENT || InvRunningData.U_Current < -INV_OVER_CURRENT){
		t = -InvRunningData.V_Current - InvRunningData.W_Current;
		if(t > INV_OVER_CURRENT || t < -INV_OVER_CURRENT){
			InvRunningData.MotorUOCCounter ++;
		}else{
			InvRunningData.U_Current = t;
			InvRunningData.MotorUOCCounter = 0;
		}
	}else{
		InvRunningData.MotorUOCCounter = 0;
	}
	if(InvRunningData.V_Current > INV_OVER_CURRENT || InvRunningData.V_Current < -INV_OVER_CURRENT){
		t = -InvRunningData.U_Current - InvRunningData.W_Current;
		if(t > INV_OVER_CURRENT || t < -INV_OVER_CURRENT){
			InvRunningData.MotorVOCCounter ++;
		}else{
			InvRunningData.V_Current = t;
			InvRunningData.MotorVOCCounter = 0;
		}
	}else{
		InvRunningData.MotorVOCCounter = 0;
	}
	if(InvRunningData.W_Current > INV_OVER_CURRENT || InvRunningData.W_Current < -INV_OVER_CURRENT){
		t = -InvRunningData.U_Current - InvRunningData.V_Current;
		if(t > INV_OVER_CURRENT || t < -INV_OVER_CURRENT){
			InvRunningData.MotorWOCCounter ++;
		}else{
			InvRunningData.W_Current = t;
			InvRunningData.MotorWOCCounter = 0;
		}
	}else{
		InvRunningData.MotorWOCCounter = 0;
	}

	if(InvRunningData.MotorUOCCounter > 2){
		InvRunningData.InvProtection.bit.U_OVERCURRENT = 1;
		InvRunningData.InvErrMark.U_OVERCURRENT = InvRunningData.U_Current;
		InvRunningData.InvErrMark.OVER_SPEED = MotorSpeedData.SpeedRpm_fr;
		InvRunningData.InvErrMark.OVER_SPEED_PR = MotorSpeedData.SpeedRpm_pr;
		InvRunningData.InvErrMark.U_D = MotorVOutDQ.D;
		InvRunningData.InvErrMark.U_Q = MotorVOutDQ.Q;
		return 1;
	}
	if(InvRunningData.MotorVOCCounter > 2){
		InvRunningData.InvProtection.bit.V_OVERCURRENT = 1;
		InvRunningData.InvErrMark.V_OVERCURRENT = InvRunningData.V_Current;
		InvRunningData.InvErrMark.OVER_SPEED = MotorSpeedData.SpeedRpm_fr;
		InvRunningData.InvErrMark.OVER_SPEED_PR = MotorSpeedData.SpeedRpm_pr;
		InvRunningData.InvErrMark.U_D = MotorVOutDQ.D;
		InvRunningData.InvErrMark.U_Q = MotorVOutDQ.Q;
		return 1;
	}
	if(InvRunningData.MotorWOCCounter > 2){
		InvRunningData.InvProtection.bit.W_OVERCURRENT = 1;
		InvRunningData.InvErrMark.W_OVERCURRENT = InvRunningData.W_Current;
		InvRunningData.InvErrMark.OVER_SPEED = MotorSpeedData.SpeedRpm_fr;
		InvRunningData.InvErrMark.OVER_SPEED_PR = MotorSpeedData.SpeedRpm_pr;
		InvRunningData.InvErrMark.U_D = MotorVOutDQ.D;
		InvRunningData.InvErrMark.U_Q = MotorVOutDQ.Q;
		return 1;
	}
/*	if(MotorSpeedPI.MeasuredValue > MOTOR_OVERSPEED || MotorSpeedPI.MeasuredValue < -MOTOR_OVERSPEED){
		InvRunningData.InvProtection.bit.OVER_SPEED = 1;
		InvRunningData.InvErrMark.OVER_SPEED = MotorSpeedData.SpeedRpm_fr;
		InvRunningData.InvErrMark.OVER_SPEED_PR = MotorSpeedData.SpeedRpm_pr;
		return 1;
	}*/
	return 0;
}

void EncoderInit(int init_rotor_position){
	MotorSpeedData.cal_angle = 1.902408885f;						// 电角度偏移
	MotorSpeedData.mech_scaler = 6.2831853071796 / (8192.0f * 4);	// 编码器线数，转一圈获得的脉冲数
	MotorSpeedData.pole_pairs = 12;									// 极对数
	MotorSpeedData.elec_scaler = MotorSpeedData.mech_scaler * (float)(MotorSpeedData.pole_pairs);
	MotorSpeedData.index_sync_flag = 0;
//	MotorSpeedData.old_position = 0;
//	MotorSpeedData.new_position = 0;
	MotorSpeedData.old_position_uint = 0;
	MotorSpeedData.new_position_uint = 0;
	MotorSpeedData.BaseRpm = 954.9296586f;							// 60*100/2pi
	MotorSpeedData.SpeedScaler = 68664.55078f;						// 32 / 4 * 150M * 60 / 8192 / 128
	MotorSpeedData.SpeedRpm_fr = 0.0f;
	MotorSpeedData.SpeedRpm_pr = 0.0f;
	MotorSpeedData.theta_elec = 0;
	MotorSpeedData.speed_sync_flag = 0;
	MotorSpeedData.F_Count = 0;

	if(init_rotor_position){
		MotorSpeedData.F[0] = GpioDataRegs.GPBDAT.bit.GPIO52;
		MotorSpeedData.F[1] = GpioDataRegs.GPBDAT.bit.GPIO49;
		MotorSpeedData.F[2] = GpioDataRegs.GPBDAT.bit.GPIO48;
		MotorSpeedData.F[3] = GpioDataRegs.GPBDAT.bit.GPIO58;		//这一位永远是1

		MotorSpeedData.F_Index = ((MotorSpeedData.F[0] << 8) + (MotorSpeedData.F[1] << 4) + MotorSpeedData.F[2]) & 0x0111;

		switch(MotorSpeedData.F_Index){
			case 0x000:MotorSpeedData.F_theta = 0.392699081f;break;		//	22.5
			case 0x001:MotorSpeedData.F_theta = 1.178097245f;break;		//	67.5
			case 0x011:MotorSpeedData.F_theta = 1.963495409f;break;		//	112.5
			case 0x010:MotorSpeedData.F_theta = 2.748893572f;break;		//	157.5
			case 0x110:MotorSpeedData.F_theta = 3.534291735f;break;		//	202.5
			case 0x111:MotorSpeedData.F_theta = 4.319689899f;break;		//	247.5
			case 0x101:MotorSpeedData.F_theta = 5.105088062f;break;		//	292.5
			case 0x100:MotorSpeedData.F_theta = 5.890486226f;break;		//	337.5
		}
		EQep1Regs.QPOSCNT = MotorSpeedData.F_theta * 434.5991f;
	}else{
		//	不检测转子初始位置
	}
}

void ProcessEncoder(void){
	unsigned long pos;
	unsigned int tmr;
	signed long delta[2];
	MotorSpeedData.DirectionQeq = EQep1Regs.QEPSTS.bit.QDF;

	/*Motor RAW Position*/
	pos = (unsigned long)EQep1Regs.QPOSCNT;
	MotorSpeedData.theta_raw = (float)pos;

	/*mech*/
	MotorSpeedData.theta_mech = MotorSpeedData.theta_raw * MotorSpeedData.mech_scaler;

	/*elect*/
	MotorSpeedData.theta_elec = MotorSpeedData.theta_raw * MotorSpeedData.elec_scaler + MotorSpeedData.cal_angle;

	// Check index occurrence
	if(EQep1Regs.QFLG.bit.IEL == 1){
		MotorSpeedData.index_sync_flag = 1;
		EQep1Regs.QCLR.bit.IEL = 1;
	}

	// High Speed Calculation using QEP Position counter
	if(EQep1Regs.QFLG.bit.UTO == 1){		// unit timer 100hz
		pos = (unsigned long)EQep1Regs.QPOSLAT;
		MotorSpeedData.new_position_uint = (signed long)pos;

		if(MotorSpeedData.new_position_uint > MotorSpeedData.old_position_uint){
			delta[0] = MotorSpeedData.new_position_uint - MotorSpeedData.old_position_uint;			// 正转
			delta[1] = MotorSpeedData.old_position_uint + 32768 - MotorSpeedData.new_position_uint;	// 反转
		}else{
			delta[0] = MotorSpeedData.new_position_uint + 32768 - MotorSpeedData.old_position_uint;	// 正转
			delta[1] = MotorSpeedData.old_position_uint - MotorSpeedData.new_position_uint;			// 反转
		}
		if(delta[0] < 2184 && delta[0] >= 0){			// 1638 -> 300rpm, 2184 -> 400rpm
			MotorSpeedData.del_position_uint = delta[0];
		}else if(delta[1] < 2184 && delta[1] >= 0){
			MotorSpeedData.del_position_uint = -(delta[1]);
		}else{
		}
		MotorSpeedData.old_position_uint = MotorSpeedData.new_position_uint;
		MotorSpeedData.SpeedRpm_fr = ((float)MotorSpeedData.del_position_uint) * 0.183105468f;

		SpeedArray[SpeedArrayIndex] = MotorSpeedData.SpeedRpm_fr;
		SpeedArrayIndex ++;
		SpeedArrayIndex = SpeedArrayIndex % SpeedArrayMax;
		SpeedAverage = (SpeedArray[0] + SpeedArray[1] + SpeedArray[2] + SpeedArray[3]) * 0.25f;

		/*
		MotorSpeedData.DirectionQeq = EQep1Regs.QEPSTS.bit.QDF;
		if(MotorSpeedData.DirectionQeq == 0){		//POSCNT counting down
			if(MotorSpeedData.new_position_uint > MotorSpeedData.old_position_uint){
				MotorSpeedData.del_position_uint = -((32768 + MotorSpeedData.old_position_uint) - MotorSpeedData.new_position_uint);
			}else{
				MotorSpeedData.del_position_uint = MotorSpeedData.new_position_uint - MotorSpeedData.old_position_uint;
			}
		}else if(MotorSpeedData.DirectionQeq == 1){	//POSCNT counting up
			if(MotorSpeedData.new_position_uint < MotorSpeedData.old_position_uint){
				MotorSpeedData.del_position_uint = (32768 + MotorSpeedData.new_position_uint) - MotorSpeedData.old_position_uint;
			}else{
				MotorSpeedData.del_position_uint = MotorSpeedData.new_position_uint - MotorSpeedData.old_position_uint;
			}
		}
		MotorSpeedData.old_position_uint = MotorSpeedData.new_position_uint;
		MotorSpeedData.SpeedRpm_fr = ((float)MotorSpeedData.del_position_uint) * 0.183105468f;
*/
		/* Orignal Speed Calculation with hardware-float
		MotorSpeedData.new_position = (float)pos * MotorSpeedData.mech_scaler;
		if(MotorSpeedData.DirectionQeq == 0){		//POSCNT counting down
			if(MotorSpeedData.new_position > MotorSpeedData.old_position){
				MotorSpeedData.del_position = -(6.2831853071796 + MotorSpeedData.old_position - MotorSpeedData.new_position);
			}else{
				MotorSpeedData.del_position = MotorSpeedData.new_position - MotorSpeedData.old_position;
			}
		}else if(MotorSpeedData.DirectionQeq == 1){	//POSCNT counting up
			if(MotorSpeedData.new_position < MotorSpeedData.old_position){
				MotorSpeedData.del_position = 6.2831853071796 + MotorSpeedData.new_position - MotorSpeedData.old_position;
			}else{
				MotorSpeedData.del_position = MotorSpeedData.new_position - MotorSpeedData.old_position;
			}
		}
		MotorSpeedData.old_position = MotorSpeedData.new_position;
		MotorSpeedData.SpeedRpm_fr = MotorSpeedData.del_position * MotorSpeedData.BaseRpm;
		*/
//		if(MotorSpeedData.SpeedRpm_fr > 100.0f){MotorSpeedData.SpeedRpm_fr = 100.0f;}
//		if(MotorSpeedData.SpeedRpm_fr < -100.0f){MotorSpeedData.SpeedRpm_fr = -100.0f;}
		MotorSpeedData.speed_sync_flag = 1;		// 10ms到了，速度环PI在下一个周期10k周期进行计算
		EQep1Regs.QCLR.bit.UTO = 1;				// clear
	}
/*
	// Low Speed Calculation using QEP capture counter	测试低速，23rpm
	if(EQep1Regs.QEPSTS.bit.UPEVNT == 1){
		if(EQep1Regs.QEPSTS.bit.COEF == 0){		// No capture overflow
			tmr = (unsigned int)EQep1Regs.QCPRDLAT;
		}else{
			tmr = 0xFFFF;
		}
		// rpm = (32/8192/4)/(tmr / (150M / 128)) * 60
		MotorSpeedData.SpeedRpm_pr = MotorSpeedData.SpeedScaler / (float)tmr;
		if(MotorSpeedData.DirectionQeq == 0){
			MotorSpeedData.SpeedRpm_pr = - MotorSpeedData.SpeedRpm_pr;
		}
		EQep1Regs.QEPSTS.all = 0x88;			// clear
	}

	if(EQep1Regs.QEPSTS.bit.CDEF == 1){
		EQep1Regs.QEPSTS.bit.CDEF = 1;
	}
*/
	/*
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;	//GPIO52 -> F0
	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;	//GPIO49 -> F1
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;	//GPIO48 -> F2
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;	//GPIO58 -> F3
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;		//Input*/
	MotorSpeedData.F[0] = GpioDataRegs.GPBDAT.bit.GPIO52;
	MotorSpeedData.F[1] = GpioDataRegs.GPBDAT.bit.GPIO49;
	MotorSpeedData.F[2] = GpioDataRegs.GPBDAT.bit.GPIO48;
	MotorSpeedData.F[3] = GpioDataRegs.GPBDAT.bit.GPIO58;		//这一位永远是1

	MotorSpeedData.F_Index = ((MotorSpeedData.F[0] << 8) + (MotorSpeedData.F[1] << 4) + MotorSpeedData.F[2]) & 0x0111;

	switch(MotorSpeedData.F_Index){
		case 0x000:MotorSpeedData.F_theta = 0.392699081f;break;		//	22.5
		case 0x001:MotorSpeedData.F_theta = 1.178097245f;break;		//	67.5
		case 0x011:MotorSpeedData.F_theta = 1.963495409f;break;		//	112.5
		case 0x010:MotorSpeedData.F_theta = 2.748893572f;break;		//	157.5
		case 0x110:MotorSpeedData.F_theta = 3.534291735f;break;		//	202.5
		case 0x111:MotorSpeedData.F_theta = 4.319689899f;break;		//	247.5
		case 0x101:MotorSpeedData.F_theta = 5.105088062f;break;		//	292.5
		case 0x100:MotorSpeedData.F_theta = 5.890486226f;break;		//	337.5
	}
/*
	if(MotorSpeedData.F_Index == 1){								//	检测F信号有多少个
		if(MotorSpeedData.F_Index_last == 0){
			MotorSpeedData.F_Count ++;
		}
	}
	MotorSpeedData.F_Index_last = MotorSpeedData.F_Index;

	if((unsigned int)EQep1Regs.QPOSCNT < 100){
		MotorSpeedData.F_Count = 0;
	}
*/
}
