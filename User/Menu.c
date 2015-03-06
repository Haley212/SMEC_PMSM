/*
 * Menu.c
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 *      7-11	Ǩ�����汾1002�ϣ�Ŀǰֱ��������������ѹ����������ɣ��д����������໷
 *      7-30	Ǩ�����汾1003��ֱ��������������ΪLV25P��ʹ�����ã�������ѹ�����໷��ɡ���һ�����бջ����ԡ�
 *      		Ǩ�����汾1004���汾1003Ϊ�����汾��ֱ�����PWM�����������������β���
 *      8-18	�汾1004��
 *      9-1		����ӵ��裬���Ƶ�������ջ������Խ�����⡣���RPWM�ض�ʱ����PWM�ź���ȻΪ�ߵ�bug����Ӧ��FPGA����Ϊ1003.
 *      		�޲����߼������Է���ʹ�á�
 *   	9-3		�汾1005.�����ɹ���ָ�����3A����100VAC��350VDC�����߼��ǣ���LCL��磬�ٺ��ϼ̵���������
 *   			��һ�������Ⱥ��ϼ̵�������PWM���ķ�����������״̬ת�����ʺ���������
 *   	9-4		�汾1006.�Ⱥ��ϼ̵�������PWM��������û�����⣬100VAC��350VDC�¶�ֱ������û���⡣
 *   			����������220V��650V����������.����û�����⣬����Q����������Ǻܿɿأ�����ֵ����һֱΪ����
 *   			�����������ѹ������ѹ��������ɣ���һ��ʹ�ó��������Ԥ��硣
 *   			�˰汾��Ҫֱ����Դ����һ���ĳ�ʼ��ѹ��
 *   	9-9		�汾1007.����ʹ�ó���·����Ԥ��磬ȥ��ֱ����Դ��
 *   			���й����м��뽻����ѹ��ֵ����������ķ�ΧΪ��90% ~ 110%
 *   	9-28	��ֱ����ѹ�����D�����ָ���޷���Ϊ15A�������ϵ��PWM���������̣������������ʽ�����ѹ����ʵ��ʹ�õĵ�ѹ���ֿ���
 *   			ͬʱ��Menu�У�����PWM����û�г��ɹ���������ʾ��
 *   	10-8	�汾1008.���Լ����е�ƽ���㷨��
 *   			����NVC_ENABLE �궨��
 *   	10-20	�汾2008.����1008�������ڵڶ���PCB����������������.�µ�·�岻��ʹ��SMECInverter1xxx���򣡣���
 *   			����˳���ѹ����絽��92%�ٺ��ϼ̵�����
 *   			�޸��˼������ص�BUG���������������Ż���
 *   	10-30	�汾2009.����2008.�����䲿�ֳ���
 *   			�޸���F28335.cmd�ļ�����.ebss��RAML4���޸�Ϊ0x003000.���빦�ʼ���.
 *   			�������bug��
 *   	11-4	�汾2109.����2009.���Խ��2009�г��ֵ����⡣�˰汾Ϊ���԰汾��
 *   			����û�����⣬����˵���ͻȻ�������⡣
 *   	11-11	�汾3109.����2109.�����廻��ģ��+ɢ��Ƭʽ�����Գ�ʱ�����С��������������������������ɡ�
 *   			����������ӿڴ�����QUADECODER��⡣
 *   			Ӳ���ϰѽ�����ѹ�����Ĳ�����������ˣ����˰汾�⣬�����汾���������У�ע�⣬��ը���ĵġ�
 *   			ȥ�����໷bug��û�м��޷�����
 *   			SOGI_Update��SVM�������RAM�����У��Լ��ټ���ʱ�䡣
 *   			PID����Ҳ����RAM�С�
 *   	12-12	����������ֱ����ѹ���Ŵ�Ϊ+-30��֮ǰΪ+-20��������������Ŵ�
 *   			ȥ����֮ǰ�Ĵ��룬�ս��ڴˡ�
 *   	12-16	�汾3110.���ڰ汾3109.��������������ԡ�
 *   			�����˱�������Ϣ��ѯ����Encoder��
 *   	12-18	�������Ϊ12�Լ����105ת�ĵ��������������8192����������λ�ź�ΪF0..2��û��F3
 *   	1-12	�ٶȱջ����ԽϺã�������2015-1-12�գ�Ŀǰ�������У�����ʱ���ٶȲ�������ȷ��
 *   	1-12	�汾3112.
 *				��ӵ���ظ��������ܣ�x�رյ����$�����������������������رա�
 *				����ٴ�����ʱ��ת�Ӳ���Ҫ�ٴζ�λ��
 *		1-26	�汾3113.����ΪSMEC_PMSM��ʹ��git���߽��д�������ߡ�
 *				������������������������ӵķ�����
 */
#include "Global.h"
#include "F28335BSP.h"

volatile unsigned char MachineState;	//������״̬
volatile unsigned char MotorState;		//����״̬
unsigned char SCICmdBuffer[8];
unsigned char SCICmdBufferIndex = 0;

char *commands[] = {
		//observer
		"m",			//menu help				0
		"v",			//software version		1
		"b",			//bus voltage			2
		"g",			//grid condition		3
		"c",			//grid current			4
		"u",			//grid voltage			5
		"r",			//state					6
		"e",			//errors				7
		"d",			//motor current			8
		"f",			//fpga regs				9
		//action
		"+",			//main relay on			10
		"=",			//main realy off		11
		"_",			//charging relay on		12
		"-",			//charging realy off	13
		"!",			//On Grid				14
		"1",			//Off Grid				15
		"s",			//shutdown all pwm and relays
						//						16
		"$",			//Motor On				17
		"x",			//Motor Off				18
		"p",			//power calcu			19
		">",			//add speed				20
		"o",			//return encoder		21
		"k",			//open loop				22
		"<",			//sub speed				23
		"&"				//all on( ! and $ included )
};

MenuStruct Menu;

void man(void);						//�����˵�
void version(void);					//����汾
void bus_voltage(void);				//ֱ�����ѹ
void grid_condition(void);			//�������
void grid_current(void);			//�������
void grid_voltage(void);			//�����ѹ
void ShowMachineState(void);		//����״̬
void ShowErrors(void);				//����
void motor_current(void);			//�������
void fpga_error_reg(void);			//fpga����λ���
void main_relay_on(void);			//���Ӵ����պ�
void main_relay_off(void);			//���Ӵ����Ͽ�
void charging_relay_on(void);		//������պ�
void charging_relay_off(void);		//������Ͽ�
void shutdown(void);				//�ر�
void OnGridCMD(void);
void OffGridCMD(void);
void MotorONCMD(void);
void MotorOffCMD(void);
void PowerCalcu(void);
void AddSpeed(void);
void Encoder(void);
void OpenLoopCMD(void);
void SubSpeed(void);
void GridAndMotorCMD(void);
void cmd_forbidden(void);			//�����ֹ

void MainMenu(void){
	unsigned char i;
	for(i=0;i<MAX_CMD;i++){
		Menu.CMD[i].cmd = commands[i];
	}
	i=0;
	Menu.CMD[0].fp = man;
	Menu.CMD[1].fp = version;
	Menu.CMD[2].fp = bus_voltage;
	Menu.CMD[3].fp = grid_condition;
	Menu.CMD[4].fp = grid_current;
	Menu.CMD[5].fp = grid_voltage;
	Menu.CMD[6].fp = ShowMachineState;
	Menu.CMD[7].fp = ShowErrors;
	Menu.CMD[8].fp = motor_current;
	Menu.CMD[9].fp = fpga_error_reg;
	Menu.CMD[10].fp = main_relay_on;
	Menu.CMD[11].fp = main_relay_off;
	Menu.CMD[12].fp = charging_relay_on;
	Menu.CMD[13].fp = charging_relay_off;
	Menu.CMD[14].fp = OnGridCMD;
	Menu.CMD[15].fp = OffGridCMD;
	Menu.CMD[16].fp = shutdown;
	Menu.CMD[17].fp = MotorONCMD;
	Menu.CMD[18].fp = MotorOffCMD;
	Menu.CMD[19].fp = PowerCalcu;
	Menu.CMD[20].fp = AddSpeed;
	Menu.CMD[21].fp = Encoder;
	Menu.CMD[22].fp = OpenLoopCMD;
	Menu.CMD[23].fp = SubSpeed;
	Menu.CMD[24].fp = GridAndMotorCMD;

	while(1){
		while(SCICmdBufferIndex){
			if(SCICmdBufferIndex == 1){sPrintf("\r\n>");SCICmdBufferIndex = 0;}
			else{
				for(i=0;i<MAX_CMD;i++){
					if(SCICmdBuffer[0] == Menu.CMD[i].cmd[0]){
						Menu.CMD[i].fp();
						sPrintf("\r\n>");
						SCICmdBufferIndex = 0;
						break;
					}
				}
				if(SCICmdBufferIndex){
					sPrintf("\r\nCommand NOT found.");
					sPrintf("\r\n>");
					SCICmdBufferIndex = 0;
				}
			}
		}
	}
}

void man(void){
	sPrintf("\r\n**************************\r\n");
	sPrintf("m| menu help\r\n");
	sPrintf("v| software version\r\n");
	sPrintf("b| bus voltage\r\n");
	sPrintf("g| grid condition\r\n");
	sPrintf("c| grid current\r\n");
	sPrintf("u| grid voltage\r\n");
	sPrintf("r| machine state\r\n");
	sPrintf("e| Errors\r\n");
	sPrintf("d| motor current\r\n");
	sPrintf("f| fpga error reg\r\n");
	sPrintf("+| main relay on\r\n");
	sPrintf("=| main relay off\r\n");
	sPrintf("_| charging relay on\r\n");
	sPrintf("-| charging relay off\r\n");
	sPrintf("s| shutdown\r\n");
	sPrintf("!| On Grid\r\n");
	sPrintf("1| Off Grid\r\n");
	sPrintf("**************************");
}

void version(void){
	sPrintf("\r\n**************************\r\n");
	sPrintf("SMEC Inverter\r\n");
	sPrintf("DSP Version: %d\r\n",DSP_VERSION);
	sPrintf("FPGA Version: %d\r\n",FpgaRegs.FpgaSoftwareVersion);
	sPrintf("**************************");
}

void bus_voltage(void){
	sPrintf("\r\nDC BUS P:%dV N:%dV",(int)RectRunningData.POS_DC_Voltage,(int)RectRunningData.NEG_DC_Voltage);
}

void grid_condition(void){
	sPrintf("\r\nAC Voltage U:%d V:%d W:%d",(int)RectRunningData.U_Voltage,(int)RectRunningData.V_Voltage,(int)RectRunningData.W_Voltage);
	sPrintf("\r\nAC Current U:%d V:%d W:%d",RectSampleRawData.U_Current,RectSampleRawData.V_Current,RectSampleRawData.W_Current);
	sPrintf("\r\nGrid Frequency: %d.%d",(unsigned int)RectRunningData.GridFreq,((unsigned int)(RectRunningData.GridFreq * 100))%100);
	sPrintf("\r\nGrid PLL Q: %d.%d",(unsigned int)SOGIPLLData.QPI.MeasuredValue,((unsigned int)(SOGIPLLData.QPI.MeasuredValue * 100))%100);
}

void grid_current(void){
	sPrintf("\r\nAC Current");
	sPrintf("\r\nU: %d A",(int)(RectRunningData.U_Current*100));
	sPrintf("\r\nV: %d A",(int)(RectRunningData.V_Current*100));
	sPrintf("\r\nW: %d A",(int)(RectRunningData.W_Current*100));
	sPrintf("\r\nAC Current on DQ: %d.%d, %d.%d",(int)RectRunningData.CurrentD,((int)(RectRunningData.CurrentD * 100))%100,(int)RectRunningData.CurrentQ,((int)(RectRunningData.CurrentQ * 100))%100);
}

void grid_voltage(void){
	sPrintf("\r\nAC Voltage U:%d V:%d W:%d",(int)RectRunningData.U_Voltage,(int)RectRunningData.V_Voltage,(int)RectRunningData.W_Voltage);
	sPrintf("\r\nAC Voltage on DQ: %d, %d.",(int)RectRunningData.VoltageD,(int)RectRunningData.VoltageQ);
}

void ShowMachineState(void){
	sPrintf("\r\nMachine State: ");
	switch(MachineState){
		case STATE_IDLE:sPrintf("IDLE");break;
		case STATE_PREP:sPrintf("PREP");break;
		case STATE_ONGRID:sPrintf("ON Grid");break;
		case STATE_ERR:sPrintf("Error");break;
	}
	sPrintf("\r\nMotor State: ");
	switch(MotorState){
		case MOTOR_IDLE:sPrintf("IDLE");break;
		case MOTOR_ON:sPrintf("Runnning");break;
		case MOTOR_ERR:sPrintf("Error");break;
	}
}

void ShowErrors(void){
	sPrintf("\r\nRect Errors: %d",(unsigned int)RectRunningData.RectProtection.all);
	if(RectRunningData.RectProtection.bit.U_OVERCURRENT){sPrintf("\r\nU_OVERCURRENT: %d",(int)RectRunningData.RectErrMark.U_OVERCURRENT);}
	if(RectRunningData.RectProtection.bit.V_OVERCURRENT){sPrintf("\r\nV_OVERCURRENT: %d",(int)RectRunningData.RectErrMark.V_OVERCURRENT);}
	if(RectRunningData.RectProtection.bit.W_OVERCURRENT){sPrintf("\r\nW_OVERCURRENT: %d",(int)RectRunningData.RectErrMark.W_OVERCURRENT);}
	if(RectRunningData.RectProtection.bit.SUM_DC_ERR){sPrintf("\r\nSUM_DC_ERR: %d",(unsigned int)RectRunningData.RectErrMark.SUM_DC_ERR);}
	if(RectRunningData.RectProtection.bit.DEL_DC_ERR){sPrintf("\r\nDEL_DC_ERR: %d",(int)RectRunningData.RectErrMark.DEL_DC_ERR);}
	if(RectRunningData.RectProtection.bit.POS_DC_ERR){sPrintf("\r\nPOS_DC_ERR: %d",(unsigned int)RectRunningData.RectErrMark.POS_DC_ERR);}
	if(RectRunningData.RectProtection.bit.NEG_DC_ERR){sPrintf("\r\nNEG_DC_ERR: %d",(unsigned int)RectRunningData.RectErrMark.NEG_DC_ERR);}
	if(RectRunningData.RectProtection.bit.GRID_FREQ_ERR){sPrintf("\r\nGRID_FREQ_ERR: %d",(unsigned int)RectRunningData.RectErrMark.GRID_FREQ_ERR);}
	if(RectRunningData.RectProtection.bit.GRID_VOLTAGE_ERR){
		sPrintf("\r\nGRID_VOL_ERR: %d",(int)RectRunningData.RectErrMark.GRID_VOLTAGE_ERR);
		sPrintf("\r\nGRID_VOL_ERRQ: %d",(int)RectRunningData.RectErrMark.GRID_VOLTAGE_ERRQ);
		sPrintf("\r\nU: %d",(int)RectRunningData.RectErrMark.GRID_VOLTAGE_U);
		sPrintf("\r\nV: %d",(int)RectRunningData.RectErrMark.GRID_VOLTAGE_V);
		sPrintf("\r\nW: %d",(int)RectRunningData.RectErrMark.GRID_VOLTAGE_W);
		sPrintf("\r\ntheta: %d",(int)(RectRunningData.RectErrMark.GRID_ANGLE*57.29577951f));
	}
	if(RectRunningData.RectProtection.bit.FPGA_OVERHEAT){sPrintf("\r\nFPGA_OVERHEAT");}
	if(RectRunningData.RectProtection.bit.FPGA_OVERCURRENT){sPrintf("\r\nFPGA_OVERCURRENT");}
	sPrintf("\r\nMotor Errors: %d",(unsigned int)InvRunningData.InvProtection.all);
	if(InvRunningData.InvProtection.bit.U_OVERCURRENT){
		sPrintf("\r\nU_OC: %d",(int)InvRunningData.InvErrMark.U_OVERCURRENT);
		sPrintf("\r\nSpeed: %d, %d",(int)InvRunningData.InvErrMark.OVER_SPEED,(int)InvRunningData.InvErrMark.OVER_SPEED_PR);
		sPrintf("\r\nUDQ: %d, %d",(int)InvRunningData.InvErrMark.U_D,(int)InvRunningData.InvErrMark.U_Q);
	}
	if(InvRunningData.InvProtection.bit.V_OVERCURRENT){
		sPrintf("\r\nV_OC: %d",(int)InvRunningData.InvErrMark.V_OVERCURRENT);
		sPrintf("\r\nSpeed: %d, %d",(int)InvRunningData.InvErrMark.OVER_SPEED,(int)InvRunningData.InvErrMark.OVER_SPEED_PR);
		sPrintf("\r\nUDQ: %d, %d",(int)InvRunningData.InvErrMark.U_D,(int)InvRunningData.InvErrMark.U_Q);
	}
	if(InvRunningData.InvProtection.bit.W_OVERCURRENT){
		sPrintf("\r\nW_OC: %d",(int)InvRunningData.InvErrMark.W_OVERCURRENT);
		sPrintf("\r\nSpeed: %d, %d",(int)InvRunningData.InvErrMark.OVER_SPEED,(int)InvRunningData.InvErrMark.OVER_SPEED_PR);
		sPrintf("\r\nUDQ: %d, %d",(int)InvRunningData.InvErrMark.U_D,(int)InvRunningData.InvErrMark.U_Q);
	}
	if(InvRunningData.InvProtection.bit.OVER_SPEED){
		sPrintf("\r\nOS_FR: %d",(int)InvRunningData.InvErrMark.OVER_SPEED);
		sPrintf("\r\nOS_PR: %d",(int)InvRunningData.InvErrMark.OVER_SPEED_PR);
	}
}

void motor_current(void){
	sPrintf("\r\nMotor Current");
	sPrintf("\r\nU: %d A",(int)(InvRunningData.U_Current*100));
	sPrintf("\r\nV: %d A",(int)(InvRunningData.V_Current*100));
	sPrintf("\r\nW: %d A",(int)(InvRunningData.W_Current*100));
	sPrintf("\r\nMotor Current on DQ: %d.%d, %d.%d",(int)InvRunningData.CurrentD,((int)(InvRunningData.CurrentD * 100))%100,(int)InvRunningData.CurrentQ,((int)(InvRunningData.CurrentQ * 100))%100);
	sPrintf("\r\nDQ_Voltage: %d, %d", (int)InvRunningData.MotorVOutD,(int)InvRunningData.MotorVOutQ);
}

void fpga_error_reg(void){
	unsigned int i = FpgaRegs.IGBTTz;
	sPrintf("\r\nFPGA ERROR: %d",i);
	if(i & 0x01){sPrintf("\r\nInverter OverHeat.");}
	if(i & 0x02){sPrintf("\r\nRectifier OverHeat.");}
	if(i & 0x04){sPrintf("\r\nRectifier OverCurrent.");}
	if(i & 0x08){sPrintf("\r\nInverter OverCurrent.");}
}

void main_relay_on(void){
//	MR_ENABLE;
	sPrintf("\r\nMain Relay ON.");
}

void main_relay_off(void){
	MR_DISABLE;
	sPrintf("\r\nMain Relay OFF.");
}

void charging_relay_on(void){
	CR_ENABLE;
	sPrintf("\r\nCharging Relay ON.");
}

void charging_relay_off(void){
	CR_DISABLE;
	sPrintf("\r\nCharging Relay OFF.");
}

void OnGridCMD(void){
	unsigned int i = 0;
	sPrintf("\r\nWaiting...");
	i = 0;
	while(i < 100){	// Check PLL
		if(RectRunningData.GridFreq > 47.0f && RectRunningData.GridFreq < 53.0f){i ++;}
		else{i = 0;}
		DelayMs(1);
	}

	DelayMs(1000);
	sPrintf("\r\nResist Charging...");
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_RESIS_CHARGE;		//����Ԥ���״̬
	i = 0;
	while(i < 10){							//������⵽100��ֱ����ѹ����ѹ�㹻�ߺ󣬺ϼ̵���
		if(RectRunningData.DC_Voltage > RectRunningData.VoltageD * 1.73 * 0.92){i ++;}
		else{i = 0;}
		DelayMs(1);
	}

	ProcessRectifierControlInit();
	sPrintf("\r\nInitilized...");

	DelayMs(10);
	sPrintf("\r\nFPGA PWM Reset Done...");
	FPGA_PWM_UPDATE_LOCK;					//��֤�͵�ƽ
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_PREP;				//LCL charging
	sPrintf("\r\nFPGA PREP��");
	DelayMs(300);
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_DC_UNSTABLE;
	sPrintf("\r\nON Grid.DC Stablizing...");
	i = 0;
	while(i < 100){							//������⵽100��ֱ����ѹ�ȶ�������Ϊ��ѹ�ȶ���700V
		if(RectRunningData.DC_Voltage < 720.0f && RectRunningData.DC_Voltage > 680.0){i ++;}
		else{i = 0;}
		if(RectRunningData.RectProtection.all){
			sPrintf("\r\nDC Voltage Error!");
			return;
		}
		DelayMs(1);
	}
	sPrintf("\r\nDC Voltage Stable...");
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_ONGRID;
}

void OffGridCMD(void){
	RPWM_DISABLE;
	IPWM_DISABLE;
	MR_DISABLE;
	CR_DISABLE;
	MachineState = STATE_IDLE;
	MotorState = MOTOR_IDLE;
	sPrintf("\r\nOffGrid.");
}

void MotorONCMD(void){
	InvControlInit();
	EncoderInit(0);
	DelayMs(20);
	if(MachineState != STATE_ONGRID){sPrintf("\r\nRect State Error.");return;}
	if(MotorState == MOTOR_ERR){sPrintf("\r\nMotor Error.");return;}
	MotorState = MOTOR_PREP;
	while(InvRunningData.MotorPWMPREP < 4);
	if(MotorState == MOTOR_ERR){sPrintf("\r\nMotor Error.");return;}
	MotorState = MOTOR_ON;
	sPrintf("\r\nMotor ON!");
}

void MotorOffCMD(void){
	MotorState = MOTOR_IDLE;
	sPrintf("\r\nMotor OFF!");
	IPWM_DISABLE;
}

void shutdown(void){
	//FPGA_PWM_SHUTDOWN;
	RPWM_DISABLE;
	IPWM_DISABLE;
	MR_DISABLE;
	CR_DISABLE;
	MachineState = STATE_IDLE;
	MotorState = MOTOR_IDLE;
	sPrintf("\r\nShutDown.");
}

void PowerCalcu(void){
	float RectP = 	RectRunningData.U_Voltage * RectRunningData.U_Current +
					RectRunningData.V_Voltage * RectRunningData.V_Current +
					RectRunningData.W_Voltage * RectRunningData.W_Current;
	float RectQ =	((RectRunningData.W_Voltage - RectRunningData.V_Voltage) * RectRunningData.U_Current +
					(RectRunningData.U_Voltage - RectRunningData.W_Voltage) * RectRunningData.V_Current +
					(RectRunningData.V_Voltage - RectRunningData.U_Voltage) * RectRunningData.W_Current) * 0.57735f;
	float InvP = 	InvRunningData.U_Voltage * InvRunningData.U_Current +
					InvRunningData.V_Voltage * InvRunningData.V_Current +
					InvRunningData.W_Voltage * InvRunningData.W_Current;
	float InvQ =	((InvRunningData.W_Voltage - InvRunningData.V_Voltage) * InvRunningData.U_Current +
					(InvRunningData.U_Voltage - InvRunningData.W_Voltage) * InvRunningData.V_Current +
					(InvRunningData.V_Voltage - InvRunningData.U_Voltage) * InvRunningData.W_Current) * 0.57735f;
	sPrintf("\r\nRect PQ: %d, %d (VA)", (int)RectP, (int)RectQ);
	sPrintf("\r\nInvt PQ: %d, %d (VA)", (int)InvP, (int)InvQ);
}

void AddSpeed(void){
	GivenSpeed += 10.0f;
	sPrintf("\r\nGiven Spd: %drpm",(int)GivenSpeed);
}

void SubSpeed(void){
	GivenSpeed -= 10.0f;
	sPrintf("\r\nGiven Spd: %drpm",(int)GivenSpeed);
}

void Encoder(void){
	sPrintf("\r\nRPM: %d, %d",(int)(MotorSpeedData.SpeedRpm_fr * 10),(int)(MotorSpeedData.SpeedRpm_pr * 10));
	sPrintf("\r\nC: %d",(unsigned int)EQep1Regs.QPOSCNT);
	sPrintf("\r\nF: %d",(unsigned int)(MotorSpeedData.F[0] * 100 + MotorSpeedData.F[1] * 10 + MotorSpeedData.F[2]));
	sPrintf("\r\nF_Count: %d",(unsigned int)MotorSpeedData.F_Count);
	sPrintf("\r\nF Index: %d",MotorSpeedData.F_Index);
	sPrintf("\r\ntheta_elec: %d",(unsigned int)(MotorSpeedData.theta_elec * 57.2957f));
	sPrintf("\r\ntheta_elec: %d",(unsigned int)(MotorSpeedData.F_theta * 57.2957f));
	sPrintf("\r\nCDEF: %d",EQep1Regs.QEPSTS.bit.CDEF);
}

void OpenLoopCMD(void){
/*
	if(OpenLoopBoolean){OpenLoopBoolean = 0;}
	else if(OpenLoopBoolean == 0){OpenLoopBoolean = 1;}
	if(OpenLoopBoolean == 1){
		sPrintf("\r\nChange to Open Loop.");
	}else if(OpenLoopBoolean == 0){
		sPrintf("\r\nChange to Closed Loop.");
	}
*/
}

void GridAndMotorCMD(void){
	sPrintf("\r\nGrid And Motor will start NOW!");
	DelayMs(1000);
	OnGridCMD();
	DelayMs(1000);
	MotorONCMD();
}

void cmd_forbidden(void){
	sPrintf("\r\bThis Command is NOT supported while DEBUG.");
}
