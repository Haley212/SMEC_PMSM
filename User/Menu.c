/*
 * Menu.c
 *
 *  Created on: 2014-7-1
 *      Author: appleseed
 *      7-11	迁移至版本1002上，目前直流采样、交流电压采样调试完成，有待检测的是锁相环
 *      7-30	迁移至版本1003，直流采样更换方案为LV25P，使用良好，交流电压，锁相环完成。下一步进行闭环测试。
 *      		迁移至版本1004，版本1003为开环版本，直接输出PWM波，可用于驱动波形测试
 *      8-18	版本1004，
 *      9-1		输出接电阻，控制电阻电流闭环，测试结果满意。解决RPWM关断时部分PWM信号依然为高的bug。对应的FPGA程序为1003.
 *      		无并网逻辑，可以放心使用。
 *   	9-3		版本1005.并网成功，指令电流3A。（100VAC，350VDC），逻辑是：先LCL充电，再合上继电器并网。
 *   			下一步进行先合上继电器，后发PWM波的方法。（这种状态转换更适合整流器）
 *   	9-4		版本1006.先合上继电器，后发PWM波，测试没有问题，100VAC，350VDC下对直流侧充电没问题。
 *   			接下来测试220V，650V并网电流环.并网没有问题，但是Q轴电流好像不是很可控（有数值，且一直为正）
 *   			接下来加入电压环。电压环测试完成，下一步使用充电电阻进行预充电。
 *   			此版本需要直流电源给定一定的初始电压。
 *   	9-9		版本1007.尝试使用充电回路进行预充电，去掉直流电源。
 *   			运行过程中加入交流电压幅值测量，允许的范围为：90% ~ 110%
 *   	9-28	将直流电压环输出D轴电流指令限幅改为15A，发现上电后PWM整流充电过程，容易跳开，故将充电电压环与实际使用的电压环分开。
 *   			同时在Menu中，对于PWM整流没有充电成功，给出提示；
 *   	10-8	版本1008.尝试加入中点平衡算法。
 *   			加入NVC_ENABLE 宏定义
 *   	10-20	版本2008.基于1008，适用于第二版PCB，电流传感器反向.新电路板不能使用SMECInverter1xxx程序！！！
 *   			提高了充电电压，充电到达92%再合上继电器。
 *   			修改了几处隐藏的BUG。将程序软件框架优化。
 *   	10-30	版本2009.基于2008.添加逆变部分程序。
 *   			修改了F28335.cmd文件，将.ebss区RAML4，修改为0x003000.加入功率计算.
 *   			发现软件bug。
 *   	11-4	版本2109.基于2009.尝试解决2009中出现的问题。此版本为测试版本。
 *   			并网没有问题，解决了电流突然变大的问题。
 *   	11-11	版本3109.基于2109.将逆变板换成模块+散热片式，可以长时间运行。电流传感器方向更换。测试完成。
 *   			加入编码器接口代码与QUADECODER检测。
 *   			硬件上把交流电压采样的采样电阻更改了，除此版本外，其他版本均不可运行！注意，会炸机的的。
 *   			去除锁相环bug（没有加限幅）。
 *   			SOGI_Update和SVM计算放入RAM中运行，以减少计算时间。
 *   			PID计算也放入RAM中。
 *   	12-12	将整流器的直流电压环放大为+-30，之前为+-20，这个后面慢慢放大。
 *   			去三菱之前的代码，终结于此。
 *   	12-16	版本3110.基于版本3109.后面是在三菱测试。
 *   			加入了编码器信息查询。（Encoder）
 *   	12-18	电机更换为12对级，额定105转的电机。编码器线数8192，编码器定位信号为F0..2，没有F3
 *   	1-12	速度闭环测试较好，保存于2015-1-12日，目前的问题有：低速时，速度测量不正确。
 *   	1-12	版本3112.
 *				添加电机重复启动功能，x关闭电机，$重新启动电机，整流测无需关闭。
 *				电机再次启动时，转子不需要再次定位。
 *		1-26	版本3113.名称为SMEC_PMSM。使用git工具进行代码管理工具。
 *				将过流保护采用另外两相相加的方法。
 */
#include "Global.h"
#include "F28335BSP.h"

volatile unsigned char MachineState;	//整流测状态
volatile unsigned char MotorState;		//逆变侧状态
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

void man(void);						//帮助菜单
void version(void);					//软件版本
void bus_voltage(void);				//直流侧电压
void grid_condition(void);			//网侧参数
void grid_current(void);			//网侧电流
void grid_voltage(void);			//网侧电压
void ShowMachineState(void);		//机器状态
void ShowErrors(void);				//错误
void motor_current(void);			//电机电流
void fpga_error_reg(void);			//fpga错误位检测
void main_relay_on(void);			//主接触器闭合
void main_relay_off(void);			//主接触器断开
void charging_relay_on(void);		//充电电阻闭合
void charging_relay_off(void);		//充电电阻断开
void shutdown(void);				//关闭
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
void cmd_forbidden(void);			//命令被禁止

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
	MachineState = STATE_RESIS_CHARGE;		//进入预充电状态
	i = 0;
	while(i < 10){							//连续监测到100次直流电压充电电压足够高后，合继电器
		if(RectRunningData.DC_Voltage > RectRunningData.VoltageD * 1.73 * 0.92){i ++;}
		else{i = 0;}
		DelayMs(1);
	}

	ProcessRectifierControlInit();
	sPrintf("\r\nInitilized...");

	DelayMs(10);
	sPrintf("\r\nFPGA PWM Reset Done...");
	FPGA_PWM_UPDATE_LOCK;					//保证低电平
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_PREP;				//LCL charging
	sPrintf("\r\nFPGA PREP。");
	DelayMs(300);
	if(MachineState == STATE_ERR){return;}
	MachineState = STATE_DC_UNSTABLE;
	sPrintf("\r\nON Grid.DC Stablizing...");
	i = 0;
	while(i < 100){							//连续检测到100次直流电压稳定，即认为电压稳定在700V
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
