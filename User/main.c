/*
 * SMEC created
 * 初次新建工程
 */
#include "F28335BSP.h"
#include "RectControl.h"
#include "MotorControl.h"
#include "Global.h"
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#define	BURNFLASH

interrupt void EPWM1_ISR(void);
interrupt void ADC_ISR(void);
interrupt void ExternalINT2_ISR(void);
interrupt void CpuTimer1ISR(void);
void ConfigureCPUTimer(void);
volatile unsigned int ADC_ISR_Counter;
volatile unsigned int ObserverValue[200];
volatile unsigned int ObserverValueIndex;
volatile unsigned int ExternalINT2Counter;

void main(void){
	InitSysCtrl();//PLL,WatchDog,Peripheral Clocks, 100M

/*Next Two Statements are used for running program from the FLASH*/
#ifdef BURNFLASH
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
#endif
	DINT;//Disable Interrupts
	InitPieCtrl();//Initialize the PIE control registers
	IER = 0x0000;//Disable CPU interrupts
	IFR = 0x0000;//Clear all CPU interrupt flags

	InitPieVectTable();//Initialize the PIE vector table with pointers to the shell Interrupt

	//FPGA
	ConfigureAllGPIO();
	ConfigureXintf();

	Reset_FPGA();				// 采用FPGA内部FLASH配置   RESET FPGA PROG_B=0
	Enable_FPGA();				// 采用FPGA内部FLASH配置   Config FPGA PROG_B=1 VS[0:2]=111
    Start_FPGA();  				// FPGA软件复位并启动
    //烧写FPGA用
    if(DSPSW2_1 == 0 && DSPSW2_2 == 0 && DSPSW2_3 == 1){	// 拨码开关拨至ON ON OFF 进入烧写FPGA程序模式
  		for(;;){
  			DELAY_US(500000);
  			LED_GPIO31 = 0;
  			DELAY_US(500000);
  			LED_GPIO31 = 1;
  		}
    }
	InitAdc();//Initialize ADC Module4
	DINT;//Disable Interrupts
	InitPieCtrl();//Initialize the PIE control registers
	IER = 0x0000;//Disable CPU interrupts
	IFR = 0x0000;//Clear all CPU interrupt flags

	InitPieVectTable();//Initialize the PIE vector table with pointers to the shell Interrupt
	ConfigureSCI();
	ConfigureCPUTimer();
	ConfigureEQEP();

	EALLOW;  // This is needed to write to EALLOW protected register
	PieVectTable.ADCINT = &ADC_ISR;
	EDIS;    // This is needed to disable write to EALLOW protected registers
	ConfigureADC();
	IER |= M_INT1; // Enable CPU Interrupt 1 for ADC
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &EPWM1_ISR;
	EDIS;    // This is needed to disable write to EALLOW protected registers
	ConfigureAllEPWM();//Configure EPWMs
	IER |= M_INT3;//Enable Group3 Interrupt
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;//Group3 INT1 -> EPWM1 enable
	// Enable global Interrupts and higher priority real-time debug events:

	EALLOW;
	PieVectTable.XINT2 = &ExternalINT2_ISR;
	EDIS;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
	IER |= M_INT1;

	DelayMs(10);
	FPGA_ENABLE;			//启动FPGA时钟，否则，没有外部同步信号中断。

    ADC_ISR_Counter = 0;
    ObserverValueIndex = 0;
    ExternalINT2Counter = 0;

    RectControlInit();
	InvControlInit();
	EncoderInit(1);
	MachineState = STATE_IDLE;
	MotorState = MOTOR_IDLE;

	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	MainMenu();
}

interrupt void EPWM1_ISR(void){//10KHz
	EPwm1Regs.ETCLR.bit.INT = 1;//Clear flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void ADC_ISR(void){

	//Sampling....
	RectSampleRawData.U_Current = AdcRegs.ADCRESULT2 >> 4;
	RectSampleRawData.V_Current = AdcRegs.ADCRESULT1 >> 4;
	RectSampleRawData.W_Current = AdcRegs.ADCRESULT0 >> 4;
	RectSampleRawData.U_Voltage = AdcRegs.ADCRESULT15 >> 4;
	RectSampleRawData.V_Voltage = AdcRegs.ADCRESULT14 >> 4;
	RectSampleRawData.W_Voltage = AdcRegs.ADCRESULT13 >> 4;
	RectSampleRawData.P_Voltage = (AdcRegs.ADCRESULT5 >> 4) + (AdcRegs.ADCRESULT4 >> 4);
	RectSampleRawData.N_Voltage = (AdcRegs.ADCRESULT9 >> 4) + (AdcRegs.ADCRESULT8 >> 4);

	InvSampleRawData.U_Current = AdcRegs.ADCRESULT10 >> 4;
	InvSampleRawData.V_Current = AdcRegs.ADCRESULT11 >> 4;
	InvSampleRawData.W_Current = AdcRegs.ADCRESULT12 >> 4;
	InvSampleRawData.P_Voltage = RectSampleRawData.P_Voltage;
	InvSampleRawData.N_Voltage = RectSampleRawData.N_Voltage;

	ObserverValue[ObserverValueIndex++] = AdcRegs.ADCRESULT2 >> 4;
	if(ObserverValueIndex > 199){ObserverValueIndex = 0;}

	ProcessRectSampleDataRoutine();
	ProcessInvSampleDataRoutine();
	GridPLLRoutine();

	switch(MachineState){
		case STATE_IDLE:
			FPGA_PWM_SHUTDOWN;
			RPWM_DISABLE;
			IPWM_DISABLE;
			MR_DISABLE;
			CR_DISABLE;
			break;
		case STATE_RESIS_CHARGE:							//预充电
			FPGA_PWM_SHUTDOWN;
			RPWM_DISABLE;
			IPWM_DISABLE;
			MR_DISABLE;
			CR_ENABLE;
			break;
		case STATE_PREP:									//FPGA开始发送PWM波，但是不通过244，用于将FPGA第一个周期刷新掉
			MR_ENABLE;										//合上继电器
			CR_DISABLE;
			ProcessRectifierPREP();
			FPGA_PWM_ENABLE;
			RPWM_DISABLE;
			break;
		case STATE_DC_UNSTABLE:								// 直流充电，电压还不稳定
		case STATE_ONGRID:
			if(RectProtectionRoutine(FpgaRegs.IGBTTz)){		// Errors Occurred
				MR_DISABLE;									// Off Grid
				//FPGA_PWM_SHUTDOWN;
				RPWM_DISABLE;
				IPWM_DISABLE;
				CR_DISABLE;
				MachineState = STATE_ERR;
				MotorState = MOTOR_ERR;
			}else{											// Take it easy ~ Everything is OK
				ProcessRectifierControl();
				FPGA_PWM_ENABLE;
				RPWM_ENABLE;
				MR_ENABLE;
				ADC_ISR_Counter += 9;
			}
			break;
		case STATE_ERR:
			//FPGA_PWM_SHUTDOWN;
			RPWM_DISABLE;
			IPWM_DISABLE;
			MR_DISABLE;
			CR_DISABLE;
			break;
		default:break;
	}

	switch(MotorState){
		case MOTOR_IDLE:
			IPWM_DISABLE;
			break;
		case MOTOR_PREP:									// FPGA发波，但驱动信号不给，五个周期
			if(InvProtectionRoutine()){						// 电机侧保护
				MR_DISABLE;
				CR_DISABLE;
				RPWM_DISABLE;
				IPWM_DISABLE;
				MachineState = STATE_ERR;
				MotorState = MOTOR_ERR;
			}
			InvRunningData.MotorPWMPREP ++;
			if(MachineState != STATE_ONGRID){MotorState == MOTOR_ERR;break;}	//整流测状态不对，无法启动电机
			ProcessInverterControl();
			IPWM_DISABLE;									// 这里关闭
			break;
		case MOTOR_ON:
			if(InvProtectionRoutine()){						// 电机侧保护
				MR_DISABLE;
				CR_DISABLE;
				RPWM_DISABLE;
				IPWM_DISABLE;
				MachineState = STATE_ERR;
				MotorState = MOTOR_ERR;
			}
			if(MachineState != STATE_ONGRID){MotorState == MOTOR_ERR;break;}	//整流测状态不对，无法启动电机
			ProcessMotorSpeedLoop();											//速度环
			ProcessInverterControl();											//电流环
			IPWM_ENABLE;
			break;
		case MOTOR_ERR:
			IPWM_DISABLE;
			break;
	}

	ProcessEncoder();

	FPGA_PWM_UPDATE_LOCK;
//	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
	FpgaRegs.u_rec = REC_PWM[0];
	FpgaRegs.v_rec = REC_PWM[1];
	FpgaRegs.w_rec = REC_PWM[2];
	FpgaRegs.u_inv = INV_PWM[0];
	FpgaRegs.v_inv = INV_PWM[1];
	FpgaRegs.w_inv = INV_PWM[2];
	FPGA_PWM_UPDATE_UNLOCK;

	ADC_ISR_Counter++;
	if(ADC_ISR_Counter < 5000){		// ON Grid 100ms, Off Grid 1s
		FpgaRegs.FpgaLed.bit.led3 = 1;
	}else if(ADC_ISR_Counter >= 5000 && ADC_ISR_Counter < 10000){
		FpgaRegs.FpgaLed.bit.led3 = 0;
	}else if(ADC_ISR_Counter > 20000){
		ADC_ISR_Counter = 0;
	}

	SCITransmit();
	GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
	// Reinitialize for next ADC sequence
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

interrupt void ExternalINT2_ISR(void){
	GpioDataRegs.GPBSET.bit.GPIO39 = 1;
	ExternalINT2Counter++;
	if(ExternalINT2Counter < 10000){		// ON Grid 100ms, Off Grid 1s
		FpgaRegs.FpgaLed.bit.led2 = 1;
	}else if(ExternalINT2Counter >= 10000 && ExternalINT2Counter <= 20000){
		FpgaRegs.FpgaLed.bit.led2 = 0;
	}else if(ExternalINT2Counter > 20000){
		ExternalINT2Counter = 0;
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

interrupt void CpuTimer1ISR(void){
	CpuTimer1.InterruptCount ++;
	if(CpuTimer1.InterruptCount % 2){
		//FpgaRegs.FpgaLed.bit.led1 = 1;
	}else{
		//FpgaRegs.FpgaLed.bit.led1 = 0;
	}
}

void ConfigureCPUTimer(void){
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.XINT13 = &CpuTimer1ISR;
	EDIS;    // This is needed to disable write to EALLOW protected registers
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
	ConfigCpuTimer(&CpuTimer1, 150, 1000000);//1S
	CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	IER |= M_INT13;
}


