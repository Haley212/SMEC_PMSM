/*
 * F28335BSP.h
 *
 *  Created on: 2013-3-19
 *      Author: appleseed
 *      Log:	28335 & FPGA HAL
 */

#ifndef F2835BSP_H_
#define F28335BSP_H_
#include "DSP28x_Project.h"

extern unsigned char SCIRxBuffer[32];
extern unsigned char SCIRxBufferIndex;

// DSP SWITCH 拨码开关定义
#define DSPSW1_1 GpioDataRegs.GPCDAT.bit.GPIO83
#define DSPSW1_2 GpioDataRegs.GPCDAT.bit.GPIO82
#define DSPSW1_3 GpioDataRegs.GPCDAT.bit.GPIO81
#define DSPSW1_4 GpioDataRegs.GPCDAT.bit.GPIO80

#define DSPSW2_1 GpioDataRegs.GPADAT.bit.GPIO20
#define DSPSW2_2 GpioDataRegs.GPADAT.bit.GPIO24
#define DSPSW2_3 GpioDataRegs.GPADAT.bit.GPIO25

// DSP LED 发光二极管定义
#define LED_GPIO30 GpioDataRegs.GPADAT.bit.GPIO30
#define LED_GPIO31 GpioDataRegs.GPADAT.bit.GPIO31
#define LED_GPIO39 GpioDataRegs.GPBDAT.bit.GPIO39

// FAN Control
#define FAN1_ON				GpioDataRegs.GPASET.bit.GPIO8 = 1;
#define FAN1_OFF			GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
#define FAN2_ON				GpioDataRegs.GPASET.bit.GPIO9 = 1;
#define FAN2_OFF			GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
#define FAN3_ON				GpioDataRegs.GPASET.bit.GPIO10 = 1;
#define FAN3_OFF			GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
#define FAN4_ON				GpioDataRegs.GPASET.bit.GPIO11 = 1;
#define FAN4_OFF			GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

// PWM Enable
#define IPWM_DISABLE		GpioDataRegs.GPASET.bit.GPIO4 = 1;
#define IPWM_ENABLE			GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
#define RPWM_DISABLE		GpioDataRegs.GPASET.bit.GPIO6 = 1;
#define RPWM_ENABLE			GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

// Relays
#define MR_ENABLE			GpioDataRegs.GPASET.bit.GPIO5 = 1;
#define MR_DISABLE			GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
#define CR_ENABLE			GpioDataRegs.GPASET.bit.GPIO7 = 1;
#define CR_DISABLE			GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

void ConfigureAllGPIO(void);

#define EPWM_TIMER_TBPRD 7500

#define FULL_PWM 5000
#define DEAD_TIME 200//1uS

//PWM
void ConfigureAllEPWM(void);
static void ConfigureEPWMn(volatile struct EPWM_REGS *EPWMn);
void SetPWMDutyCycle(unsigned int *Dty);

//SCI
void ConfigureSCI(void);
void SCITransmit(void);
interrupt void SCICRXISR(void);
void SCITransmitByte(unsigned char Buffer);

//ADC
void ConfigureADC(void);

//Encoder
void ConfigureEQEP(void);

//FPGA
#define Fpga_Software_Version 1001
#define FpgaRight 0
#define FpgaError 1

//FPGA FUNCTION
//#define FPGA_DISABLE			GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;		//FPGA 停止工作，同时刷新保护寄存器
#define FPGA_ENABLE				GpioDataRegs.GPASET.bit.GPIO16 = 1;			//FPGA 使能，必须要在FPGA_PWM_SHUTDOWN时使能FPGA
#define FPGA_PWM_UPDATE_LOCK 	GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;		//FPGA 接收数据锁定
#define FPGA_PWM_UPDATE_UNLOCK	GpioDataRegs.GPASET.bit.GPIO17 = 1;			//放开
#define FPGA_PWM_SHUTDOWN 		GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;		//FPGA 关闭PWM波
#define FPGA_PWM_ENABLE			GpioDataRegs.GPASET.bit.GPIO27 = 1;			//使能PWM波
//-----------为方便FPGA ram的使用，特别定义了以下结构体，模仿了片内外设寄存器的RAM分配方式。
//-----------不同于片内寄存器，由于读写片外寄存器的速度较慢，尽量不要对结构体进行位操作，对寄存器操作要一步到位。
struct Fpga_Led_bits{
	unsigned int led1:1;
	unsigned int led2:1;
	unsigned int led3:1;
	unsigned int rsvd:13;
};

union Fpga_Led{
	unsigned int all;
	struct Fpga_Led_bits bit;
};

struct Fpga_Regs {
	unsigned int 		FpgaSoftwareVersion;			//baseaddr+0 软件版本号 与DSP的版本号相同的FPGA程序才运行，否则要报错
	unsigned int 		rsvd1;							//baseaddr+1 rsvd
	unsigned int 		CheckValue1;					//baseaddr+2 用于检测 若不等于0xaaaa 说明出事了
	unsigned int 		CheckValue2;					//baseaddr+3 用于检测 若不等于0x5555 说明出事了
	unsigned int        rsvd2;							// +4 rsvd
	unsigned int        rsvd3;							// +5 rsvd
	unsigned int        rsvd4;							// +6 rsvd
	union Fpga_Led 		FpgaLed;						//baseaddr+7 用于亮灯
	unsigned int 		rsvd5;							// +8
	unsigned int 		rsvd6;							// +9
	unsigned int 		rsvd7;							// +10
	unsigned int 		IGBTTz;							// +11
	unsigned int		u_rec;							// +12
	unsigned int		v_rec;							// +13
	unsigned int		w_rec;							// +14
	unsigned int		u_inv;							// +15
	unsigned int		v_inv;							// +16
	unsigned int		w_inv;							// +17
	unsigned int		top;							// +18
};

extern volatile struct Fpga_Regs FpgaRegs;
void ConfigureXintf(void);
void Enable_FPGA(void);
void Reset_FPGA(void);
void Stop_FPGA(void);
void Start_FPGA(void);
extern int CheckFpga(void);
extern int CheckSoftwareVersion(void);

//Delay
void DelayMs(unsigned int T);


#endif /* F28335BSP_H_ */
