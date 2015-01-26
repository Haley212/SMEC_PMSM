/*
 * F28335BSP.c
 *
 *  Created on: 2013-3-19
 *      Author: appleseed
 */
#include "F28335BSP.h"
#include "Printf_sci.h"
#include "Menu.h"

unsigned char SCIRxBuffer[32];
unsigned char SCIRxBufferIndex = 0;

void ConfigureAllGPIO(void)
{
   EALLOW;

   //------------------EPWM Section---------------------//

    /*GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;   // Disable pullup on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;   // Disable pullup on GPIO1
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;   // Disable pullup on GPIO2
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;   // Disable pullup on GPIO3
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;   // Disable pullup on GPIO4
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;   // Disable pullup on GPIO5

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;   // Disable pullup on GPIO6
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;   // Disable pullup on GPIO7
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;   // Disable pullup on GPIO8
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;   // Disable pullup on GPIO9
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;   // Disable pullup on GPIO10
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;   // Disable pullup on GPIO11

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;  // GPIO6 = PWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;  // GPIO7 = PWM4B
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;  // GPIO8 = PWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;  // GPIO9 = PWM5B
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;  // GPIO10 = PWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;  // GPIO11 = PWM6B

   //Trip Zone Hardware protection
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPIO12 TZ1
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO13 TZ2
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO14 TZ3
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pullup on GPIO15 TZ4
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pullup on GPIO16 TZ5
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;	  // Enable pullup on GPIO17 TZ6


    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO12 = TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // GPIO13 = TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // GPIO14 = TZ3
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // GPIO15 = TZ4
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // GPIO16 = TZ5
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // GPIO17 = TZ6

    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 2; // 6个采样点才能触发TZ的改变
	GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 2; // 6个采样点才能触发TZ的改变
	GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 2; // 6个采样点才能触发TZ的改变
	GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 2; // 6个采样点才能触发TZ的改变
	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 2; // 6个采样点才能触发TZ的改变
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 2; // 6个采样点才能触发TZ的改变
*/
 //------------------EPWM END--------------------//


 //------------------FPGA PROG_B Section---------------------//
 	GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;			//GPIO61 configs as GPIO Disable PUD
 	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
 	GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;         // GPIO61 = OUTPUT;
 	GpioDataRegs.GPBDAT.bit.GPIO61 = 0;			//FPGA RESET = 0, START = 1

 	GpioCtrlRegs.GPBPUD.bit.GPIO34  = 0;		// ENABLE pud
 	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;		// GPIO34
 	GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;		// output
 	GpioDataRegs.GPBSET.bit.GPIO34  = 1;		// high = software reset low = fpga start

 	GpioCtrlRegs.GPBPUD.bit.GPIO59  = 0;		// enable pullup
 	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;		// CCLK
 	GpioCtrlRegs.GPBDIR.bit.GPIO59  = 1;		// out
 	GpioDataRegs.GPBSET.bit.GPIO59  = 1;		// 1

 	GpioCtrlRegs.GPBPUD.bit.GPIO60  = 0;		// enable pullup
 	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;		// DOUT
 	GpioCtrlRegs.GPBDIR.bit.GPIO60  = 1;		// out
 	GpioDataRegs.GPBSET.bit.GPIO60  = 1;		// 1

 //------------------FPGA PROG_B END-----------------//



  //------------------ECAN Section--------------------//
/*
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	    // Enable pull-up for GPIO18 (CANRXA)
  	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	    // Enable pull-up for GPIO19 (CANTXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;	// Async input
  	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;	// Configure GPIO18 for CANRXA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;	// Configure GPIO19 for CANTXA operation
*/
  //-------------------ECAN END-----------------------//



 //------------------I2C Section--------------------//
 /*
 	GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
 	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;
 	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;       //Asynchonous input
 	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;       //Asynchonous input
 	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;		//GPIO32 configs as I2C SDA
 	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;		// I2C SCL
 */
 //-------------------I2C END-----------------------//

 //------------------XINTF Section---------------------//
 //--			地址线上的上拉电阻均使能			--
// 	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 2;    //Configs as XZCS6
// 	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 2;    //Configs as XR/W
// 	GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 2;	//Configs as DXZCS0
// 	GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 2;	//Configs as DXZCS7
// 	GpioCtrlRegs.GPBPUD.bit.GPIO38 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 2;	//Configs as XWE0

// 	GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;	//Configs as XA0
// 	GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;	//Configs as XA1
// 	GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 2;	//Configs as XA2
// 	GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 2;	//Configs as XA3
// 	GpioCtrlRegs.GPBPUD.bit.GPIO44 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 2;	//Configs as XA4
// 	GpioCtrlRegs.GPBPUD.bit.GPIO45 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 2;	//Configs as XA5
// 	GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 2;	//Configs as XA6
// 	GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;		//Enable Pull up
 	GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 2;	//Configs as XA7


 	GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
    GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
    GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
    GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
    GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
    GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
    GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
    GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
    GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0


 //------------------XINTF END-----------------//

/****************FPGA IO Communication***************/
    //GPIO16连接PWM_Enable，高电平使能，低电平禁止
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;//output
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;

    //GPIO17连接PWM_Update，高电平刷新，低电平不刷新，常态是高电平
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;

    //GPIO27,PWM Shutdown, 低电平闭锁
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

/****************FPGA IO Communication***************/

 //-------------------SCI SECTION-------------//
 /*	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;	   // Enable pull-up for GPIO22 (SCITXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;	   // Enable pull-up for GPIO23 (SCIRXDB)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation

    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;	// Gpio21 as RS485-TX_ENABLE
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;		// Output TX ENABLE 485
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;		// Disable
    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;	// Output = low Recieve Enable
*/
 //-------------------SCI END-----------------//

 //------------------SPI SECTION-------------//
 //   GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;
 //   GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;
 //   GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;

 //   GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;
 //   GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;
 //   GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1;

 //-------------------SPI END-----------------//

 //-------------------GPIO SECTION------------//

 // 拨码开关部分
 	GpioCtrlRegs.GPCPUD.bit.GPIO80 = 0;		// Enable Pullup
 	GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;		// Enable Pullup
 	GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0;		// Enable Pullup
 	GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0;		// Enable Pullup

 	GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 3;	// XA8 as gpio
 	GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 3;	// XA9 as gpio
 	GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 3;	// XA10 as gpio
	GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 3;	// XA11 as gpio

	GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0;		// input DSPSW1.4
	GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;		// input DSPSW1.3
	GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;		// input DSPSW1.2
	GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0;		// input DSPSW1.1

	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;		// Enable PU
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;		// Enable PU
 	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;		// Enable PU

 	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;	// GPIO
 	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;	// GPIO
 	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;	// GPIO

 	GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;		// input DSPSW2.1
 	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;		// input DSPSW2.2
 	GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;		// input DSPSW2.3

 //----------------GPIO END----------------------//

 //------------------LED SECTION-----------------//
	GpioCtrlRegs.GPBPUD.bit.GPIO39 = 1;				// disable
	GpioCtrlRegs.GPAPUD.bit.GPIO30 = 1; 			// disable
	GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1; 			// disable

	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;          //GPIO30 = OUTPUT
	GpioDataRegs.GPASET.bit.GPIO30 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;          //GPIO31 = OUTPUT
	GpioDataRegs.GPASET.bit.GPIO31 = 1;
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;          //GPIO39 = OUTPUT
	GpioDataRegs.GPBSET.bit.GPIO39 = 1;


 //------------------LED END------------------//


//-------------------FAN GOIO-----------------//
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
	GpioDataRegs.GPASET.bit.GPIO8 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO9 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
//-------------------FAN END------------------//

//------------PWM Enable Section--------------//
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
	IPWM_DISABLE;

	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
	RPWM_DISABLE;
//-------------------END----------------------//

//-------------Relays Enable------------------//
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
	MR_DISABLE;

	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
	CR_DISABLE;
//-----------------END------------------------//

//----------------Encoder---------------------//
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;	//GPIO52 -> F0
	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;	//GPIO49 -> F1
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;	//GPIO48 -> F2
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;	//GPIO58 -> F3
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;	//GPIO50 -> CHA
	GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;	//GPIO51 -> CHB
	GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;		//Input

	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;	//GPIO53 -> CHZ
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;		//Input

	GpioCtrlRegs.GPBCTRL.bit.QUALPRD3 = 75;	// 32 samples
	GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 75;
//-------------End OF Encoder-----------------//

// Xint2
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;		//input
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 2;	//6 samples

	GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 26;	//gpio26 as input

	XIntruptRegs.XINT2CR.bit.POLARITY = 1;	//rising
	XIntruptRegs.XINT2CR.bit.ENABLE = 1;

   EDIS;
}

void ConfigureAllEPWM(void){
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;//Disable Clock
	EDIS;

	ConfigureEPWMn(&EPwm1Regs);
	ConfigureEPWMn(&EPwm2Regs);
	ConfigureEPWMn(&EPwm3Regs);
	ConfigureEPWMn(&EPwm4Regs);
	ConfigureEPWMn(&EPwm5Regs);
	ConfigureEPWMn(&EPwm6Regs);

    // Interrupt where we will change the Compare Values
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;	               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 3rd event

    //trigger SOC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;       // Select SOC from ZERO
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;//Enable Clock
	EDIS;

	/*EPwm1Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD;
	EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm3Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD;
	EPwm4Regs.CMPA.half.CMPA = 0;
	EPwm5Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD;
	EPwm6Regs.CMPA.half.CMPA = 0;*/
}

static void ConfigureEPWMn(volatile struct EPWM_REGS *EPWMn){
	EPWMn->TBPRD = EPWM_TIMER_TBPRD;
	EPWMn->TBPHS.half.TBPHS = 0x0000;
	EPWMn->TBCTR = 0x0000;

	EPWMn->CMPA.half.CMPA = 0;
	EPWMn->CMPB = 0;

	EPWMn->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPWMn->TBCTL.bit.PHSEN = TB_DISABLE;
	EPWMn->TBCTL.bit.PRDLD = TB_SHADOW;
	EPWMn->TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPWMn->TBCTL.bit.CLKDIV = TB_DIV1;

	EPWMn->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPWMn->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPWMn->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPWMn->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPWMn->AQCTLA.bit.CAU = AQ_SET;		//Set PWM1A on EventA, Up Count
	EPWMn->AQCTLA.bit.CAD = AQ_CLEAR;	//Clear PWM1A on EventA, Down Count
	//EPWMn->AQCTLB.bit.CBU = AQ_SET;
	//EPWMn->AQCTLB.bit.CBD = AQ_CLEAR;

	EPWMn->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPWMn->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPWMn->DBCTL.bit.IN_MODE = DBA_ALL;
	EPWMn->DBRED = DEAD_TIME;			// Falling Edge Delay = 100 TBCLKs
	EPWMn->DBFED = DEAD_TIME;			// Raising Edge Delay = 100 TBCLKs
}

void SetPWMDutyCycle(unsigned int *Dty){
	EPwm1Regs.CMPA.half.CMPA = Dty[0];
	EPwm2Regs.CMPA.half.CMPA = Dty[1];
	EPwm3Regs.CMPA.half.CMPA = Dty[2];
	EPwm4Regs.CMPA.half.CMPA = Dty[3];
	EPwm5Regs.CMPA.half.CMPA = Dty[4];
	EPwm6Regs.CMPA.half.CMPA = Dty[5];
}

void ConfigureSCI(void){
	EALLOW;
	//GPIO62->SCIRXDC GPIO63->SCITXDC
	//Pull Up
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;

	ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit, 8 bit, No Parity
	ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.bit.TXINTENA = 1; // TX Interrupt Enable
	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;//Rx Interrupt Enable
	//BaudRate = LSPCLK/(BRR+1)/8
	//LSPCLK = SYSOUT/4 (reset default and configurable)
	//BRR = LSPCLK/8*BaudRate - 1
	//115200 = 37.5M / 8 / 115200 -1 = 39
	ScicRegs.SCIHBAUD = 0;
	ScicRegs.SCILBAUD = 39;
	ScicRegs.SCICTL1.all =0x0023;// ReEnable SCI after Reset

	//Enhanced Mode with FIFO
	ScicRegs.SCIFFTX.all = 0xE001;
	//SCI FIFO Enahancements enabled, Depth 16 word,Interrupt Disable
	ScicRegs.SCIFFRX.all = 0x2021;
	//SCI FIFO Rx enabled, Depth 16 word, Interrupt enabled

	PieVectTable.SCIRXINTC = &SCICRXISR;
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;//Group8 INT5, RX Interrupt
	IER |= M_INT8;

	EDIS;
}

interrupt void SCICRXISR(void){
	unsigned char Recvd;
	unsigned char i;
	if(ScicRegs.SCIFFRX.bit.RXFFINT){//Rx Interrupted Occurred
		Recvd = ScicRegs.SCIRXBUF.all;
		switch(Recvd){
			case 0x0D:for(i=0;i<SCIRxBufferIndex;i++){		//回车
					SCICmdBuffer[i] = SCIRxBuffer[i];
					SCIRxBuffer[i] = 0;
				}
				SCICmdBuffer[SCIRxBufferIndex] = 0;			//后面加上\0
				SCICmdBufferIndex = SCIRxBufferIndex + 1;	//多了\0，Menu响应输入
				SCIRxBufferIndex = 0;
				break;
			case 0x1B:SCICmdBuffer[0] = 0x1B;				//ESC
				SCICmdBufferIndex = 1;
				break;
			case 0x08:SCIRxBufferIndex = 0; 				//backspace
				SCITransmitByte(0x08);
				break;
			default:SCITransmitByte(Recvd);
				SCIRxBuffer[SCIRxBufferIndex] = Recvd;
				SCIRxBufferIndex ++;
				break;
		}
	}
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;//Clear flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

void SCITransmit(void)
{
//	while(!ScicRegs.SCICTL2.bit.TXRDY);

	if(SciDataInP!=SciDataOutP)
	{
		if(ScicRegs.SCIFFTX.bit.TXFFST<16)
		{
			ScicRegs.SCITXBUF = SciDataBuff[SciDataOutP];
			SciDataOutP=(SciDataOutP+1)%SCIDATABUFF;
		}
	}

}

void SCITransmitByte(unsigned char Buffer){
	while(!ScicRegs.SCICTL2.bit.TXRDY);
	ScicRegs.SCITXBUF = Buffer;
}

void ConfigureADC(void){
	// Configure ADC Using Sequential Sampling Mode
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0x1; // Setup cascaded sequencer mode
    AdcRegs.ADCTRL1.bit.ACQ_PS = 15;//16 cycle sample and hold
    AdcRegs.ADCTRL1.bit.CPS = 1;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 1;
    AdcRegs.ADCMAXCONV.all = 0x000F;    // 16 CONV
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9;
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xA;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xB;
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xC;
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xD;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xE;
    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xF;
//    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Started by ePWMx SOCA trigger
//    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)
    AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1 = 1;// started by Xint2
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;// enable SEQ1 interrupt
}

/*******************************Encoder*************************************/
void ConfigureEQEP(void){
	#if (CPU_FRQ_150MHZ)
	  EQep1Regs.QUPRD=1500000;			// 输出100Hz中断，每10ms锁定位置信号
	#endif
	#if (CPU_FRQ_100MHZ)
	  EQep1Regs.QUPRD=1000000;			//
	#endif

	EQep1Regs.QDECCTL.bit.QSRC=00;		// 正交编码模式

	EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep1Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
	EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable
	EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep1Regs.QPOSMAX= (unsigned int)32767;
	EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

	EQep1Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock , 150M/128
	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable
}
/***************************************************************************/

/***************************************************************************/
/********************************FPGA***************************************/
volatile struct Fpga_Regs FpgaRegs;
#pragma DATA_SECTION(FpgaRegs,"fpga_ram");
// 内存分配
// ---ZONE0---保留
// ---ZONE6---FPGA交互
// ---ZONE7---保留

void ConfigureXintf(void)
{

    EALLOW;

    XintfRegs.XRESET.bit.XHARDRESET = 1;	// reset xintf


    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;	  //XTIMCLK=SYSCLKOUT/2
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;	  //NO Buffer
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;    // XCLKOUT is disabled
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;   //XCLKOUT=XTIMCLK/2=SYSCLKOUT/4=37.5M@150MHz

    // Zone 0------------------------------------
    XintfRegs.XTIMING0.bit.XWRLEAD = 3;
    XintfRegs.XTIMING0.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING0.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING0.bit.XRDLEAD = 3;
    XintfRegs.XTIMING0.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING0.bit.XRDTRAIL = 3;

    XintfRegs.XTIMING0.bit.X2TIMING = 1;   //double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING0.bit.USEREADY = 0;   // Zone will sample XREADY signal
    XintfRegs.XTIMING0.bit.READYMODE = 1;  // sample asynchronous

    XintfRegs.XTIMING0.bit.XSIZE = 3;		//Zone0 Width = 16bits

    // Zone 6------------------------------------

    XintfRegs.XTIMING6.bit.XWRLEAD = 1;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 3;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 3;

    XintfRegs.XTIMING6.bit.XRDLEAD = 1;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 2;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 1;


    XintfRegs.XTIMING6.bit.X2TIMING = 1;   // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.USEREADY = 0;   // Zone will not sample XREADY signal
    XintfRegs.XTIMING6.bit.READYMODE = 1;  // sample asynchronous

    XintfRegs.XTIMING6.bit.XSIZE = 3;  //Width=16bit

    // Zone 7------------------------------------

    XintfRegs.XTIMING7.bit.XWRLEAD = 1;		//WR Lead
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7;	//WR Active
    XintfRegs.XTIMING7.bit.XWRTRAIL = 1;	//WR Trail

    XintfRegs.XTIMING7.bit.XRDLEAD = 1;		//RD Lead
    XintfRegs.XTIMING7.bit.XRDACTIVE = 3;	//RD Active
    XintfRegs.XTIMING7.bit.XRDTRAIL = 2;	//RD Trail

    XintfRegs.XTIMING7.bit.X2TIMING = 1;    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.USEREADY = 0;	// Zone will not sample XREADY signal
    XintfRegs.XTIMING7.bit.READYMODE = 1;   // sample asynchronous

    XintfRegs.XTIMING7.bit.XSIZE = 3;		//Width=16bit

    XintfRegs.XBANK.bit.BANK = 7;
    XintfRegs.XBANK.bit.BCYC = 2;
    EDIS;

   asm(" RPT #15 || NOP");

}

void Enable_FPGA(void)
{
	GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;   //PROG_B pin of FPGA = 0
	DELAY_US(100);							//Delay 1ms 500ns is enough in fact
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;
	DELAY_US(500000); 						//Waiting for FPGA to start -- 0.5s
}


void Reset_FPGA(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
	DELAY_US(10);
}

void Stop_FPGA(void)
{
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;
}

void Start_FPGA(void)
{
	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
}

//------------读Checkvalue1和2 不对就说明有问题----
int CheckFpga(void)
{
	if(FpgaRegs.CheckValue1 == 0xaaaa && FpgaRegs.CheckValue2 == 0x5555)
		return FpgaRight;
	else
		return FpgaError;
}

//-------------检查软件版本---------
int CheckSoftwareVersion(void)
{
	int checktemp;

	checktemp = Fpga_Software_Version;

	if(FpgaRegs.FpgaSoftwareVersion == checktemp)
		return FpgaRight;
	else
		return FpgaError;
}

void DelayMs(unsigned int T){
	volatile unsigned int i,j,k;
	for(i=0;i<T;i++){
		for(j=0;j<5000;j++){
			k++;
		}
	}
}
