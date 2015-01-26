/*
 * SVM.c
 *
 *  Created on: 2013-3-19
 *      Author: appleseed
 */

#include "SVM.h"
#include "math.h"
#include "F28335BSP.h"

#pragma CODE_SECTION(SVPWM, "ramfuncs");
#pragma CODE_SECTION(SVPWMCalcuSector, "ramfuncs");
#pragma CODE_SECTION(ABC2DQ, "ramfuncs");
#pragma CODE_SECTION(DQ2ABC, "ramfuncs");

//Some ConsTant
float SVMArguments[SVM_ARG_L];
unsigned char SectorObj[3];		//Main and Co-Sectors
float Ta, Tb, Tc, D1FLOAT, D2FLOAT, D3FLOAT;
unsigned int D1,D2,D3;
float Urefa, Urefb;
float UrefLength;
/*
* PWM1&PWM2->A
* PWM3&PWM4->B
* PWM5&PWM6->C
*/
unsigned int REC_PWM[3];
unsigned int INV_PWM[3];
/*
sPIDParams DCurrentPI;
sPIDParams QCurrentPI;
sPIDParams MidVoltagePI;
sDQ		   DQ;
*/
/*
 * Author: HuangruZhu
 * Email : appleseed@sjtu.edu.cn
 * Time  : 2014-6-20
 * Arguments: SVPWM Parameters
 * Version :0.3 FPGA use only
 * 			with NVC
 * Included: SVPWM()&SVPWMCalcuSector()
 */
void SVPWM(float * Arg, unsigned int *PWM){
	float DCVoltage = Arg[DC_VOLTAGE];
	//float SVMAngle = Arg[SVM_ANGLE]*CONVERSION_OF_ARC;
//	float V_OUT	= Arg[VOLTAGE_ACT];
	float Ua, Ub, Uc;
	float Shift = Arg[VOLTAGE_SHIFT];
	float Ts,Ts_DFLOAT;
//	float Ts_DFLOATn[6];
	float Ia = Arg[CURRENT_A];
	float Ib = Arg[CURRENT_B];
	float Ic = Arg[CURRENT_C];

    Ua = Arg[VOLTAGE_A];
    Ub = Arg[VOLTAGE_B];
	Uc = Arg[VOLTAGE_C];

	Urefa = 1.0 * Ua - 0.5 * Ub - 0.5 * Uc;
	Urefb = HALF_SQRT_3 * Ub - HALF_SQRT_3 * Uc;
	UrefLength = sqrt(Urefa*Urefa + Urefb*Urefb);

	if(UrefLength > SQRT_3*DCVoltage){
	    Urefa = Urefa*SQRT_3*DCVoltage/UrefLength;
	    Urefb = Urefb*SQRT_3*DCVoltage/UrefLength;
	}else{
	}

	Urefa=Urefa / DCVoltage;
	Urefb=Urefb / DCVoltage;

	SVPWMCalcuSector(SectorObj);
	switch(SectorObj[1]){
		case 1:if(SectorObj[2]==2){Ta=Urefa-0.577350269*Urefb;Tb=1.154700538*Urefb;Tc=1-Ta-Tb;}
			else{Ta=1.154700538*Urefb;Tc=Urefa-0.577350269*Urefb;Tb=1-Ta-Tc;}break;
		case 2:if(SectorObj[2]==2){Tc=Urefa+0.577350269*Urefb-1;Tb=0.577350269*Urefb-Urefa+1;Ta=1-Tb-Tc;}
			else{Ta=0.577350269*Urefb-Urefa+1;Tb=Urefa+0.577350269*Urefb-1;Tc=1-Ta-Tb;}break;
		case 3:Ta=2-Urefa-0.577350269*Urefb;Tb=Urefa-0.577350269*Urefb;Tc=1-Ta-Tb;break;
		case 4:Ta=2-Urefa-0.577350269*Urefb;Tc=1.154700538*Urefb;Tb=1-Ta-Tc;break;
	}

	/* NVC */
	//	Ts = U(delta)*Cdc(470uF *3)/2/In
	//	Ts_DFLOAT = Ts * 10000(switch freq) * FULL_PWM
	Ts = Shift * 750e-2;		//	Ts = U(delta)*Cdc(470uF *3) * 10k /2
	Ts_DFLOAT = 0.0f;
	/*
	Ts_DFLOATn[0] = FULL_PWM * 10000.0f * Ts / Ia;
	Ts_DFLOATn[1] = -FULL_PWM * 10000.0f * Ts / Ic;
	Ts_DFLOATn[2] = FULL_PWM * 10000.0f * Ts / Ib;
	Ts_DFLOATn[3] = -FULL_PWM * 10000.0f * Ts / Ia;
	Ts_DFLOATn[4] = FULL_PWM * 10000.0f * Ts / Ic;
	Ts_DFLOATn[5] = -FULL_PWM * 10000.0f * Ts / Ib;*/
	if(Ia > -0.5 && Ia < 0.5){Ia = 0.5f;Ts = 0.0f;}
	if(Ib > -0.5 && Ib < 0.5){Ib = 0.5f;Ts = 0.0f;}
	if(Ic > -0.5 && Ic < 0.5){Ic = 0.5f;Ts = 0.0f;}

	//	Ts_DFLOAT = Ts * 1(switch freq) * FULL_PWM / In
	switch(SectorObj[0]){				//main sector
		case 1:	if(SectorObj[2] == 1){	//30-60
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ic;
					}
				}else{					//0-30
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ia;
					}
				}
				break;
		case 2:	if(SectorObj[2] == 1){	//60-90
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ic;
					}
				}else{					//90-120
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ib;
					}
				}
				break;
		case 3:	if(SectorObj[2] == 1){	//150-180
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ia;
					}
				}else{					//120-150
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ib;
					}
				}
				break;
		case 4:	if(SectorObj[2] == 1){	//180-210
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ia;
					}
				}else{					//210-240
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ic;
					}
				}
				break;
		case 5:	if(SectorObj[2] == 1){	//270-300
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ib;
					}
				}else{					//240-270
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ic;
					}
				}
				break;
		case 6:	if(SectorObj[2] == 1){	//300-330
					if(Ic > 0.5f || Ic < -0.5f){
						Ts_DFLOAT = (float)(-FULL_PWM) * 1 * Ts / Ib;
					}
				}else{					//330-360
					if(Ia > 0.5f || Ia < -0.5f){
						Ts_DFLOAT = (float)FULL_PWM * 1 * Ts / Ia;
					}
				}
				break;
	}
	if(Ts_DFLOAT > FULL_PWM * Ta * 0.25){				//	这里要注意，千万不能超出范围，超出了炸管子
		Ts_DFLOAT = FULL_PWM * Ta * 0.25;
	}else if(Ts_DFLOAT < -FULL_PWM * Ta * 0.25){
		Ts_DFLOAT = -FULL_PWM * Ta * 0.25;
	}
	/* EONVC */


	D1FLOAT = FULL_PWM*Ta*0.5 + Ts_DFLOAT;D1 = (unsigned int)D1FLOAT;//Min
	D2FLOAT = FULL_PWM*Tb+D1;D2 = (unsigned int)D2FLOAT;//Middle
	D3FLOAT = FULL_PWM*Tc+D2;D3 = (unsigned int)D3FLOAT;//Max

	switch(SectorObj[0]){
		case 1:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//oon ooo poo ppo poo ooo oon
							PWM[0] = 0x8000 + D2;
							PWM[1] = 0x8000 + D3;
							PWM[2] = D1;
/*							PWM[0] = D2;
							PWM[1] = 0;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }else{			//onn oon ooo poo ooo oon onn
						    PWM[0] = 0x8000 + D3;
						    PWM[1] = D1;
						    PWM[2] = D2;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/
					   }break;
				case 2:if(SectorObj[2]==1){//oon pon poo ppo poo pon oon
							PWM[0] = 0x8000 + D1;
							PWM[1] = 0x8000 + D3;
							PWM[2] = D2;
/*							PWM[0] = D1;
							PWM[1] = 0;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/
					   }else{			//onn oon pon poo pon oon onn
						    PWM[0] = 0x8000 + D2;
						    PWM[1] = D1;
						    PWM[2] = D3;
/*							PWM[0] = D2;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/
					   }break;
				case 3:		PWM[0] = 0x8000 + D1;
							PWM[1] = 0x8000 + D2;
							PWM[2] = D3;
/*							PWM[0] = D1;//oon pon ppn ppo ppn pon oon
							PWM[1] = 0;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/break;
				case 4:		PWM[0] = 0x8000 + D1;
							PWM[1] = D2;
							PWM[2] = D3;
/*							PWM[0] = D1;//onn pnn pon poo pon pnn onn
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/break;
				}break;
		case 2:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//oon ooo opo ppo opo ooo oon
							PWM[0] = 0x8000 + D3;
							PWM[1] = 0x8000 + D2;
							PWM[2] = D1;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }else{				//non oon ooo opo ooo oon non
						    PWM[0] = D1;
						    PWM[1] = 0x8000 + D3;
						    PWM[2] = D2;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/
					   }break;
				case 2:if(SectorObj[2]==1){//oon opn opo ppo opo opn oon
							PWM[0] = 0x8000 + D3;
							PWM[1] = 0x8000 + D1;
							PWM[2] = D2;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/
					   }else{				//non oon opn opo opn oon non
						    PWM[0] = D1;
						    PWM[1] = 0x8000 + D2;
						    PWM[2] = D3;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/
					   }break;
				case 3:		PWM[0] = 0x8000 + D2;
							PWM[1] = 0x8000 + D1;
							PWM[2] = D3;
/*							PWM[0] = D2;//oon opn ppn ppo ppn opn oon
							PWM[1] = 0;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/break;
				case 4:		PWM[0] = D2;
							PWM[1] = 0x8000 + D1;
							PWM[2] = D3;
/*							PWM[0] = EPWM_TIMER_TBPRD;//non npn opn opo opn npn non
							PWM[1] = D2;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D3;*/break;
				}break;
		case 3:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//noo ooo opo opp opo ooo noo
							PWM[0] = D1;
							PWM[1] = 0x8000 + D2;
							PWM[2] = 0x8000 + D3;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }else{			//non noo ooo opo ooo noo non
						    PWM[0] = D2;
						    PWM[1] = 0x8000 + D3;
						    PWM[2] = D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D2;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }break;
				case 2:if(SectorObj[2]==1){//noo npo opo opp opo npo noo
							PWM[0] = D2;
							PWM[1] = 0x8000 + D1;
							PWM[2] = 0x8000 + D3;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D2;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }else{			//non noo npo opo npo noo non
						    PWM[0] = D3;
						    PWM[1] = 0x8000 + D2;
						    PWM[2] = D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D3;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }break;
				case 3:		PWM[0] = D3;
							PWM[1] = 0x8000 + D1;
							PWM[2] = 0x8000 + D2;
/*							PWM[0] = EPWM_TIMER_TBPRD;//noo npo npp opp npp npo noo
							PWM[1] = D3;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = D2;
							PWM[5] = 0;*/break;
				case 4:		PWM[0] = D3;
							PWM[1] = 0x8000 + D1;
							PWM[2] = D2;
/*							PWM[0] = EPWM_TIMER_TBPRD;//non npn npo opo npo npn non
							PWM[1] = D3;
							PWM[2] = D1;
							PWM[3] = 0;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/break;
				}break;
		case 4:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//noo ooo oop opp oop ooo noo
							PWM[0] = D1;
							PWM[1] = 0x8000 + D3;
							PWM[2] = 0x8000 + D2;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = D2;
							PWM[5] = 0;*/
					   }else{			//nno noo ooo oop ooo noo nno
						    PWM[0] = D2;
						    PWM[1] = D1;
						    PWM[2] = 0x8000 + D3;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D2;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }break;
				case 2:if(SectorObj[2]==1){//noo nop oop opp oop nop noo
							PWM[0] = D2;
							PWM[1] = 0x8000 + D3;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D2;
							PWM[2] = D3;
							PWM[3] = 0;
							PWM[4] = D1;
							PWM[5] = 0;*/
					   }else{			//nno noo nop oop nop noo nno
						    PWM[0] = D3;
						    PWM[1] = D1;
						    PWM[2] = 0x8000 + D2;
/*							PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D3;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = D2;
							PWM[5] = 0;*/
					   }break;
				case 3:		PWM[0] = D3;
							PWM[1] = 0x8000 + D2;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;//noo nop npp opp npp nop noo
							PWM[1] = D3;
							PWM[2] = D2;
							PWM[3] = 0;
							PWM[4] = D1;
							PWM[5] = 0;*/break;
				case 4:		PWM[0] = D3;
							PWM[1] = D2;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;//nno nnp nop oop nop nnp nno
							PWM[1] = D3;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = D1;
							PWM[5] = 0;*/break;
				}break;
		case 5:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//ono ooo oop pop oop ooo ono
							PWM[0] = 0x8000 + D3;
							PWM[1] = D1;
							PWM[2] = 0x8000 + D2;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = D2;
							PWM[5] = 0;*/
					   }else{			//nno ono ooo oop ooo ono nno
							PWM[0] = D1;
							PWM[1] = D2;
							PWM[2] = 0x8000 + D3;
/*						    PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }break;
				case 2:if(SectorObj[2]==1){//ono onp oop pop oop onp oon
							PWM[0] = 0x8000 + D3;
							PWM[1] = D2;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = D1;
							PWM[5] = 0;*/
					   }else{			//nno ono onp oop onp ono nno
							PWM[0] = D1;
							PWM[1] = D3;
							PWM[2] = 0x8000 + D2;
/*						    PWM[0] = EPWM_TIMER_TBPRD;
							PWM[1] = D1;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = D2;
							PWM[5] = 0;*/
					   }break;
				case 3:		PWM[0] = 0x8000 + D2;
							PWM[1] = D3;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = D2;//ono onp pnp pop pnp onp ono
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = D1;
							PWM[5] = 0;*/break;
				case 4:		PWM[0] = D2;
							PWM[1] = D3;
							PWM[2] = 0x8000 + D1;
/*							PWM[0] = EPWM_TIMER_TBPRD;//nno nnp onp oop onp nnp nno
							PWM[1] = D2;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = D1;
							PWM[5] = 0;*/break;
				}break;
		case 6:switch(SectorObj[1]){
				case 1:if(SectorObj[2]==1){//ono ooo poo pop poo ooo ono
							PWM[0] = 0x8000 + D2;
							PWM[1] = D1;
							PWM[2] = 0x8000 + D3;
/*							PWM[0] = D2;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D1;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }else{		//onn ono ooo poo ooo ono onn
						    PWM[0] = 0x8000 + D3;
						    PWM[1] = D2;
						    PWM[2] = D1;
/*							PWM[0] = D3;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }break;
				case 2:if(SectorObj[2]==1){//ono pno poo pop poo pno ono
							PWM[0] = 0x8000 + D1;
							PWM[1] = D2;
							PWM[2] = 0x8000 + D3;
/*							PWM[0] = D1;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D2;
							PWM[4] = D3;
							PWM[5] = 0;*/
					   }else{		//onn ono pno poo pno ono onn
						    PWM[0] = 0x8000 + D2;
						    PWM[1] = D3;
						    PWM[2] = D1;
/*							PWM[0] = D2;
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D1;*/
					   }break;
				case 3:		PWM[0] = 0x8000 + D1;
							PWM[1] = D3;
							PWM[2] = 0x8000 + D2;
/*							PWM[0] = D1;//ono pno pnp pop pnp pno ono
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = D2;
							PWM[5] = 0;*/break;
				case 4:		PWM[0] = 0x8000 + D1;
							PWM[1] = D3;
							PWM[2] = D2;
/*							PWM[0] = D1;//onn pnn pno poo pno pnn onn
							PWM[1] = 0;
							PWM[2] = EPWM_TIMER_TBPRD;
							PWM[3] = D3;
							PWM[4] = EPWM_TIMER_TBPRD;
							PWM[5] = D2;*/break;
				}break;
	}
}

static void SVPWMCalcuSector(unsigned char *Sector){
	unsigned char A,B,C,T;
	float tUrefa;
	A = B = C = 0;
	if(Urefb>0){A = 1;}
	if((SQRT_3*Urefa-Urefb)>0){B = 1;}
	if((SQRT_3*Urefa+Urefb)<0){C = 1;}
	T = A+2*B+4*C;
	switch(T){
		case 1:Sector[0]=2;break;
		case 2:Sector[0]=6;break;
		case 3:Sector[0]=1;break;
		case 4:Sector[0]=4;break;
		case 5:Sector[0]=3;break;
		case 6:Sector[0]=5;break;
		default:Sector[0]=1;break;
	}
	tUrefa = Urefa;
	switch(*Sector){
		case 1:break;
		case 2:Urefa=-0.5*Urefa+HALF_SQRT_3*Urefb;Urefb=HALF_SQRT_3*tUrefa+0.5*Urefb;break;
		case 3:Urefa=-0.5*Urefa+HALF_SQRT_3*Urefb;Urefb=-HALF_SQRT_3*tUrefa-0.5*Urefb;break;
		case 4:Urefa=-0.5*Urefa-HALF_SQRT_3*Urefb;Urefb=-HALF_SQRT_3*tUrefa+0.5*Urefb;break;
		case 5:Urefa=-0.5*Urefa-HALF_SQRT_3*Urefb;Urefb=HALF_SQRT_3*tUrefa-0.5*Urefb;break;
		case 6:Urefb=-Urefb;break;
	}
	Sector[1] = Sector[2] = 0;
	/*Urefa-b ranges from 0 to 2*/
	if(Urefb<=-SQRT_3*(Urefa-1)){
		if(Urefb>=ONE_THIRD_SQRT_3*Urefa){Sector[1] = 1;Sector[2] = 1;}//A1, above 30degree
		else{Sector[1] = 1;Sector[2] = 2;}//A2
	}else if(Urefb<=SQRT_3*(Urefa-1)){
		Sector[1] = 4;//D
	}else if(Urefb<=HALF_SQRT_3){
		if(Urefb>=ONE_THIRD_SQRT_3*Urefa){Sector[1] = 2;Sector[2] = 1;}//B1, above 30degree
		else{Sector[1] = 2;Sector[2] = 2;}//B2
	}else{		//Urefb > Udc/2
		Sector[1] = 3;//C
	}
}
void SVMInit(void){
	unsigned char i;
	for(i=0;i<SVM_ARG_L;i++){
		SVMArguments[i] = 0.0f;
	}
	REC_PWM[0] = 0;
	REC_PWM[1] = 0;
	REC_PWM[2] = 0;
	INV_PWM[0] = 0;
	INV_PWM[1] = 0;
	INV_PWM[2] = 0;
}

void  DQInit(DQ * v){
	v -> U = 0;
	v -> V = 0;
	v -> W = 0;
	v -> cosine_value = 0;
	v -> sine_value = 0;
	v -> D = 0;
	v -> Q = 0;
}

void  ABC2DQ(DQ * v){
	v->D=0.666666666666666667*(v->cosine_value*v->U+(-0.5*v->cosine_value+0.86602540378444*v->sine_value)*v->V+(-0.5*v->cosine_value-0.86602540378444*v->sine_value)*v->W);
	v->Q=0.666666666666666667*(- v->sine_value*v->U+(0.5*v->sine_value+0.86602540378444*v->cosine_value)*v->V+(0.5*v->sine_value-0.86602540378444*v->cosine_value)*v->W);
}

void  DQ2ABC(DQ * v){
    v->U=v->cosine_value*v->D-v->Q*v->sine_value;
	v->V=(-0.5*v->cosine_value+0.86602540378444*v->sine_value)*v->D+(0.5*v->sine_value+0.86602540378444*v->cosine_value)*v->Q;
	v->W=(-0.5*v->cosine_value-0.86602540378444*v->sine_value)*v->D+(0.5*v->sine_value-0.86602540378444*v->cosine_value)*v->Q;
}
/*
void  DQInit(void){
	DQ.Ua = 0.0f;
	DQ.Ub = 0.0f;
	DQ.Uc = 0.0f;
	DQ.Ia = 0.0f;
	DQ.Ib = 0.0f;
	DQ.Ic = 0.0f;
	DQ.Ud = 0.0f;
	DQ.Uq = 0.0f;
	DQ.Ud_back = 0.0f;
	DQ.Uq_back = 0.0f;
	DQ.Id = 0.0f;
	DQ.Iq = 0.0f;
	DQ.Id_back = 0.0f;
	DQ.Iq_back = 0.0f;
	DQ.Theta = 0.0f;
	DQ.Sin_Theta = 0.0f;
	DQ.Cos_Theta = 0.0f;
	DQ.IdqBufferPointer = 0;
	DQ.IdAvg = 0.0f;
	DQ.IqAvg = 0.0f;
}

void  DQStoreAndAvg(sDQ *v){
	unsigned char i;
	float sum;
	v->IdBuffer[v->IdqBufferPointer] = v->Id;
	v->IqBuffer[v->IdqBufferPointer] = v->Iq;
	v->IdqBufferPointer++;
	if(v->IdqBufferPointer >= DQ_BUFFER_LENGTH){v->IdqBufferPointer = 0;}
	sum = 0.0f;
	for(i=0;i<DQ_BUFFER_LENGTH;i++){
		sum += v->IdBuffer[i];
	}
	v->IdAvg = sum/DQ_BUFFER_LENGTH;
	sum = 0.0f;
	for(i=0;i<DQ_BUFFER_LENGTH;i++){
		sum += v->IqBuffer[i];
	}
	v->IqAvg = sum/DQ_BUFFER_LENGTH;
}
*/
