/*
 * SOGI_PLL.h
 *
 *  Created on: 2014-7-11
 *      Author: appleseed
 */

#ifndef SOGI_PLL_H_
#define SOGI_PLL_H_

#include "Global.h"

#define	SOGI_K					1
#define	SOGI_FREQ				50.0f
#define	SOGI_TS					0.0001f
#define SOGI_B0					0.015461283f
#define SOGI_A1					1.968105973f
#define SOGI_A2					-0.969077433f
#define SOGI_B1					0.062831853f
#define SOGI_ONE_THIRD_SQRT_3	0.577350269f
#define SOGI_2PIF				314.1592654f
#define SOGI_PI					3.141592654f
#define SOGI_2PI				6.283185307f
#define SOGI_HALF_PI			1.570796327f

#define SOGI_OMEGA_MAX			326.0f
#define SOGI_OMEGA_MIN			302.0f


typedef struct{
	sPIDParams	QPI;
	float U;
	float V;
	float W;
	float Ud;
	float Uq;
	float theta;
	float theta_cos;
	float theta_sin;
	float Omega;
	float GridFrequency;
}SOGIPLLStruct;

extern SOGIPLLStruct SOGIPLLData;

extern void SOGI_PLL_Init(SOGIPLLStruct *pSOGIPLL);
extern void SOGI_PLL_Update(SOGIPLLStruct *pSOGIPLL);

#endif /* SOGI_PLL_H_ */
