/*	SOGI_PLL
	Second Order Generalized Integrator Phase Lock Loop
	Author:	HuangruZhu
	Date:	7/10/2014
*/
#include "SOGI_PLL.h"
#include "F28335BSP.h"

#pragma CODE_SECTION(SOGI_PLL_Update, "ramfuncs");

float alpha_x[3];
float alpha_y_filter[3];			//alpha轴 滤波、不滞后
float alpha_y_lag[3];				//alpha轴滤波+90度滞后
float beta_x[3];
float beta_y_filter[3];				//beta轴 滤波、不滞后
float beta_y_lag[3];				//beta轴滤波+90度滞后
float Ualpha;
float Ubeta;
float theta_cos;
float theta_sin;

SOGIPLLStruct SOGIPLLData;

void SOGI_PLL_Init(SOGIPLLStruct *pSOGIPLL){
	IncPIDControlInit(&(pSOGIPLL->QPI));
	pSOGIPLL->QPI.ProportionalGain = 0.1f;
	pSOGIPLL->QPI.IntergralGain = 0.05f;
	pSOGIPLL->QPI.DesiredValue = 0.0f;
	pSOGIPLL->theta = 0.0f;
	pSOGIPLL->Omega = SOGI_2PIF;

	alpha_x[0] = alpha_x[1] = alpha_x[2] = 0;
	alpha_y_filter[0] = alpha_y_filter[1] = alpha_y_filter[2] = 0;
	alpha_y_lag[0] = alpha_y_lag[1] = alpha_y_lag[2] = 0;
	beta_x[0] = beta_x[1] = beta_x[2] = 0;
	beta_y_filter[0] = beta_y_filter[1] = beta_y_filter[2] = 0;
	beta_y_lag[0] = beta_y_lag[1] = beta_y_lag[2];
}

void SOGI_PLL_Update(SOGIPLLStruct *pSOGIPLL){
	Ualpha = (pSOGIPLL->U - 0.5 * pSOGIPLL->V - 0.5 * pSOGIPLL->W) * 0.666666666666666f;
	Ubeta = (pSOGIPLL->V - pSOGIPLL->W) * SOGI_ONE_THIRD_SQRT_3; 
	
	//alpha SOGI
	alpha_x[0] = Ualpha;
	alpha_y_filter[0] = SOGI_B0 * alpha_x[0] 
					- SOGI_B0 * alpha_x[2] 
					+ SOGI_A1 * alpha_y_filter[1]
					+ SOGI_A2 * alpha_y_filter[2];
	alpha_y_filter[2] = alpha_y_filter[1];
	alpha_y_filter[1] = alpha_y_filter[0];

	alpha_y_lag[0] = SOGI_B1*SOGI_B0*alpha_x[1] 
					+ SOGI_A1 * alpha_y_lag[1] 
					+ SOGI_A2 * alpha_y_lag[2];
	alpha_y_lag[2] = alpha_y_lag[1];
	alpha_y_lag[1] = alpha_y_lag[0];
	
	alpha_x[2] = alpha_x[1];
	alpha_x[1] = alpha_x[0];

	//beta SOGI
	beta_x[0] = Ubeta;
	beta_y_filter[0] = SOGI_B0 * beta_x[0] 
					- SOGI_B0 * beta_x[2] 
					+ SOGI_A1 * beta_y_filter[1]
					+ SOGI_A2 * beta_y_filter[2];
	beta_y_filter[2] = beta_y_filter[1];
	beta_y_filter[1] = beta_y_filter[0];

	beta_y_lag[0] = SOGI_B1*SOGI_B0*beta_x[1] 
					+ SOGI_A1 * beta_y_lag[1] 
					+ SOGI_A2 * beta_y_lag[2];
	beta_y_lag[2] = beta_y_lag[1];
	beta_y_lag[1] = beta_y_lag[0];
	
	beta_x[2] = beta_x[1];
	beta_x[1] = beta_x[0];
	
	Ualpha = 0.5 * (alpha_y_filter[0] - beta_y_lag[0]);
	Ubeta = 0.5 * (alpha_y_lag[0] + beta_y_filter[0]);
	
	pSOGIPLL->theta += (pSOGIPLL->Omega) * 0.0001f;		//这句话需要先于PI算法，放在后面会有相位延迟。

	if(pSOGIPLL->theta > SOGI_PI){
		pSOGIPLL->theta -= SOGI_2PI;
	}
	if(pSOGIPLL->theta < -SOGI_PI){
		pSOGIPLL->theta += SOGI_2PI;
	}

	theta_cos = cos(pSOGIPLL->theta);
	theta_sin = sin(pSOGIPLL->theta);
	pSOGIPLL->Ud = Ualpha * theta_cos + Ubeta * theta_sin;
	pSOGIPLL->Uq = - Ualpha * theta_sin + Ubeta * theta_cos;
	
	pSOGIPLL->QPI.MeasuredValue = - pSOGIPLL->Uq;
	pSOGIPLL->Omega += IncPIControl(&(pSOGIPLL->QPI));
	if(pSOGIPLL->Omega > SOGI_OMEGA_MAX){pSOGIPLL->Omega = SOGI_OMEGA_MAX;}
	if(pSOGIPLL->Omega < SOGI_OMEGA_MIN){pSOGIPLL->Omega = SOGI_OMEGA_MIN;}
	
	if((pSOGIPLL->theta > -SOGI_HALF_PI) && (pSOGIPLL->theta < SOGI_HALF_PI)){	//zero cross
		LED_GPIO30 = 1;
	}else{
		LED_GPIO30 = 0;
	}
	
	pSOGIPLL->GridFrequency = pSOGIPLL->Omega * 0.159154943;
	pSOGIPLL->theta_cos = theta_cos;
	pSOGIPLL->theta_sin = theta_sin;
}
