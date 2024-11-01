/*
 * LPF.c
 *
 *  Created on: Aug 30, 2024
 *      Author: RMG
 */

#include "LPF.h"

void LPF_EMA_Init(LPF_EMA *filt, float alpha){

	//Set filter coefficient

	LPF_EMA_SetAlpha(filt, alpha);

	//Clear filter output

	filt->output = 0.0f;
}

void LPF_EMA_SetAlpha(LPF_EMA *filt, float alpha){

	//Correct filter

	if(alpha > 1.0f){
		alpha = 1.0f;

	}else if (alpha < 0.0f){
		alpha = 0.0f;
	}

	//Set filter coefficient

	filt->alpha = alpha;
}

float LPF_EMA_Update(LPF_EMA *filt, float in){

	filt->output = filt->alpha*in + (1-filt->alpha)*filt->output;

	return filt->output;
}



