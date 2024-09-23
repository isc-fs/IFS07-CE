/*
 * LPF.h
 *
 *  Created on: Aug 30, 2024
 *      Author: RMG
 */

#ifndef INC_LPF_H_
#define INC_LPF_H_

typedef struct{
	/*Filter coeficcient*/

	float alpha;

	/*Filter output*/

	float output;
}LPF_EMA;

void LPF_EMA_Init(LPF_EMA *filt, float alpha);
void LPF_EMA_SetAlpha(LPF_EMA *filt, float alpha);
float LPF_EMA_Update(LPF_EMA *filt, float in);


#endif /* INC_LPF_H_ */
