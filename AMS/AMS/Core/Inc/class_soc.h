#ifndef CLASS_SOC_H
#define CLASS_SOC_H

#include <stdint.h>
#include "class_bms.h"

class SOC_MOD {
public:
	SOC_MOD(float fullCapacity_Ah, float Vmin_cell, float Vmax_cell, int numCells);

	void update(uint32_t time_ms, BMS_MOD *bms_array, int bms_count, float packCurrent_A);
	void updateFromLSTM(float input_seq[50][8]);  // NUEVO MÉTODO
	float getSOC();        // 0.0 - 1.0
	int getSOC_percent();  // 0 - 100 %

private:
	float soc;
	float capacity_Ah;
	float Vmin, Vmax;
	int totalCells;

	uint32_t lastUpdateTime_ms;
	bool initialized;

	float estimateFromVoltage(BMS_MOD *bms_array, int bms_count);
};

#endif
