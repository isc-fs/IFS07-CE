#include "class_soc.h"
#include "lstm_model_interface.h"  

SOC_MOD::SOC_MOD(float fullCapacity_Ah, float Vmin_cell, float Vmax_cell, int numCells) {
	capacity_Ah = fullCapacity_Ah;
	Vmin = Vmin_cell;
	Vmax = Vmax_cell;
	totalCells = numCells;
	soc = 1.0;
	lastUpdateTime_ms = 0;
	initialized = false;
}

float SOC_MOD::getSOC() {
	if (soc > 1.0f) soc = 1.0f;
	if (soc < 0.0f) soc = 0.0f;
	return soc;
}

int SOC_MOD::getSOC_percent() {
	return (int)(getSOC() * 100.0f);
}

void SOC_MOD::update(uint32_t time_ms, BMS_MOD *bms_array, int bms_count, float packCurrent_A) {
	if (!initialized) {
		lastUpdateTime_ms = time_ms;
		soc = estimateFromVoltage(bms_array, bms_count);
		initialized = true;
		return;
	}

	float delta_h = (time_ms - lastUpdateTime_ms) / 3600000.0f;
	lastUpdateTime_ms = time_ms;

	float deltaSOC = -(packCurrent_A * delta_h) / capacity_Ah;
	soc += deltaSOC;
}

void SOC_MOD::updateFromLSTM(float input_seq[50][8]) {
	soc = SOC_LSTM_Evaluate(input_seq); 
}

float SOC_MOD::estimateFromVoltage(BMS_MOD *bms_array, int bms_count) {
	const int N = 22;
	const float voltages[N] = {
		3.00, 3.20, 3.25, 3.30, 3.35, 3.40, 3.45, 3.50, 3.55, 3.60, 3.65,
		3.70, 3.75, 3.80, 3.85, 3.90, 3.95, 4.00, 4.05, 4.10, 4.15, 4.20
	};
	const float socs[N] = {
		0.00, 0.02, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
		0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00
	};

	float v_sum = 0.0f;
	int cell_count = 0;

	for (int i = 0; i < bms_count; i++) {
		for (int j = 0; j < bms_array[i].NUM_CELLS; j++) {
			int v_mV = bms_array[i].cellVoltagemV[j];
			if (v_mV > 1000 && v_mV < 5000) {
				float v = v_mV / 1000.0f;
				v_sum += v;
				cell_count++;
			}
		}
	}

	if (cell_count == 0) return 0.0f;
	float v_avg = v_sum / cell_count;

	for (int i = 1; i < N; i++) {
		if (v_avg <= voltages[i]) {
			float dv = voltages[i] - voltages[i - 1];
			float dsoc = socs[i] - socs[i - 1];
			return socs[i - 1] + (v_avg - voltages[i - 1]) * dsoc / dv;
		}
	}

	return 1.0f;
}
