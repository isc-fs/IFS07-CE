#ifndef SOC_FEATURES_H
#define SOC_FEATURES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Requiere acceso al array BMS[]
extern float get_avg_voltage_all_bms(void);
extern float get_min_voltage_all_bms(void);
extern float get_max_voltage_all_bms(void);
extern float get_avg_temperature_all_bms(void);

// Buffer LSTM de 50x8
void soc_lstm_update_buffer(float input_seq[50][8]);

#ifdef __cplusplus
}
#endif

#endif
