#include "soc_features.h"
#include "class_bms.h"
#include "class_current.h"

extern BMS_MOD BMS[];
extern int BMS_N;
extern Current_MOD current;

float get_avg_voltage_all_bms(void) {
    int sum = 0, count = 0;
    for (int i = 0; i < BMS_N; i++)
        for (int j = 0; j < BMS[i].NUM_CELLS; j++) {
            int v = BMS[i].cellVoltagemV[j];
            if (v > 1000 && v < 5000) {
                sum += v;
                count++;
            }
        }
    return (count > 0) ? (sum / 1000.0f / count) : 0.0f;
}

float get_min_voltage_all_bms(void) {
    int vmin = 5000;
    for (int i = 0; i < BMS_N; i++)
        for (int j = 0; j < BMS[i].NUM_CELLS; j++) {
            int v = BMS[i].cellVoltagemV[j];
            if (v > 1000 && v < vmin) vmin = v;
        }
    return vmin / 1000.0f;
}

float get_max_voltage_all_bms(void) {
    int vmax = 0;
    for (int i = 0; i < BMS_N; i++)
        for (int j = 0; j < BMS[i].NUM_CELLS; j++) {
            int v = BMS[i].cellVoltagemV[j];
            if (v > vmax && v < 5000) vmax = v;
        }
    return vmax / 1000.0f;
}

float get_avg_temperature_all_bms(void) {
    int sum = 0, count = 0;
    for (int i = 0; i < BMS_N; i++)
        for (int j = 0; j < 38; j++) {
            int t = BMS[i].cellTemperature[j];
            if (t >= 0 && t <= 120) {
                sum += t;
                count++;
            }
        }
    return (count > 0) ? (sum * 1.0f / count) : 0.0f;
}

// Variables internas para cálculo de derivadas
static float prev_current = 0.0f, prev_v = 0.0f, prev_t = 0.0f;
static int lstm_index = 0;

// Actualiza input_seq[50][8] en memoria, con desplazamiento si es necesario
void soc_lstm_update_buffer(float input_seq[50][8]) {
    float I = current.getCurrent();
    float Vavg = get_avg_voltage_all_bms();
    float Vmin = get_min_voltage_all_bms();
    float Vmax = get_max_voltage_all_bms();
    float Tavg = get_avg_temperature_all_bms();

    float dI = (I - prev_current) * 10.0f;
    float dV = (Vavg - prev_v) * 10.0f;
    float dT = (Tavg - prev_t) * 10.0f;

    prev_current = I;
    prev_v = Vavg;
    prev_t = Tavg;

    if (lstm_index >= 50) {
        for (int i = 0; i < 49; i++)
            for (int j = 0; j < 8; j++)
                input_seq[i][j] = input_seq[i + 1][j];
        lstm_index = 49;
    }

    input_seq[lstm_index][0] = I;
    input_seq[lstm_index][1] = Vavg;
    input_seq[lstm_index][2] = Vmin;
    input_seq[lstm_index][3] = Vmax;
    input_seq[lstm_index][4] = Tavg;
    input_seq[lstm_index][5] = dI;
    input_seq[lstm_index][6] = dV;
    input_seq[lstm_index][7] = dT;

    lstm_index++;
}
