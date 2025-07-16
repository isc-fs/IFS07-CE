// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_cpu.h
// Description  :
// * This file is for defining the BMS class
// -----------------------------------------------------------------------------

#ifndef INC_CLASS_BMS_H_
#define INC_CLASS_BMS_H_

#include <stdint.h>
#include "main.h"

#define BMS_OK                    0
#define Temperatures_OK 		  0
#define BMS_ERROR_COMMUNICATION   1
#define BMS_ERROR_VOLTS           2
#define BMS_ERROR_TEMP            3

class BMS_MOD {
private:
	uint32_t CANID = 0x12C;                          // The id of the CAN device
	int error = BMS_OK;                     // Variable for error handling
	int cellVoltagemV[19] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0 };  // Voltage in mV
	int cellTemperature[38] = { 0 }; // Temperatures
	int LIMIT_MAX_V = 0;
	int LIMIT_MIN_V = 0;
	int LIMIT_MAX_T = 0;
	int MAX_V = 0;

	uint32_t TIME_LIM_PLOT = 1000; // Interval of time for ploting BMS info in ms
	uint32_t TIME_LIM_SEND = 500;   // Interval to send mesage in ms
	uint32_t TIME_LIM_RECV = 3000;   // Limit time for communication in ms
	uint32_t time_lim_plotted = TIME_LIM_PLOT; // Dont plot in the initil time because the value is null
	uint32_t time_lim_sended = 0;      // When has been last sended
	uint32_t time_lim_received = TIME_LIM_RECV; // When have been last received // Initialized to 1000 to wait one minute to receive the data
	uint8_t message_balancing[2] = { 0, 0 }; // Voltage in mV
	uint8_t message_temperatures[2] = { 0, 0 };
	uint8_t NUM_CELLS = 19;
	int max_flag = 3;
	int flag_error_volt[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int flag_error_temp = 0;

public:
	int voltage_acum = 0;
	int MIN_V = 0;
	int BALANCING_V = 4200;
	int flag_charger = 0;
	int MIN_T = 0;
	int MAX_T = 0;

	BMS_MOD(uint32_t _ID, int _MAXV, int _MINV, int _MAXT, uint8_t _NUMCELLS,
			unsigned int _SHUNT, int _LAG = 0);
	void voltage_info(char *buffer);
	void temperature_info(char *buffer);
	bool parse(uint32_t id, uint8_t *buf, uint32_t t);
	int return_error();
	int query_voltage(uint32_t time, char *buffer);
	int query_temperature(uint32_t time, char *buffer);
	//HAL_StatusTypeDef module_send_message_CAN0(uint32_t id, uint8_t* data, uint8_t length);
	//HAL_StatusTypeDef module_send_message_CAN1(uint32_t id, uint8_t* data, uint8_t length);
};

#endif /* INC_CLASS_BMS_H_ */
