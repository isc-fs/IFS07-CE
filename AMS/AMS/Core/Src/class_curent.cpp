// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_current.h
// Description  :
// * This file is for declaring the functions and variables of the class of the current
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_current.h"

extern ADC_HandleTypeDef hadc1;

int readCurrent() {

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	float adc_value, V_adc;
	int current;

	adc_value = HAL_ADC_GetValue(&hadc1);
	V_adc = (adc_value * 5 / 4095); //12 bits de resolución

	current = (3020 - adc_value) * 0.14286;

	HAL_ADC_Stop(&hadc1);
	return current;
}

// ********************************************************************************************************
// **Function name:           Current_MOD
// **Descriptions:            Initialization function of current class
// **********************************************************************************************************
Current_MOD::Current_MOD(uint32_t ID, int _C_MAX) {
	CANID = ID;
	C_MAX = _C_MAX;
	flag_error_current = 1;
}

// ********************************************************************************************************
// ** Function name:           query
// ** Descriptions:            Function that transforms the voltage measured by the sensor to its equivalent current
// **********************************************************************************************************
int Current_MOD::query(int time, char *buffer) {
	error = Current_OK;

	Current = readCurrent();

	if (Current > C_MAX * 0.8 && Current < C_MAX) {
		if (flag_error_current == 0)
			module_send_message_NoExtId_CAN1(0x500, message, 1); //If current between 80 and 100% of maximun, sends alert
	}

	if (Current > C_MAX) {
		if (flag_error_current == 1) {
			module_send_message_NoExtId_CAN1(0x501, message, 2); //If current over maximun, sends alert
		}

		flag_error_current++;

		if (flag_charger != 1 || flag_charger == 1) //Only cut the AMS if accu not connected to charger, during charging the current control is on charger
				{
			if (flag_error_current >= 100) //I need to pass the maximun current for more than 100 times for cutting, maybe it was EMI
					{
				//error = Current_ERROR_MAXIMUN_C;
				//print((char*)"MAXIMA Corriente");
			}
		}
	} else {
		if (flag_error_current != 0)
			for (int i = 0; i < 5; i++) {
				module_send_message_NoExtId_CAN1(0x502, 0, 2); //If current normal, sends green flag
			}
		flag_error_current = 0;
	}

	if (time > time_lim_sended) {
		time_lim_sended += TIME_LIM_SEND;
		message[0] = 0;
		message[1] = Current & 0xFF;
		module_send_message_NoExtId_CAN1(CANID, message, 2); //Sends current through CAN each interval of ms
	}

	if (TIME_LIM_PLOT > 0 && time > time_lim_plotted) {
		time_lim_plotted += TIME_LIM_PLOT;
		//info(buffer);

	}

	return error;
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void Current_MOD::info(char *buffer) {
	Current = 0;
	if (getUARTState() == HAL_UART_STATE_READY) { // Send the message just if there is a serial por connected
		sprintf(buffer, "\n***********************\n");
		print(buffer);
		sprintf(buffer, "         Current\n");
		print(buffer);
		sprintf(buffer, "***********************\n");
		print(buffer);
		sprintf(buffer, " - ERROR:     %i\n", error);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx\n", CANID);
		print(buffer);
		sprintf(buffer, " - LIM C =    %i A\n", C_MAX);
		print(buffer);
		sprintf(buffer, "-----------------------\n");
		print(buffer);
		sprintf(buffer, "Current (A): %i\n", Current);
		print(buffer);

	}
}

