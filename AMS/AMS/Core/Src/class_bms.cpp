// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_bms.h
// Description  :
// * This file is for defining the BMS class
// -----------------------------------------------------------------------------

#include <class_bms.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_bms.h"

///////// Dont ever touch this function if BMS are still ZEVA

/*********************************************************************************************************
 ** Function name:           BMS_MOD
 ** Descriptions:            Initialization function of teh class
 *********************************************************************************************************/

BMS_MOD::BMS_MOD(uint32_t _ID, int _MAXV, int _MINV, int _MAXT,
		uint8_t _NUMCELLS, unsigned int _SHUNT, int _LAG_V, int _LAG_T) {
	CANID = _ID;
	LIMIT_MAX_V = _MAXV;
	LIMIT_MIN_V = _MINV;
	LIMIT_MAX_T = _MAXT;
	NUM_CELLS = _NUMCELLS;
	time_lim_plotted_volts += _LAG_V;
	time_lim_plotted_temps += _LAG_T;

	time_lim_sent_volts += _LAG_V;
	time_lim_sent_temps += _LAG_T;

	time_lim_received_volts += _LAG_V;
	time_lim_received_temps += _LAG_T;

}

/*********************************************************************************************************
 ** Function name:           info
 ** Descriptions:            Function for printing the class data
 *********************************************************************************************************/
void BMS_MOD::voltage_info(char *buffer) {

	if (getUARTState() == HAL_UART_STATE_READY) { // Send the message just if there is a serial por connected
		print((char*) "\n***********************");
		print((char*) "         BMS");
		print((char*) "***********************");
		sprintf(buffer, " - ERROR:     %i", error);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx", CANID);
		print(buffer);
		sprintf(buffer, " - MAX V =    %i mV", MAX_V);
		print(buffer);
		sprintf(buffer, " - MIN V =    %i mV", MIN_V);
		print(buffer);
		print((char*) "-----------------------");
		sprintf(buffer, "VOLTS (mV): [%i", cellVoltagemV[0]);
		printnl(buffer);
		for (int i = 0; i < NUM_CELLS; i++) {
			sprintf(buffer, ", %i", cellVoltagemV[i]);
			printnl(buffer);
		}

		for (int i = 0; i < NUM_CELLS; i++) {
			voltage_acum += cellVoltagemV[i];
		}

		print((char*) "]");
		sprintf(buffer, " - V(max) = %i mV || V(min) = %i", MAX_V, MIN_V);
		print(buffer);
	}
	sprintf(buffer, "- BALANCING V = %i mV", BALANCING_V);
	print(buffer);

	sprintf(buffer, "- STACK VOLTAGE = %i V", voltage_acum / 1000);
	print(buffer);
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void BMS_MOD::temperature_info(char *buffer) {
	if (getUARTState() == HAL_UART_STATE_READY) //Send the message just if there is a serial por connected
	{
		print((char*) "\n***********************");
		print((char*) "     Temperatures");
		print((char*) "***********************");
		sprintf(buffer, " - ERROR:     %i", error);
		print(buffer);
		sprintf(buffer, " - CAN ID:    0x%lx", CANID+20);
		print(buffer);
		sprintf(buffer, " - MAX T =    %i ºC", MAX_T);
		print(buffer);
		sprintf(buffer, " - MIN T =    %i ºC", MIN_T);
		print(buffer);
		sprintf(buffer, " - LIM T =    %i ºC", LIMIT_MAX_T);
		print((char*) "-----------------------");
		sprintf(buffer, "Temperatures (ºC): [%i", cellTemperature[0]);
		printnl(buffer);
		for (int i = 1; i < 38; i++) {
			sprintf(buffer, ", %i", cellTemperature[i]);
			printnl(buffer);
		}
		print((char*) "]");

	}
}

/*********************************************************************************************************
 ** Function name:           parse
 ** Descriptions:            Function for parsing the received data via CAN protocol
 *********************************************************************************************************/
bool BMS_MOD::parse(uint32_t id, uint8_t *buf, uint32_t t) {
	if (id > CANID && id < CANID + 30) {
		int m = id % CANID;
		int pos = 0;
		if (m >= 1 && m <= 5) {
			time_lim_received_volts = t + TIME_LIM_RECV_VOLTS;

			for (int i = 0; i < 4; i++) {
				pos = (m - 1) * 4 + i;
				if (pos >= 19)
					break;

				cellVoltagemV[pos] = (buf[2 * i] << 8) | buf[2 * i + 1];

				if ((cellVoltagemV[pos] > LIMIT_MAX_V
						|| cellVoltagemV[pos] < LIMIT_MIN_V)
						&& pos < NUM_CELLS) {
					flag_error_volt[pos]++;
					if (flag_error_volt[pos] >= max_flag)
						error = BMS_ERROR_VOLTS;
				} else {
					flag_error_volt[pos] = 0;
				}
			}

			MAX_V = cellVoltagemV[0];
			MIN_V = cellVoltagemV[0];
			for (int i = 1; i < 19; i++) {
				if (cellVoltagemV[i] > MAX_V)
					MAX_V = cellVoltagemV[i];
				else if (cellVoltagemV[i] < MIN_V)
					MIN_V = cellVoltagemV[i];
			}

			return true;

		} else if (m >= 21 && m <= 25) {
			// m = 21 → packet 0, m = 25 → packet 4
			time_lim_received_temps = t + TIME_LIM_RECV_TEMPS;
			if (flag_charger == 1)
				module_send_message_CAN1(id, buf, 8);

			for (int i = 0; i < 8; i++) {
				pos = (m - 21) * 8 + i;
				if (pos >= 38)
					break;

				cellTemperature[pos] = buf[i];
				if (cellTemperature[pos] > LIMIT_MAX_T)
					error = BMS_ERROR_TEMP;
			}

			if (m == 25) {
				MAX_T = cellTemperature[0];
				MIN_T = cellTemperature[0];
				for (int i = 1; i < 38; i++) {
					if (cellTemperature[i] > MAX_T)
						MAX_T = cellTemperature[i];
					else if (cellTemperature[i] < MIN_T
							&& cellTemperature[i] != 0)
						MIN_T = cellTemperature[i];
				}
			}


			return true;
		}

	}

	return false;
}

/*********************************************************************************************************
 ** Function name:           return_error
 ** Descriptions:            Function for returning the state of the BMS
 *********************************************************************************************************/
int BMS_MOD::return_error() {
	return error;
}

/*********************************************************************************************************
 ** Function name:           query_voltage
 ** Descriptions:            Function to check if i need to send a message new message and the received messages interval are within the limits
 *********************************************************************************************************/
int BMS_MOD::query_voltage(uint32_t time, char *buffer) {
	// Shunt voltage in milivolts

	message_balancing[1] = BALANCING_V & 0xFF; // Coment this two lines for disabling the balancing
	message_balancing[0] = (BALANCING_V >> 8) & 0xFF; // Coment this two lines for disabling the balancing

	if (time > time_lim_sent_volts) {
		time_lim_sent_volts += TIME_LIM_SEND_VOLTS;
		if (CANID != 0x00) { //It keeps sending 0x00 and dont know where
			if (module_send_message_CAN2(CANID, message_balancing, 2)
					!= HAL_OK) {
				error = BMS_ERROR_COMMUNICATION;
			}
		}

	}

	if (time > time_lim_received_volts) {
		error = BMS_ERROR_COMMUNICATION;
	}

	if (TIME_LIM_PLOT_VOLTS > 0 && time > time_lim_plotted_volts) {
		time_lim_plotted_volts += TIME_LIM_PLOT_VOLTS;
		voltage_info(buffer);
	}

	for (int i = 0; i < NUM_CELLS; i++) {
		voltage_acum += cellVoltagemV[i];
	}
	return error;
}

/*********************************************************************************************************
 ** Function name:           query_temperature
 ** Descriptions:            Function to check if i need to send a message new message and the received messages interval are within the limits
 *********************************************************************************************************/

int BMS_MOD::query_temperature(uint32_t time, char *buffer) {
// Function for performing a correct behavior
	if (time > time_lim_sent_temps) { // HERE I HAVE TO SEND THE REQUEST MESSAGE FOR THE TEMPERATURES
		time_lim_sent_temps += TIME_LIM_SEND_TEMPS;

		if (module_send_message_CAN2(CANID + 20, message_temperatures, 2)
				!= HAL_OK) {
			error = BMS_ERROR_TEMP; // If the message is not sended then, error
		}
	}
	if (time > time_lim_received_temps) {
		error = BMS_ERROR_COMMUNICATION;
	}
	if (TIME_LIM_PLOT_TEMPS > 0 && time > time_lim_plotted_temps) {
		time_lim_plotted_temps += TIME_LIM_PLOT_TEMPS;
		//temperature_info(buffer);
	}



	return error;

}

