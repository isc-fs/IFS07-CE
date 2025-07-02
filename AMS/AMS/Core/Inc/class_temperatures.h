// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_temperatures.h
// Description  :
// * This file is for declaring the functions and variables of the class of the temperatures
// -----------------------------------------------------------------------------

#ifndef INC_CLASS_TEMPERATURES_H_
#define INC_CLASS_TEMPERATURES_H_


#define Temperatures_OK 0
#define Temperatures_ERROR_COMMUNICATION 1
#define Temperatures_ERROR_MAXIMUM_T 2

class Temperatures_MOD
{
private:
	int error = Temperatures_OK;     // Variable for error handling
	int LIMIT_MAX_T = 0;             // Maximum value of temperatures
	uint32_t MODULEID = 0;           // ID of each slave
	uint32_t CANIDSENDTEMPS = 0x500; // ID for telling the slaves to send temperatures
	uint32_t CANIDTEL = 0x400;

	int cellTemperature[38];
	int temp_bckp[38];

private:
	uint32_t TIME_LIM_PLOT = 1000;              // Interval of time for ploting BMS info in ms
	uint32_t TIME_LIM_SEND = 5000;              // Interval to send mesage in ms
	uint32_t TIME_LIM_RECV = 1000;              // Limit time for communication in ms
	uint32_t time_lim_plotted = TIME_LIM_PLOT;  // Dont plot in the initil time because the value is null
	uint32_t time_lim_sended = 0;               // When has been last sended
	uint32_t time_lim_received = TIME_LIM_RECV; // When have been last received // Initialized to 1000 to wait one minute to receive the data
	uint32_t time_sending = 0;
	uint8_t message_temperatures[2] = { 0, 0 };


public:
	int MIN_T = 0;
	int MAX_T = 0;
	int temperature_media;
	int flag_sending;
	int flag_charger = 0;
	Temperatures_MOD(uint32_t ID, int _T_MAX, int _LAG = 0);
	void info(char* buffer);
	int query(uint32_t time, char* buffer);
	bool parse(uint32_t id, uint8_t* buf, uint32_t t);
};


#endif /* INC_CLASS_TEMPERATURES_H_ */
