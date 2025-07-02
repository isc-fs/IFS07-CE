// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_cpu.h
// Description  :
// * This file is for defining the class of the connection protocols with putside
// -----------------------------------------------------------------------------

#ifndef INC_CLASS_CPU_H_
#define INC_CLASS_CPU_H_


#define CPU_OK                    0
#define CPU_ERROR_COMMUNICATION   1
#define CPU_BUS_LINE_OK           2

#define CPU_POWER       0
#define CPU_PRECHARGE   1
#define CPU_DISCONNECTED 2
#define CPU_ERROR       3

class CPU_MOD {
private:
	uint32_t CANID_send = 0;
	uint32_t CANID_recv = 0;
	int error = CPU_ERROR_COMMUNICATION;
	uint8_t currentState[1] = { CPU_DISCONNECTED };
	int current_state = CPU_DISCONNECTED;

private:
	uint32_t TIME_LIM_PLOT = 1000;   // Intervalo de tiempo para plotear informaci�n en ms
	uint32_t TIME_LIM_SEND = 100;    // Intervalo para enviar mensajes en ms
	uint32_t TIME_LIM_RECV = 1000000; // L�mite de tiempo para la comunicaci�n en ms
	uint32_t time_lim_plotted = TIME_LIM_PLOT;
	uint32_t time_lim_sended = 0;
	uint32_t time_lim_received = TIME_LIM_RECV;

public:
	int voltage_acum = 0;
	int DC_BUS = 0;

	CPU_MOD(uint32_t _ID_send, uint32_t _ID_recv, int _LAG = 0);
	void info(char* buffer);
	int return_error();
	int query(uint32_t time, char* buffer);
	bool parse(uint32_t id, uint8_t* buf, uint32_t t);
	void updateState(int s);
	//HAL_StatusTypeDef module_send_message_CAN1(uint32_t id, uint8_t* data, uint8_t length);
};

#endif /* INC_CLASS_CPU_H_ */
