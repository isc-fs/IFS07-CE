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


// ********************************************************************************************************
// **Function name:           Current_MOD
// **Descriptions:            Initialization function of current class
// **********************************************************************************************************
Current_MOD::Current_MOD(uint32_t ID, int _C_MAX)
{
    CANID = ID;
    C_MAX = _C_MAX;
    flag_error_current = 1;
}

// ********************************************************************************************************
// ** Function name:           query
// ** Descriptions:            Function that transforms the voltage measured by the sensor to its equivalent current
// **********************************************************************************************************
int Current_MOD::query(int time, char* buffer)
{
    error = Current_OK;
    /*
    VoltageADC = readAnalogValue();

    //printValue(VoltagemV);

    if(VoltageADC < 400){
    	flag_error_current = 1;
    	flag_current = 1;
    	error = 1;
    }
    else{
    	flag_error_current = 0;
    	flag_current = 0;
    	error = Current_OK;
    }
	
    if(VoltageADC <=  400)
    {
     //error=Current_ERROR_Comunication;
    }

    VoltageV=VoltageADC*3.3/1023; //AnalogRead function reads a value between 0-1023 (1024, 10 bits) here I get the real voltage value based on the value the function gets

    if(VoltageV >= 2.8){
    	flag_current = 1;
    }


    printnl("mV");
    printValue(VoltageADC);
    printnl("V");
    printValue(VoltageV);
    

    //printValue(Current);
    Current=(2.5-VoltageV)/0.0057; //Sensitivity is 5,7 mv/A
    */

    int Current1,Current2,Current3,Current4,Current5,Current6,Current7;
    Current1 = readAnalogValue();
    //HAL_Delay(3);
    Current2 = readAnalogValue();
    //HAL_Delay(3);
    Current3 = readAnalogValue();
    //HAL_Delay(3);
    Current4 = readAnalogValue();
    //HAL_Delay(3);
    Current5 = readAnalogValue();
    //HAL_Delay(3);
    Current6 = readAnalogValue();
    //HAL_Delay(3);
    Current7 = readAnalogValue();
    //HAL_Delay(3);

    Current = (Current1+Current2+Current3+Current4+Current5+Current6+Current7)/7;

    Current = 0.22727 * Current - 489.455 + 0.5;

    printValue(Current);
    //printValue(Current);

    if(Current > C_MAX*0.8 && Current < C_MAX)
    {
        if(flag_error_current == 0) module_send_message_NoExtId_CAN1(0x500,message,1); //If current between 80 and 100% of maximun, sends alert
    }

    if (Current > C_MAX)
    {
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
    }
    else
    {
        if (flag_error_current != 0)
		for (int i = 0; i < 5; i++)
		{
			module_send_message_NoExtId_CAN1(0x502, 0, 2); //If current normal, sends green flag
		}
        flag_error_current = 0;
    }


    if (time > time_lim_sended)
    {
        time_lim_sended += TIME_LIM_SEND;
        message[0] = 0;
        message[1] = Current & 0xFF;
        module_send_message_NoExtId_CAN1(CANID, message, 2); //Sends current through CAN each interval of ms
    }


    if (TIME_LIM_PLOT > 0 && time > time_lim_plotted)
    {
        time_lim_plotted += TIME_LIM_PLOT;
        info(buffer);

    }

    
    return error;
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void Current_MOD::info(char* buffer) {
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


