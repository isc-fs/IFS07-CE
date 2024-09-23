// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_temperatures.h
// Description  :
// * This file is for declaring the functions and variables of the class of the temperatures
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "class_temperatures.h"


// ********************************************************************************************************
// **Function name:           Temperatures_MOD
// **Descriptions:            Initialization function of teh class
// **********************************************************************************************************
Temperatures_MOD::Temperatures_MOD(uint32_t ID, int _T_MAX, int _LAG)
{
  MODULEID = ID;
  LIMIT_MAX_T = _T_MAX;

  time_lim_plotted += _LAG;
  time_lim_sended += _LAG;
  time_lim_received += _LAG;
}

// ********************************************************************************************************
// **Function name:           info
// **Descriptions:            Function for printing the class data
// **********************************************************************************************************
void Temperatures_MOD::info(char *buffer)
{
  if(getUARTState() == HAL_UART_STATE_READY) //Send the message just if there is a serial por connected
  {
	print((char*)"\n***********************");
	print((char*)"     Temperatures");
    print((char*)"***********************");
    sprintf(buffer, " - ERROR:     %i", error);
    print(buffer);
    sprintf(buffer, " - CAN ID:    0x%lx", MODULEID);
    print(buffer);
    sprintf(buffer, " - MAX T =    %i ºC", MAX_T);
    print(buffer);
    sprintf(buffer, " - MIN T =    %i ºC", MIN_T);
    print(buffer);
    sprintf(buffer, " - LIM T =    %i ºC", LIMIT_MAX_T);
    print((char*)"-----------------------");
    sprintf(buffer, "Temperatures (ºC): [%i", cellTemperature[0]);
    printnl(buffer);
    for (int i = 1 ; i < 38; i++)
    {
    	sprintf(buffer, ", %i", cellTemperature[i]);
     	printnl(buffer);
    }
    print((char*)"]");


  }
}


// ********************************************************************************************************
// **Function name:           parse
// **Descriptions:            Function for parsing the received data via CAN protocl
// **********************************************************************************************************
bool Temperatures_MOD::parse(uint32_t id, uint8_t *buf, uint32_t t)
{
  if (id > MODULEID && id < MODULEID + 7)
  {
	//module_send_message_CAN1(0x530, buf, 8);
    int m = id % MODULEID;
    int pos = 0;
    if (m > 0 && m < 7)
    {
      time_lim_received = t + TIME_LIM_RECV;

      if (m < 6)
      {

        if (flag_charger == 1){
        	module_send_message_CAN1(id, buf, 8); //Reenviar temperaturas por CAN1 tanto en cargador como en coche
    	}
        for (int i = 0; i < 8; i++)
        {
          pos = (m - 1) * 8 + i;
          cellTemperature[pos] = buf[i];
         // if (cellTemperature[pos] > LIMIT_MAX_T) error = Temperatures_ERROR_MAXIMUM_T;

        }
      }
      else if (m == 6)
      {
        if (flag_charger == 1){
        	module_send_message_CAN1(id, buf, 8); //Reenviar temperaturas por CAN1 tanto en cargador como en coche
        }

        for (int i = 0; i < 3; i++)
        {
          pos = (m - 1) * 8 + i;
         // if (cellTemperature[pos] > LIMIT_MAX_T) error = Temperatures_ERROR_MAXIMUM_T;

        }
      }

      MAX_T = cellTemperature[0];
      MIN_T = cellTemperature[0];
      for (int i = 0; i < 38; i++)
      {
        if (cellTemperature[i] > MAX_T)
          MAX_T = cellTemperature[i];
        else if (cellTemperature[i] < MIN_T && cellTemperature[i]!=0){
          MIN_T = cellTemperature[i];
        }

        if (id!= 0x530){
        	temp_bckp[i] = cellTemperature[i];
        }

         else if(id == 0x530){
        	cellTemperature[i] = temp_bckp[i];
        }
      }

      return true;
    }
  }

  return false;
}

// ** Function name:           query
/*
// ********************************************************************************************************
// **Function name:           return_error
// **Descriptions:            Function for returning the state of the BMS
// **********************************************************************************************************
int BMS_MOD::return_error()
{
  return error;
}
*/
// ********************************************************************************************************
// ** Descriptions:            Function to check if i need to send a mesage new mesage and the received mesajes interval are within the limits
// **********************************************************************************************************
int Temperatures_MOD::query(uint32_t time, char *buffer)
{
  // Function for performing a correct behaivour
  if (time > time_lim_sended)
  { // HERE I HAVE TO SEND THE REQUEST MESSAGE FOR THE TEMPERATURES
    time_lim_sended += TIME_LIM_SEND;

    if (module_send_message_CAN2(MODULEID, message_temperatures, 2) != HAL_OK)
    {
      error = Temperatures_ERROR_COMMUNICATION; // If the message is not sended then, error
    }
    else
    {
      /*       Serial.print("Ennvado solicitud a: ");
            Serial.println(MODULEID,HEX); */
    }

    for(int i = 0; i<38; i++){
    	if(cellTemperature[i] > 65){
    		error = 2;
    	}
    }

    // time_lim_sended += TIME_LIM_SEND; //Si actualizas dos veces, el mensaje se envía en la mitad del periodo
  }
  if (time > time_lim_received)
  {
    //error = Temperatures_ERROR_COMMUNICATION;
  }
  if (TIME_LIM_PLOT > 0 && time > time_lim_plotted)
  {
    time_lim_plotted += TIME_LIM_PLOT;
    info(buffer);
  }

/*     if(time > time_lim_sended)
    {
      time_lim_sended += TIME_LIM_SEND;

      message_temperatures[0] = 0;
      message_temperatures[1] = MAX_T & 0xFF;
      module_send_message_CAN1(CANIDTEL, 0, 2, message_temperatures);
    } */

  return error;
}










