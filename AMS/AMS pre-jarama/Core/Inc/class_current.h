// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   class_current.h
// Description  :
// * This file is for declaring the functions and variables of the class of the current
// -----------------------------------------------------------------------------

#ifndef INC_CLASS_CURRENT_H_
#define INC_CLASS_CURRENT_H_


#define Current_OK                    0
#define Current_ERROR_MAXIMUN_C       1
#define Current_ERROR_Comunication    2


class Current_MOD {
private:

  int C_MAX             = 0;    //Maximum value of current

  uint32_t  CANID         = 0;    // ID of can device
  uint8_t   message[2]    = {0,0};

  float VoltageV = 0.0;             // Voltage in V     //All of them need to be in float, if the voltage changes from 0-5 V, if they were int, there would only be 5 possible values
  float VoltageADC = 0.0;
  int Current =0 ;              // Current
  float offset_V=0.0;               //offset voltage
  int time;

  int flag_error_current = 0;



  int TIME_LIM_PLOT   = 1500;     // The info will be shown through Serial comunications each 5 second
  int TIME_LIM_SEND   = 250;     //The value will be sended each second
  int time_lim_plotted= 50;
  int time_lim_sended = 50;

public:
  int error             = 0;    // Variable for error handling //Needs to be public so it can be seen from outside the class
  int voltage_acum  = 0;
  int flag_charger = 0;

  int flag_current = 0;


  Current_MOD(uint32_t ID,int _C_MAX);
  int query(int time, char *buffer);
  void info(char *buffer);
  //HAL_StatusTypeDef module_send_message_CAN1(uint32_t id, uint8_t* data, uint8_t length);
};

#endif /* INC_CLASS_CURRENT_H_ */
