/*
 * VCU.h
 *
 *  Created on: Mar 16, 2024
 *      Author: canib
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_


#ifndef INT32U
#define INT32U unsigned long
#endif

#ifndef INT8U
#define INT8U byte
#endif

#define MAX_LENGTH 32

// CAN Telemetría config
#define CAN_TEL_CS 44
#define CAN_TEL_INT 46
#define CAN_TEL_KBPS 250000

// CAN Inversor config
#define CAN_INV_CS 4
#define CAN_INV_INT 5
#define CAN_INV_KBPS 500000
#define N_DATOS_INV 5

// Conversiones inversor
#define CONV_DC_BUS_VOLTAGE 55

// Sensores config
#define S1_ACELERACION_PIN A2
#define S2_ACELERACION_PIN A3
#define S_FRENO_PIN A1

#define FR_SUSPENSION_PIN A17
#define FL_SUSPENSION_PIN A16


// Secuencia de arranque config
#define RTDS_PIN 5
#define START_BUTTON_PIN 4
#define START_BUTTON_LED 6

// Botones dashboard
#define DASH_INPUT_1 35
#define DASH_INPUT_2 34

// INVERSOR
#define INV_DATA_PERIOD 0x19 // Send data each 25 ms
#define READ 0x3D // REGID for reading data from the servo and transmission to the CAN
#define RFE_RUN 0xD8 // RUN and RFE REGID
#define RFE_RUN_EN 0x30 // Si RUN (bit 4) y RFE (bit 5) están enable
#define BTB 0xE2 // Device ready
#define MODE 0x51 // Mode State
#define I_FRG 0xE8 // Digital input RUN
#define TORQUE 0x90

// Telemetry data
#define DC_BUS_VOLTAGE 0xEB // DC-Bus voltage
#define I_ACTUAL 0X5F // Filtered actual current
#define T_MOTOR 0x49 // Motor temperature
#define T_IGBT 0x4A // Power stage temperature
#define T_AIR 0x4B // Air temperature
#define N_ACTUAL 0x30 // Speed actual value

// Delays
#define DELAY_CONFIG 100
#define DELAY_CAN_SEND 10 //Cambiado de 0

#define UMBRAL_FRENO  1000 // Valor léido del ADC a partir del cual se considera que el pedal de freno ha sido pulsado
#define UMBRAL_FRENO_APPS 3000

//Filtro sensores acelerador
#define N_LECTURAS 10


// Periodo de recogida y envío de datos
unsigned long periodo_inv = 200; //ms
unsigned long periodo_tel = 800; //ms

// Comprobaciones y flags: 1 listo, 0 en proceso todavía
// Comprobaciones y flags: 1 listo, 0 en proceso todavía
int precarga_inv = 0; //0;
int config_inv_lectura_v = 0; //0;
int RTS_inv = 0; //0;
int BTB_todo = 0; //0
int BTB_inv_1 = 0;
int BTB_inv_2 = 0;
int boton_arranque = 0; //0

// ---------- IDs de los buses CAN ----------
// IDs CAN Inversor (definidos desde el punto de vista del inversor) - BAMOCAR
INT32U rxID_inversor = 0x201;
INT32U txID_inversor = 0x181;

// IDs CAN INVERSOR (definidos desde el punto de vista del inversor) - EPOWERLABS
#define TX_STATE_1  0x460
#define TX_STATE_2  0x461
#define TX_STATE_3  0x462
#define TX_STATE_4  0x463
#define TX_STATE_5  0x464
#define TX_STATE_6  0x465
#define TX_STATE_7  0x466
#define TX_STATE_8  0x467
#define TX_STATE_9  0x468


INT32U RX_SETPOINT_1 = 0x360;
INT32U RX_SETPOINT_2 = 0x361;
INT32U RX_SETPOINT_3 = 0x362;
INT32U RX_SETPOINT_4 = 0x363;
INT32U RX_SETPOINT_6 = 0x365;

// --------- BMI088 ---------

// IDs CAN Telemetría Generales
INT32U ID_RTD_all = 0x80;

// IDs CAN Telemería AMS
INT32U ID_dc_bus_voltage = 0x100; // COMPROBAR CON JULIANI
INT32U ID_ack_precarga = 0x20;
INT32U ID_v_celda_min= 0x12C;

// IDs CAN Telemetría ECU_Telemetría
INT32U ID_ok_telemetria = 0xA0;
INT32U ID_ack_telemetria = 0x30;

// IDs CAN Telemetría ECU_CAJA_NEGRA
INT32U ID_ok_caja_negra = 0xB0;
INT32U ID_ack_caja_negra = 0x40;

// IDs CAN Telemetría RPI_PANTALLA
INT32U ID_ok_pantalla = 0xE0;
INT32U ID_ack_pantalla = 0x50;

// IDs Sensores
INT32U ID_s1_aceleracion = 0x101;
INT32U ID_s2_aceleracion = 0x102;
INT32U ID_s_freno = 0x103;
INT32U ID_torque_total = 0x106;
INT32U ID_velocidad = 0x104;
INT32U ID_sfr_suspension = 0x104;
INT32U ID_sfl_suspension = 0x105;

// IDs Envío Datos Inversor por el CAN_TEL
INT32U ID_t_motor = 0x301;
INT32U ID_t_igbt = 0x302;
INT32U ID_t_air = 0x303;
INT32U ID_n_actual = 0x304;
INT32U ID_i_actual = 0x305;


INT32U ID_IMU_FRONT = 0x620;


#endif /* INC_VCU_H_ */
