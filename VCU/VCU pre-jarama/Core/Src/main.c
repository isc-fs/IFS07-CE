/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "time.h"
#include "VCU.h"
#include "LPF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ---------- MODOS DEBUG ----------
#define DEBUG 1
#define CALIBRATION 0

// ---------- FILTROS LECTURA PEDAL ACELERADOR --------
uint32_t lecturas_s1[N_LECTURAS] = {0};
uint32_t lecturas_s2[N_LECTURAS] = {0};
uint8_t index_s1 = 0, index_s2 = 0;

// ---------- VARIABLES DEL CAN ----------
FDCAN_TxHeaderTypeDef TxHeader_Inv;
FDCAN_RxHeaderTypeDef RxHeader_Inv;
FDCAN_TxHeaderTypeDef TxHeader_Acu;
FDCAN_RxHeaderTypeDef RxHeader_Acu;
FDCAN_TxHeaderTypeDef TxHeader_Dash;
FDCAN_RxHeaderTypeDef RxHeader_Dash;

uint8_t TxData_Inv[8];
uint8_t RxData_Inv[8];
uint8_t TxData_Acu[8];
uint8_t RxData_Acu[8];
uint8_t TxData_Dash[8];
uint8_t RxData_Dash[8];

// Datos a recibir del inversor
INT32U datos_inversor[N_DATOS_INV] = {T_MOTOR, T_IGBT, T_AIR, N_ACTUAL,
									  I_ACTUAL};

// Datalogger MicroSD
FATFS FatFs; // Filesystem
#define LOG_BUFFER_SIZE 512
char logBuffer1[LOG_BUFFER_SIZE];
char logBuffer2[LOG_BUFFER_SIZE];
uint16_t bufferIndex = 0;
char *activeBuffer = logBuffer1;
char *writeBuffer = NULL;
uint8_t bufferReady = 0;

// Detector de flanco botón de arranque
int start_button_act;
int start_button_ant = 0; // 1

// ---------- VARIABLES DE LECTURA DE SENSORES ----------

// Inversor
int inv_dc_bus_voltage = 0; // Lectura de tensión DC bus (V)
int inv_dc_bus_power = 0;	// Lectura de potencia DC bus (W)
int e_machine_rpm = 0;		// Lectura RPMs motor
int inv_t_motor;			// Lectura de motor temperature
int inv_t_igbt;				// Lectura de power stage temperature
int inv_t_air;				// Lectura de air temperature
int inv_n_actual;			// Lectura de speed actual value

// Sensores
uint16_t buffer_adc[3]; // Buffer para DMA
uint16_t s1_aceleracion = 0; // Lectura del sensor 1 del pedal de aceleración
uint16_t s2_aceleracion = 0; // Lectura del sensor 2 del pedal de aceleración
uint16_t s1_aceleracion_aux = 0;
uint16_t s2_aceleracion_aux = 0;
LPF_EMA s1_filt, s2_filt;
int s_freno; // Lectura del sensor de freno
float s_freno_aux;
int sdd_suspension; // Lectura del sensor delantero derecho de suspensión
int sdi_suspension; // Lectura del sensor delantero izquierdo de suspensión
float aux_velocidad;
float v_celda_min = 3600; // Contiene el ultimo valor de tension minima de una celda enviada por el AMS.

// ---------- VARIABLES DE CONTROL DEL INVERSOR ----------
int porcentaje_pedal_acel;
uint16_t torque_total = 0;
uint8_t byte_torque_1 = 0;
uint8_t byte_torque_2 = 0;
int torque_limitado;
uint16_t real_torque = 0;
int media_s_acel = 0;
uint8_t state = 0;
uint8_t byte0_voltage = 0;
uint8_t byte1_voltage = 0;
uint8_t byte0_power = 0;
uint8_t byte1_power = 0;
uint32_t last_time_t_11_8 = 0;

// Revisar normativa
int flag_EV_2_3 = 0;
int flag_T11_8_9 = 0;
int count_T11_8_9 = 0;

int precharge_button = 0;

char uart_msg[100];
char TxBuffer[250];
uint8_t error = 0;

void print(char uart_buffer[]);
void printValue(int value);
void printHex(uint8_t value);

uint16_t setTorque(void);
int SMA(uint32_t *lecturas, uint8_t *index, uint32_t lectura);

uint8_t flag_react = 0;

uint8_t flag_r2d = 0;

void SDCard_start(void);
void logBufferToSD(void);
void addDataToLogBuffer(char *data, uint16_t length);
void processLogging(void);
void SDCard_write(char *filename, char *data, int newFile);
//void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
	// Inicializar tarjeta microSD
	//SDCard_start();
	//HAL_Delay(2000);

	// EJEMPLO SDCARD

	//char buffer[100];
	// char timestamp[50];

	//sprintf(buffer, "%u,test,test1,test2\n", (unsigned)time(NULL));

	//print(buffer);
	//SDCard_write("data.csv", buffer, 1);
	//SDCard_write("data.csv", "test,test1,test2\n", 1); // el modo 1 crea un archivo nuevo (importante añadir \n al final)
	//print(buffer);						 // Mostrar el buffer por USART
	// fprintf(timestamp, "%u\n", (unsigned)time(NULL));
	//sprintf(buffer, "%u,1test,1test1,1test2\n", (unsigned)time(NULL));
	//SDCard_write("data.csv", "1test,1test1,1test2\n", 0); // el modo 0 asume que existe el archivo y añade datos
	//print(buffer);

	// HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buffer_adc, 3) != HAL_OK)
	{
#if DEBUG
		print("Error al inicializar ADC_DMA");
#endif
		Error_Handler();
	}
	// Inicializacion filtro paso bajo
	//LPF_EMA_Init(&s1_filt, 0.2f);
	//LPF_EMA_Init(&s2_filt, 0.2f);

	// Inicialización de buses CAN
	// Inversor
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
#if DEBUG
		print("Error al inicializar CAN_INV");
#endif
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{

#if DEBUG
		print("Error al activar NOTIFICACION CAN_INV");
#endif
		Error_Handler();
	}

	/*if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK){
#if DEBUG
		print("Error al activar NOTIFICACION BUS OFF CAN_INV");
#endif
		Error_Handler();
	}*/

	// Acumulador
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{

#if DEBUG
		print("Error al inicializar CAN_ACU");

#endif
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{

#if DEBUG
		print("Error al activar NOTIFICATION CAN_ACU");
#endif
		Error_Handler();
	}

	//Dash
	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{

#if DEBUG
		print("Error al inicializar CAN_DASH");

#endif
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{

#if DEBUG
		print("Error al activar NOTIFICATION CAN_DASH");
#endif
		Error_Handler();
	}

	//---------- SECUENCIA DE ARRANQUE ----------
#if (DEBUG)
	print("Solicitar tensión inversor");
#endif

#if (CALIBRATION)
	config_inv_lectura_v = 1;
#endif

	// Espera ACK inversor (DC bus)
	while (config_inv_lectura_v == 0)
	{
		print("Solicitar tensión inversor");
		if (config_inv_lectura_v == 1)
		{

#if DEBUG
			print("CAN_INV: Lectura de DC_BUS_VOLTAGE correctamente");
#endif
		}
	}

#if !CALIBRATION

	// PRE-CHARGE
	while (precarga_inv == 0 && inv_dc_bus_voltage < 300)
	{

#if DEBUG
		sprintf(TxBuffer, "DC_BUS_VOLTAGE: %i V\r\n", inv_dc_bus_voltage);
		//print(TxBuffer);
		// printValue((int) ((byte1_voltage << 8) | byte0_voltage));
#endif

		// Reenvío DC_BUS_VOLTAGE al AMS por CAN_ACU
		TxHeader_Acu.Identifier = ID_dc_bus_voltage;
		TxHeader_Acu.DataLength = 2;
		TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
		TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
		TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;

		/*		TxData_Acu[0] = byte0_voltage;
		 TxData_Acu[1] = byte1_voltage;*/
		TxData_Acu[0] = inv_dc_bus_voltage & 0xFF;
		TxData_Acu[1] = (inv_dc_bus_voltage >> 8) & 0xFF;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK)
		{
#if DEBUG
			//print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
#endif
		}

		precharge_button = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,
											START_BUTTON_Pin);
		if (precharge_button == 1){
			TxHeader_Acu.Identifier = 0x600;
			TxHeader_Acu.DataLength = 2;
			TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
			TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
			TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;


			TxData_Acu[0] = precharge_button;

			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK)
			{
	#if DEBUG
				//print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
	#endif
			}
		}



		if (precarga_inv == 1)
		{
#if DEBUG
			print("CAN_ACU: Precarga correcta");
#endif
		}

	}

	TxHeader_Acu.Identifier = ID_dc_bus_voltage;
	TxHeader_Acu.DataLength = 2;
	TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;

	TxData_Acu[0] = inv_dc_bus_voltage & 0xFF;
	TxData_Acu[1] = (inv_dc_bus_voltage >> 8) & 0xFF;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK)
	{
#if DEBUG
		//print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
#endif
	}

#endif

	/*
	 * TIM16 -> APB2 => 264MHzw
	 * 10 ms interruption => 10ms * 264MHz = 2640000
	 * preescalado 264 (por ejemplo)
	 * timer count = 2640000 / 264 = 10000
	 */
#if !CALIBRATION
	HAL_TIM_Base_Start_IT(&htim16);
#endif

#if 1
	// Espera a que se pulse el botón de arranque mientras se pisa el freno
	while (boton_arranque == 0)
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

		s_freno = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);

		//printValue(s_freno);
		print("Pulsa botón");

		start_button_act = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,
											START_BUTTON_Pin);

		if (start_button_act == 1)
		{

#if DEBUG
			//printValue(s_freno);
			print("Pulsa freno");
#endif
			if (s_freno > 900)
			{
				boton_arranque = 1;
#if DEBUG
				print("Coche arrancado correctamente");
#endif
			}
			else
			{
#if DEBUG
				print("Pulsar freno para arrancar");
#endif
			}
		}
	}
#endif

	// Activar READY-TO-DRIVE-SOUND (RTDS) durante 2s
#if DEBUG
	print("RTDS sonando");
#endif
#if !CALIBRATION

	flag_r2d = 1;
	HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_SET); // Enciende RTDS
	HAL_Delay(2000);
	HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_RESET); // Apaga RTDS

#endif

#if DEBUG
	print("RTDS apagado");
#endif

	// Estado STAND BY inversor
	while (state != 3)
	{
		if (state == 3)
		{
#if DEBUG
			print("Precarga");
#endif
		}
	}

#if DEBUG
	print("state : stand by");
#endif

	while (state != 4)
	{
		// Estado READY inversor
		TxHeader_Inv.Identifier = RX_SETPOINT_1;
		TxHeader_Inv.DataLength = 3;
		TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

		TxData_Inv[0] = 0x0;
		TxData_Inv[1] = 0x0;
		TxData_Inv[2] = 0x4;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);

		TxHeader_Inv.Identifier = 0x362;
		TxHeader_Inv.DataLength = 4;

		real_torque = 0;

		TxData_Inv[0] = 0x0;
		TxData_Inv[1] = 0x0;
		TxData_Inv[2] = real_torque;
		TxData_Inv[3] = 0x0;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
		HAL_Delay(10);

	}

#if DEBUG
	print("state: ready");
#endif



	// Avisar a resto de ECUs de que pueden comenzar ya a mandar datos al CAN (RTD_all)
	// Inicia telemetria y activa los ventiladores
	/*TxHeader_Acu.Identifier = ID_RTD_all;
	 TxHeader_Acu.DataLength = 1;
	 TxData_Acu[0] = 1;
	 HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		// Envío datos telemetría
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 1;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0;
	sFilterConfig.FilterID2 = 0x0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 16;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 16;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0;
	sFilterConfig.FilterID2 = 0x0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 6;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 2;
  hfdcan3.Init.NominalTimeSeg2 = 5;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 1;
  hfdcan3.Init.RxFifo0ElmtsNbr = 16;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 16;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0;
	sFilterConfig.FilterID2 = 0x0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 528;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, START_BUTTON_LED_Pin|RTDS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_Data_GPIO_Port, DS18B20_Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_BUTTON_LED_Pin RTDS_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_LED_Pin|RTDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MICROSD_DET_Pin */
  GPIO_InitStruct.Pin = MICROSD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MICROSD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Data_Pin */
  GPIO_InitStruct.Pin = DS18B20_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON1_Pin */
  GPIO_InitStruct.Pin = START_BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(START_BUTTON1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print(char uart_buffer[])
{
	sprintf(uart_msg, "%s \n\r", uart_buffer);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg, strlen(uart_msg),
					  HAL_MAX_DELAY);
}

void printValue(int value)
{
	sprintf(uart_msg, "%hu \n\r", value);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg, strlen(uart_msg),
					  HAL_MAX_DELAY);
}

void printHex(uint8_t value)
{
	sprintf(uart_msg, "0x%02X \n\r", value);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_msg, strlen(uart_msg),
					  HAL_MAX_DELAY);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//s1_aceleracion = buffer_adc[0];
	//s2_aceleracion = buffer_adc[1];
	//s_freno = buffer_adc[2];
}

void ADC2_Select_FR(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC2_Select_Brake(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC2_Select_FL(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC2_Select_RL(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC2_Select_RR(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retreive Rx messages from RX FIFO0 */

		if (hfdcan->Instance == FDCAN1)
		{
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv,
									   RxData_Inv) == HAL_OK)
			{
				switch (RxHeader_Inv.Identifier)
				{
				case TX_STATE_2:
					state = RxData_Inv[4] & 0xF;

					if (state == 10 || state == 11)
					{
						error = RxData_Inv[2];
						//printHex(error);
					}
					break;

				case TX_STATE_4:
					e_machine_rpm = (RxData_Inv[7] & 0x0F << 16) | RxData_Inv[8] << 8 | RxData_Inv[5];
					if (e_machine_rpm & 0x80000)
					{
						e_machine_rpm |= 0xFFF0000;
					}
					break;

				case TX_STATE_7:
					if (RxHeader_Inv.DataLength == 6)
					{
						if (config_inv_lectura_v == 0)
						{
							config_inv_lectura_v = 1;
						}
						if (config_inv_lectura_v == 1)
						{
							//inv_dc_bus_voltage = (int)RxData_Inv[1] << 8 | (int)RxData_Inv[0];
							inv_dc_bus_voltage = RxData_Inv[3] << 8 | RxData_Inv[2];
							//printValue(inv_dc_bus_voltage);
							//							byte0_voltage = RxData_Inv[0];
							//							byte1_voltage = RxData_Inv[1];
							//inv_dc_bus_power = (int)RxData_Inv[2] << 8 | (int)RxData_Inv[1];
							//inv_dc_bus_power = inv_dc_bus_power >> 2; // Bits 10 to 16
							//if (inv_dc_bus_power & 0x2000)
							//{ // Check for bit signing
							//	inv_dc_bus_power |= 0xC000;
							//}
							//inv_dc_bus_power = inv_dc_bus_power * 32767; // Scale factor
						}
					}

					break;
				}
			}
		}
		else if (hfdcan->Instance == FDCAN2)
		{
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_Acu,
									   RxData_Acu) == HAL_OK)
			{
				switch (RxHeader_Acu.Identifier)
				{
				case 0x20: // ID_ack_precarga:
					if (RxData_Acu[0] == 0)
					{
						precarga_inv = 1;
					}
					break;

				case 0x12C:
					v_celda_min = (int)(RxData_Acu[0] << 8 | RxData_Acu[1]);
					break;
				}
			}
		}
		else if (hfdcan->Instance == FDCAN3)
		{
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_Dash,
									   RxData_Dash) == HAL_OK)
			{
				switch (RxHeader_Dash.Identifier)
				{
				case 0x133:
					s1_aceleracion = ((uint16_t)RxData_Dash[0] << 8) | RxData_Dash[1];
					s2_aceleracion = ((uint16_t)RxData_Dash[2] << 8) | RxData_Dash[3];
					/*print("s1");
					printValue(s1_aceleracion);
					print("s2");
					printValue(s2_aceleracion);*/

				}
			}
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
	if (hfdcan == &hfdcan2) {
		if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
			//CAN_bus_off_check_reset(hfdcan);
		}
	}
}

/*int SMA(uint32_t *lecturas, uint8_t *index, uint32_t lectura)
{
	lecturas[*index] = lectura;
	*index = (*index + 1) % N_LECTURAS;
	uint32_t sum = 0;
	for (uint8_t i = 0; i < N_LECTURAS; i++)
	{
		sum += lecturas[i];
	}
	return sum / N_LECTURAS;
}*/

uint16_t setTorque()
{
	// Leemos sensores de posición del pedal de acelaración

	//int s1_aceleracion_filtr = LPF_EMA_Update(&s1_filt, s1_aceleracion);
	//int s2_aceleracion_filtr = LPF_EMA_Update(&s2_filt, s2_aceleracion);


#if 0
	print("Sensor 1: ");
	printValue(s1_aceleracion);
	print("");
	print("Sensor 2: ");
	printValue(s2_aceleracion);
	print("");
#endif

#if 0
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	s_freno = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);
	print("Sensor freno: ");
	printValue(s_freno);
#endif

	// Calculamos % torque  en función de la posición de los sensores
	s1_aceleracion_aux = (s1_aceleracion - 2050) / (29.5 - 20.5);
	if (s1_aceleracion_aux < 0)
	{
		s1_aceleracion_aux = 0;
	}
	else if (s1_aceleracion_aux > 100)
	{
		s1_aceleracion_aux = 100;
	}

	s2_aceleracion_aux = (s2_aceleracion - 1915) / (25.70 - 19.15);
	if (s2_aceleracion_aux < 0)
	{
		s2_aceleracion_aux = 0;
	}
	else if (s2_aceleracion_aux > 100)
	{
		s2_aceleracion_aux = 100;
	}

#if 0
	print("Sensor % 1: ");
	printValue(s1_aceleracion_aux);
	print("");
	print("Sensor % 2: ");
	printValue(s2_aceleracion_aux);
	print("");
#endif

	// Torque enviado es la media de los dos sensores
	if (s1_aceleracion_aux > 8 && s2_aceleracion_aux > 8)
	{
		torque_total = (s1_aceleracion_aux + s2_aceleracion_aux) / 2;
	}
	else
	{
		torque_total = 0;
	}

	// Por debajo de un 10% no acelera y por encima de un 90% esta a tope
	if (torque_total < 10)
	{
		torque_total = 0;
	}
	else if (torque_total > 90)
	{
		torque_total = 100;
	}

	// Comprobamos EV 2.3 APPS/Brake Pedal Plausibility Check
	// En caso de que se esté pisando el freno y mas de un 25% del pedal para. Se resetea
	// solo si el acelerador vuelve por debajo del 5%
	if (s_freno > UMBRAL_FRENO_APPS && torque_total > 25)
	{
		print("EV_2_3");
		flag_EV_2_3 = 1;
	}
	else if (s_freno < UMBRAL_FRENO_APPS && torque_total < 5)
	{
		flag_EV_2_3 = 0;
	}
	// If an implausibility occurs between the values of the APPSs and persists for more than
	// 100ms The power to the motor(s) must be immediately shut down completely
	// T11.8.9 Implausibility is defined as a deviation of more than ten percentage points
	// pedal travel between any of the used APPSs
	if (abs(s1_aceleracion_aux - s2_aceleracion_aux) > 10)
	{

		// if (HAL_GetTick() - last_time_t_11_8 > 100) {
		print("T11.8.9");
		flag_T11_8_9 = 1;
		//}
	}
	else
	{
		last_time_t_11_8 = HAL_GetTick();
		flag_T11_8_9 = 0;
	}

	if (flag_EV_2_3 || flag_T11_8_9)
	{
		torque_total = 0;
	}

#if 0
	print("Torque total solicitado: ");
	printValue(torque_total);
#endif

	// Limitación del torque en función de la carga
	if (v_celda_min < 3500)
	{
		if (v_celda_min > 2800)
		{
			torque_limitado = torque_total * (1.357 * v_celda_min - 3750) / 1000;
		}
		else
		{
			torque_limitado = torque_total * 0.05;
		}
	}
	else
	{
		torque_limitado = torque_total;
	}

	// Limitacion del torque en funcion de la potencia

#if 0
print("Torque limitado en: ");
printValue(torque_limitado);
#endif

	// torque_total = torque_total * 240 / 100;
	if (torque_total >= 10)
	{
		torque_total = (torque_total * 240 / 90 - 2400 / 90) * (100 / 100);
	}

	/*if(torque_total < 0){
		torque_total = 0;
	}*/

	// Invertir todos los bits (complemento a uno)
	uint16_t complement_one = ~torque_total;

	// Sumar 1 para obtener el complemento a dos
	uint16_t torque_real = complement_one + 1;

#if 0
	print("Torque mandado al inversor: ");
	printHex(torque_real);
#endif
	return torque_real;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim16)
	{
		// Reenvío DC_BUS_VOLTAGE al AMS por CAN_ACU
		TxHeader_Acu.Identifier = ID_dc_bus_voltage;
		TxHeader_Acu.DataLength = 2;
		TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
		TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
		TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;

		/*		TxData_Acu[0] = byte0_voltage;
		 TxData_Acu[1] = byte1_voltage;*/
		TxData_Acu[0] = inv_dc_bus_voltage & 0xFF;
		TxData_Acu[1] = (inv_dc_bus_voltage >> 8) & 0xFF;
		//printValue(inv_dc_bus_voltage);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK)
		{
#if DEBUG
			//print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
#endif
		}

		precharge_button = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);

		TxHeader_Acu.Identifier = 0x600;
		TxHeader_Acu.DataLength = 2;
		TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
		TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
		TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;


		TxData_Acu[0] = precharge_button;
		printValue(TxData_Acu[0]);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK)
		{
#if DEBUG
			//print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
#endif
		}


#if CALIBRATION
		real_torque = setTorque();
		byte_torque_1 = real_torque & 0xFF;
		byte_torque_2 = (real_torque >> 8) & 0xFF;
/*		printHex(byte_torque_1);
		printHex(byte_torque_2);*/
#endif

#if !CALIBRATION

		// ---------- CONTROL DEL INVERSOR ----------

		//printHex(state);
		// Estado TORQUE
		if ((state == 4 || state == 6) && flag_r2d == 1)
		{ // Si no hay que reactivar el coche manda siempre torque

			TxHeader_Inv.Identifier = RX_SETPOINT_1;
			TxHeader_Inv.DataLength = 3;
			TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

			TxData_Inv[0] = 0x0;
			TxData_Inv[1] = 0x0;
			TxData_Inv[2] = 0x6;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
		}

		if(flag_r2d == 1){
			switch (state)
			{
			case 0:
				TxHeader_Inv.Identifier = RX_SETPOINT_1;
				TxHeader_Inv.DataLength = 3;
				TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

				TxData_Inv[0] = 0x0;
				TxData_Inv[1] = 0x0;
				TxData_Inv[2] = 0x1;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);

			case 3:
	#if DEBUG
				//print("state: standby");
	#endif
				flag_react = 0;
				// Estado READY inversor
				TxHeader_Inv.Identifier = RX_SETPOINT_1;
				TxHeader_Inv.DataLength = 3;
				TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

				TxData_Inv[0] = 0x0;
				TxData_Inv[1] = 0x0;
				TxData_Inv[2] = 0x4;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);



				// while (state != 4) {

				//}

			case 4:

	#if DEBUG
				print("state: ready");
	#endif
				if (flag_r2d == 1){
					TxHeader_Inv.Identifier = 0x362;
					TxHeader_Inv.DataLength = 4;

					real_torque = 0;

					TxData_Inv[0] = 0x0;
					TxData_Inv[1] = 0x0;
					TxData_Inv[2] = real_torque;
					TxData_Inv[3] = 0x0;
					HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
					flag_react = 0; // Reactivado
				}
				else{
					flag_react = 0;
					// Estado READY inversor
					TxHeader_Inv.Identifier = RX_SETPOINT_1;
					TxHeader_Inv.DataLength = 3;
					TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

					TxData_Inv[0] = 0x0;
					TxData_Inv[1] = 0x0;
					TxData_Inv[2] = 0x4;
					HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
				}

				break;
			case 6:
				print("state: torque");

				// Request TORQUE inversor

				// flag_react = 1;

				real_torque = setTorque();

				TxHeader_Inv.Identifier = 0x362;
				TxHeader_Inv.DataLength = 4;

				// real_torque = 0;
				byte_torque_1 = real_torque & 0xFF;
				byte_torque_2 = (real_torque >> 8) & 0xFF;
				TxData_Inv[0] = 0x00;
				TxData_Inv[1] = 0x00;
				/*if(acelera > 0 && frena == 0){
					acelera++;
					TxData_Inv[2] = 0xFE;
					TxData_Inv[3] = 0xFF;
				}
				if(acelera > 250){
					TxData_Inv[2] = 0xFD;
					TxData_Inv[3] = 0xFF;
				}
				if(acelera > 350){
					TxData_Inv[2] = 0x0;
					TxData_Inv[3] = 0x0;
					frena = 1;
					acelera = 0;
				}
				if(frena > 0 && acelera == 0){
					frena++;
					TxData_Inv[2] = 0x0;
					TxData_Inv[3] = 0x0;
					if(frena > 500){
						acelera = 1;
						frena = 0;
					}
				}*/
				TxData_Inv[2] = byte_torque_1;
				TxData_Inv[3] = byte_torque_2;
				// TxData_Inv[2] = 0xFE;
				// TxData_Inv[3] = 0xFF;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
				//CAN_bus_off_check_reset(&hfdcan1);

				break;

			case 10:
				print("state: soft fault");
				printValue(error);

				// Estado READY inversor
				TxHeader_Inv.Identifier = RX_SETPOINT_1;
				TxHeader_Inv.DataLength = 3;
				TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

				TxData_Inv[0] = 0x0;
				TxData_Inv[1] = 0x0;
				TxData_Inv[2] = 0x13;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
				/*switch (error) {
				case 1:
					print("Error: Lost msg");
					break;

				case 2:
					print("Error: Undervoltage");
					break;
				case 3:
					print("Error: Overtemperature");
					break;
				}*/

				/*if (inv_dc_bus_voltage < 60)
				{

					// Estado STAND BY inversor
					while (state != 3)
					{

						flag_react = 1;

						TxHeader_Inv.Identifier = RX_SETPOINT_1;
						TxHeader_Inv.DataLength = 3;
						TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

						TxData_Inv[0] = 0x0;
						TxData_Inv[1] = 0x0;
						TxData_Inv[2] = 0x3;
						HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv,
													  TxData_Inv);
					}*/

			case 11:
				print("state: hard fault");
				flag_react = 1;
				TxHeader_Inv.Identifier = RX_SETPOINT_1;
				TxHeader_Inv.DataLength = 3;
				TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

				TxData_Inv[0] = 0x0;
				TxData_Inv[1] = 0x0;
				TxData_Inv[2] = 13;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);

			case 13:
				print("state: shutdown");
				TxHeader_Inv.Identifier = RX_SETPOINT_1;
				TxHeader_Inv.DataLength = 3;
				TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

				TxData_Inv[0] = 0x0;
				TxData_Inv[1] = 0x0;
				TxData_Inv[2] = 0x1;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);


				break;
			}
		}
#endif
	}
}
/*
void SDCard_start(void)
{
	FRESULT FR_Status;
	FATFS *FS_Ptr;
	DWORD FreeClusters;
	uint32_t TotalSize, FreeSpace;
	do
	{
		//------------------[ Mount The SD Card ]--------------------
		FR_Status = f_mount(&FatFs, SDPath, 1);
		if (FR_Status != FR_OK)
		{
			sprintf(TxBuffer, "Error! Error Code: (%i)\r\n", FR_Status);
			print(TxBuffer);
			break;
		}
		sprintf(TxBuffer, "SD montada correctamente \r\n\n");
		print(TxBuffer);
		//------------------[ Get & Print The SD Card Size & Free Space ]--------------------
		f_getfree("", &FreeClusters, &FS_Ptr);
		TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
		FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
		sprintf(TxBuffer, "Espacio total: %lu Bytes\r\n", TotalSize);
		print(TxBuffer);
		sprintf(TxBuffer, "Espacio libre: %lu Bytes\r\n\n", FreeSpace);
		print(TxBuffer);
	} while (0);
}

void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus;
	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
	if (protocolStatus.BusOff)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}

void logBufferToSD(void)
{
	if (writeBuffer == NULL)
		return; // No buffer is ready for writing

	FIL file;
	FRESULT res;
	UINT bytesWritten;

	// Open the log file in append mode. If it doesn't exist, create it.
	res = f_open(&file, "log.txt", FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK)
	{
		// Handle error: File not opened
		Error_Handler();
		return;
	}

	// Write the buffer to the file
	res = f_write(&file, writeBuffer, LOG_BUFFER_SIZE, &bytesWritten);
	if (res != FR_OK || bytesWritten < LOG_BUFFER_SIZE)
	{
		// Handle error: Write failed or not all bytes were written
		Error_Handler();
		f_close(&file); // Ensure the file is closed
		return;
	}

	// Close the log file
	res = f_close(&file);
	if (res != FR_OK)
	{
		// Handle error: File not closed properly
		Error_Handler();
	}

	// Mark buffer as available
	bufferReady = 0;
	writeBuffer = NULL;
}

void addDataToLogBuffer(char *data, uint16_t length)
{
	// Check if there is enough space in the current buffer
	if ((bufferIndex + length) >= LOG_BUFFER_SIZE)
	{
		// If not, mark the current buffer as ready for writing
		writeBuffer = activeBuffer;

		// Switch active buffer
		if (activeBuffer == logBuffer1)
		{
			activeBuffer = logBuffer2;
		}
		else
		{
			activeBuffer = logBuffer1;
		}

		// Reset the index for the new active buffer
		bufferIndex = 0;
		bufferReady = 1;
	}

	// Add new data to the active buffer
	memcpy(&activeBuffer[bufferIndex], data, length);
	bufferIndex += length;
}

void processLogging(void)
{
	// Call this function regularly, e.g., in the main loop or a timer interrupt
	if (bufferReady)
	{
		logBufferToSD(); // Write the data to the SD card
	}
}

void SDCard_write(char *filename, char *data, int newFile)
{
	FIL file;	// File object
	FRESULT fr; // FATFS function common result code
	UINT bw;	// Bytes written

	//------------------[ Open or Create a File ]--------------------

	if (newFile == 1)
	{
		fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS); // este modo crea un archivo y sobreescribe los anteriores
		if (fr != FR_OK)
		{
			sprintf(TxBuffer,
					"Error opening/creating file: %s. Error Code: (%i)\r\n",
					filename, fr);
			print(TxBuffer);
			return;
		}
	}
	else
	{
		fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND); // este modo añade datos sin sobreescribir
		if (fr != FR_OK)
		{
			sprintf(TxBuffer,
					"Error opening file: %s. Error Code: (%i)\r\n",
					filename, fr);
			print(TxBuffer);
			return;
		}
	}

	//------------------[ Write Data to the File ]--------------------
	fr = f_write(&file, data, strlen(data), &bw);
	if (fr != FR_OK || bw < strlen(data))
	{
		sprintf(TxBuffer, "Error writing to file: %s. Error Code: (%i)\r\n",
				filename, fr);
		print(TxBuffer);
		f_close(&file);
		return;
	}

	//------------------[ Close the File ]--------------------
	fr = f_close(&file);
	if (fr != FR_OK)
	{
		sprintf(TxBuffer, "Error closing file: %s. Error Code: (%i)\r\n",
				filename, fr);
		print(TxBuffer);
		return;
	}

	//------------------[ Success Message ]--------------------
	sprintf(TxBuffer, "Data written successfully to file: %s\r\n", filename);
	print(TxBuffer);
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
