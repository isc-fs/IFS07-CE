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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "LTC6802-2.h"
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CORRECTION 6
#define LOW_LTC_CORRECTION 6  // Calibration to account for voltage drop through buffer resistors and LTC6802 variations
#define HIGH_LTC_CORRECTION 6 // Typical value is about 6 for both of these, but may be +/-3mV in some cases
#define BASE_ID 300			  // Starting ID used for BMS module messaging to/from EVMS
#define USE_29BIT_IDS 0		  // Or 0 for 11-bit IDs
enum {
	BMS12_REQUEST_DATA, BMS12_REPLY1, BMS12_REPLY2, BMS12_REPLY3, BMS12_REPLY4
};

#define true 1
#define false 0

uint8_t errorLTC1 = 0;
uint8_t errorLTC2 = 0;

// Inputs from hex rotary pot for module ID selection

#define MOD_ID_NUM1 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
#define MOD_ID_NUM2 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
#define MOD_ID_NUM4 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
#define MOD_ID_NUM8 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)

#define cellRegisters 19

#define maxCells 24
#define maxTemps 38

#define COMMS_TIMEOUT 16

#define LTC6802_CS1_GPIO_PORT GPIOA
#define LTC6802_CS1_GPIO_PIN GPIO_PIN_8

#define LTC6802_CS2_GPIO_PORT GPIOB
#define LTC6802_CS2_GPIO_PIN GPIO_PIN_12

#define TEMPS 0 // For testing the ds18b20 readings disabling cell measurement

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// LTC6802 Command codes
const uint8_t WRCFG = 0x01;
const uint8_t RDCFG = 0x02;
const uint8_t RDCV = 0x04;
const uint8_t RDTMP = 0x08;
const uint8_t STCVAD = 0x10;
const uint8_t STTMPAD = 0x30;
const uint8_t PLADC = 0x40;
const uint8_t STOWDC = 0x70;
const uint8_t STOWAD = 0x20;

CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

uint8_t rxData[8];							  // CAN receive buffer
uint8_t txData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // CAN transmit buffer

uint32_t TxMailBox;

volatile uint8_t dataRequestedL = false;		// Low group, LTC #2
volatile uint8_t dataRequestedH = false;		// High group, LTC #1
volatile uint8_t temperaturesRequested = false; // Temperatures
volatile uint8_t rawValuesRequested = false;
volatile uint8_t selfTestRequested = false;
volatile uint8_t selfTest2Requested = false;

uint16_t moduleID = 0;

int data_counter = 0;

uint32_t commsTimer = 0;

float temp[maxTemps];	  // In deg C
uint16_t voltages[maxCells]; // In millivolts

volatile uint16_t shuntVoltage; // In millivolts
uint32_t shuntBits;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void GetModuleID(void);
void PubModuleID(void);
void Delay_us(uint16_t us);
void SPIWrite(SPI_HandleTypeDef *hspi, uint8_t data);
void SPIRead(SPI_HandleTypeDef *hspi, uint8_t cmd, uint8_t numRegisters,
		uint8_t *const buff);
void waitForADCComplete(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin);
bool readCellValues(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin, uint8_t *buff);
void decodeCellVoltages(const uint8_t *cellBytes, uint16_t *voltages_mV,
		int offset, float shuntVoltage, uint32_t *shuntBits);
void updateLEDStatus(uint16_t *voltages, int num_cells, uint16_t shuntBits,
		uint8_t slowCounter, uint32_t commsTimer, uint32_t comms_timeout,
		uint16_t shuntVoltage);
uint8_t calculatePEC(uint8_t *data, uint8_t len);
void RunSelfTest(uint8_t *cellBytes1, uint8_t *cellBytes2, uint8_t testCommand);
void configureLTC(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin, uint32_t shuntBits);
void startConversion(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	TM_OneWire_t OneWire1;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_CAN_Start(&hcan);
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	GetModuleID();

	PubModuleID();
	TM_OneWire_Init(&OneWire1, GPIOA, DQ_Pin);

	for (int n = 0; n < maxCells; n++)
		voltages[n] = 0;
	for (int n = 0; n < maxTemps; n++)
		temp[n] = 0;

	uint8_t cellBytes1[cellRegisters];
	uint8_t cellBytes2[cellRegisters];
	for (int n = 0; n < cellRegisters; n++) {
		cellBytes1[n] = 0;
		cellBytes2[n] = 0;
	}
	//int voltages[24][8]; //De momento no hacemos la media
	uint8_t counter = 0;
	uint8_t slowCounter = 0;

	uint8_t ds18b20_flag = 0;

	uint8_t devices, sensor_count, device[4][8];

	/* Check for any device on 1-wire bus*/

	devices = TM_OneWire_First(&OneWire1);
	sensor_count = 0;
	while (devices) {
		/* Increase count variable */
		sensor_count++;

		/* Get full 8-bytes rom address */
		TM_OneWire_GetFullROM(&OneWire1, device[sensor_count - 1]);

		/* Check for new device */
		devices = TM_OneWire_Next(&OneWire1);
	}

	// Set 9bit resolution for all sensors (93.75ms max conversion time)

	for (int i = 0; i < sensor_count; i++) {
		TM_DS18B20_SetResolution(&OneWire1, device[i],
				TM_DS18B20_Resolution_9bits);
	}

	// Calculates the necessary CAN packets for sending all the temperatures
	uint8_t n_packets_temps = sensor_count / 8 + ((sensor_count % 8) ? 1 : 0);

	if (sensor_count > 0)
		ds18b20_flag = 1;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// Split up the 32-bit shuntBits variable into two 12-bit chunks for each LTC
		uint32_t shuntBitsL = 0; // shuntBits & 0x0FFF; // Lower 12 bits
		uint32_t shuntBitsH = 0; // shuntBits >> 12; // Upper 12 bits
#if !TEMPS
		// Configure LTC6802 1 (HV-)

		configureLTC(&hspi1, LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				shuntBitsL);
		configureLTC(&hspi2, LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				shuntBitsH);

		// Start voltage sampling
		startConversion(&hspi1, LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN);
		startConversion(&hspi2, LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN);

		// Wait for ADC conversion
		waitForADCComplete(&hspi1, LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN);
		waitForADCComplete(&hspi2, LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN);

		// Read cell voltage registers HV-
		if (readCellValues(&hspi1, LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				cellBytes1)) {
			decodeCellVoltages(cellBytes1, voltages, 0, shuntVoltage,
					&shuntBits);

		if (readCellValues(&hspi2, LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN, cellBytes2)) {
			decodeCellVoltages(cellBytes2, voltages, 12, shuntVoltage,
					&shuntBits);
		}

#endif

			// Start temperature sampling on all devices

			TM_DS18B20_StartAll(&OneWire1);
			while (!TM_DS18B20_AllDone(&OneWire1))
				;

			// Extract temperature data

			for (int i = 0; i < sensor_count; i++) {
				if (TM_DS18B20_Read(&OneWire1, device[i], &temp[i])) {
					ds18b20_flag = 1;
				}
			}

			counter++;

			if (counter >= 8) {
				counter = 0;
				slowCounter = (slowCounter + 1) % 4;

				updateLEDStatus(voltages, maxCells, shuntBits,
						slowCounter, commsTimer, COMMS_TIMEOUT, shuntVoltage);
			}

			// ds18b20_flag = TM_OneWire_Reset(&OneWire1);
#if TEMPS
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, !ds18b20_flag);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
			HAL_Delay(500);
#endif
			// Update status LEDs

			if (commsTimer < COMMS_TIMEOUT)
				commsTimer++;
			else
				shuntVoltage = 0;

			if (dataRequestedL) {
				dataRequestedL = false;
				commsTimer = 0;

				// Voltage packets
				for (int packet = 0; packet < 3; packet++) {
					for (int n = 0; n < 4; n++) {
						uint16_t voltageInt = (uint16_t)(voltages[packet * 4 + n]);
						txData[n * 2] = voltageInt >> 8;
						txData[n * 2 + 1] = voltageInt & 0xFF;
					}

					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;
					txHeader.StdId = moduleID + packet + 1;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK) {
						Error_Handler();
					}

					HAL_Delay(1);
				}
			} else if (dataRequestedH) {
				dataRequestedH = false;
				commsTimer = 0;

				for (int packet = 0; packet < 3; packet++) {
					for (int n = 0; n < 4; n++) {
						uint16_t voltageInt = (uint16_t)(voltages[12 + packet * 4 + n]);
						txData[n * 2] = voltageInt >> 8;
						txData[n * 2 + 1] = voltageInt & 0xFF;
					}

					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;
					txHeader.StdId = moduleID + 10 + packet + 1;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK) {
						Error_Handler();
					}

					HAL_Delay(1);
				}
			} else if (temperaturesRequested) {
				temperaturesRequested = false;
				commsTimer = 0;

				for (int packet = 0; packet < n_packets_temps; packet++) {

					for (int i = 0; i < 8; i++) {
						if ((i + 8 * packet) < sensor_count) {
							txData[i] = temp[i + 8 * packet];
						} else {
							txData[i] = 0;
						}
					}

					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;
					txHeader.StdId = moduleID + 20 + packet + 1;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK) {
						Error_Handler();
					}

					HAL_Delay(1);
				}
			} else if (rawValuesRequested) {
				rawValuesRequested = false;
				commsTimer = 0;

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes1[packet * 6 + i];
					}
					txData[6] = 0xC0 + packet;
					txData[7] = 0; // Padding

					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;
					txHeader.StdId = moduleID + 100 + packet;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes2[packet * 6 + i];
					}
					txData[6] = 0xD0 + packet;
					txData[7] = 0;

					txHeader.StdId = moduleID + 110 + packet;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}
			} else if (selfTestRequested) {
				selfTestRequested = false;
				commsTimer = 0;

				RunSelfTest(cellBytes1, cellBytes2, 0x1E);

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes1[packet * 6 + i];
					}
					txData[6] = 0xE0 + packet; // etiqueta especial para Self Test
					txData[7] = 0;

					txHeader.StdId = moduleID + 120 + packet;
					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes2[packet * 6 + i];
					}
					txData[6] = 0xF0 + packet;
					txData[7] = 0;

					txHeader.StdId = moduleID + 125 + packet;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}
			} else if (selfTest2Requested) {
				selfTest2Requested = false;
				commsTimer = 0;

				RunSelfTest(cellBytes1, cellBytes2, 0x1F); // Self Test 2

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes1[packet * 6 + i];
					}
					txData[6] = 0xD0 + packet; // etiqueta para Self Test 2 LTC1
					txData[7] = 0;

					txHeader.StdId = moduleID + 110 + packet;
					txHeader.DLC = 8;
					txHeader.IDE = CAN_ID_STD;
					txHeader.RTR = CAN_RTR_DATA;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}

				for (int packet = 0; packet < 3; packet++) {
					for (int i = 0; i < 6; i++) {
						txData[i] = cellBytes2[packet * 6 + i];
					}
					txData[6] = 0xC0 + packet; // etiqueta para Self Test 2 LTC2
					txData[7] = 0;

					txHeader.StdId = moduleID + 115 + packet;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData,
							&TxMailBox) != HAL_OK)
						Error_Handler();

					HAL_Delay(1);
				}
			}

			else {
				HAL_Delay(4);
			}
		}
		/* USER CODE END WHILE */
	}
	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	/* USER CODE END CAN_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 16 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DQ_Pin | GPIO_PIN_10 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	/*Configure GPIO pins : DQ_Pin PA8 PA10 PA15 */
	GPIO_InitStruct.Pin = DQ_Pin | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin =
	GPIO_PIN_10 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
		// Reception error
		Error_Handler();
	}

	// Process received data
	uint32_t rxPacketID = rxHeader.StdId;
	if (USE_29BIT_IDS) {
		rxPacketID = rxHeader.ExtId;
	}

	// Only one packet we care about - the data request
	if (rxPacketID == moduleID) {
		shuntVoltage = (rxData[0] << 8) + rxData[1]; // Big endian format (high byte first)
		dataRequestedL = true;
	}

	if (rxPacketID == moduleID + 10) {
		shuntVoltage = (rxData[0] << 8) + rxData[1];
		dataRequestedH = true;
	}

	if (rxPacketID == moduleID + 20) {
		temperaturesRequested = true;
	}

	if (rxPacketID == moduleID + 25) {
		rawValuesRequested = true;
	}

	if (rxPacketID == moduleID + 15) {
		selfTestRequested = true;
	}

	else if (rxPacketID == moduleID + 16) {
		selfTest2Requested = true;
	}
}

void GetModuleID(void) {
	uint8_t rotarySwitch = 0;

	if (MOD_ID_NUM1)
		rotarySwitch += 1;
	if (MOD_ID_NUM2)
		rotarySwitch += 2;
	if (MOD_ID_NUM4)
		rotarySwitch += 4;
	if (MOD_ID_NUM8)
		rotarySwitch += 8;

	moduleID = BASE_ID + rotarySwitch * 30; // Atomic enough for 8-bit on STM32F1
}

void PubModuleID(void) {
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = moduleID;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox) != HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(1);
}

void Delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		;
}

void SPIWrite(SPI_HandleTypeDef *hspi, uint8_t cmd) {
	HAL_SPI_Transmit(hspi, (uint8_t*) &cmd, 1, HAL_MAX_DELAY);
}

void SPIRead(SPI_HandleTypeDef *hspi, uint8_t cmd, uint8_t numRegisters,
		uint8_t *const buff) {

	// Send command to read data
	HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);

	// Read the data registers
	HAL_SPI_Receive(hspi, buff, numRegisters, HAL_MAX_DELAY);
}

void waitForADCComplete(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin) {
	uint8_t cmd = 0x40; // PLADC (Poll ADC Conversion Status)
	uint8_t response = 0x00;

	do {
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(hspi, &response, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	} while (response == 0x00); // Mientras el ADC esté ocupado
}

/*void readCellValues(SPI_HandleTypeDef *hspi, uint8_t cmd, uint8_t numRegisters,
 uint8_t *const buff)
 {
 do
 {
 SPIRead(hspi, cmd, numRegisters, buff);
 } while (buff[0] == 0xff);
 }*/

bool readCellValues(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin, uint8_t *buff) {
	uint8_t tx[2] = { 0x80, RDCV }; // Address + command
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, buff, 19, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	uint8_t pec_received = buff[18];
	uint8_t pec_calc = calculatePEC(buff, 18);

	return pec_received == pec_calc;
}

void decodeCellVoltages(const uint8_t *cellBytes, uint16_t *voltages_mV,
		int offset, float shuntVoltage, uint32_t *shuntBits) {
	int correction = (offset == 0) ? LOW_LTC_CORRECTION : HIGH_LTC_CORRECTION;

	for (int i = 0; i < 6; i++) {
		int idx = i * 3;

		uint16_t adc1 = cellBytes[idx] + ((cellBytes[idx + 1] & 0x0F) << 8);
		uint16_t adc2 = ((cellBytes[idx + 1] >> 4) & 0x0F)
				+ (cellBytes[idx + 2] << 4);

		int c1 = offset + i * 2;
		int c2 = offset + i * 2 + 1;

		float v1 = adc1 * 1.5f;
		float v2 = adc2 * 1.5f;

		if (v1 > 0) {
			v1 += correction;
			if (c1 == 0 || c1 == 12)
				v1 -= correction / 2.0f;
		}
		if (v2 > 0) {
			v2 += correction;
			if (c2 == 0 || c2 == 12)
				v2 -= correction / 2.0f;
		}

		voltages_mV[c1] = (v1 <= 5000.0f) ? v1 : 0.0f;
		voltages_mV[c2] = (v2 <= 5000.0f) ? v2 : 0.0f;

		if (v1 > shuntVoltage && shuntVoltage > 0)
			*shuntBits |= (1 << c1);
		else
			*shuntBits &= ~(1 << c1);

		if (v2 > shuntVoltage && shuntVoltage > 0)
			*shuntBits |= (1 << c2);
		else
			*shuntBits &= ~(1 << c2);
	}
}

uint8_t calculatePEC(uint8_t *data, uint8_t len) {
	uint8_t crc = 0x00;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}

void RunSelfTest(uint8_t *cellBytes1, uint8_t *cellBytes2, uint8_t testCommand) {
	// Enviar comando de Self Test
	HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
			GPIO_PIN_RESET);
	SPIWrite(&hspi1, testCommand);
	HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
			GPIO_PIN_SET);

	HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
			GPIO_PIN_RESET);
	SPIWrite(&hspi2, testCommand);
	HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
			GPIO_PIN_SET);

	HAL_Delay(20);

	// Leer registros de celdas
	readCellValues(&hspi1, LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
			cellBytes1);
	readCellValues(&hspi2, LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
			cellBytes2);
}

void updateLEDStatus(uint16_t *voltages, int num_cells, uint16_t shuntBits,
		uint8_t slowCounter, uint32_t commsTimer, uint32_t comms_timeout,
		uint16_t shuntVoltage) {
	char notAllZeroVolts = false;

	for (int n = 0; n < num_cells; n++) {
		if (voltages[n] > 0)
			notAllZeroVolts = true;
	}

	// LEDs default: Green ON, Red OFF
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Green ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Red OFF

	if ((shuntBits != 0) && (slowCounter & 0x01)) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Red blinking - balancing
	} else if (!notAllZeroVolts) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Green OFF
		if (slowCounter & 0x01)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Red blinking - no cells
	} else if ((commsTimer == COMMS_TIMEOUT) && (slowCounter & 0x01)) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Green blinkng - no CAN comunication
	}
}

void configureLTC(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin, uint32_t shuntBits) {
	uint8_t wrbuf[6] = { 0x01,              // CFGR0 (normal mode, reference ON)
			(uint8_t) (shuntBits & 0xFF), // CFGR1 (lower 8 bits of balance config)
			(uint8_t) (shuntBits >> 8),     // CFGR2 (upper 4 bits if needed)
			0x00,                           // CFGR3 (no cell masking)
			0x00,                          // CFGR4 (VUV threshold, unused here)
			0x00                          // CFGR5 (VOV threshold, unused here)
			};

	uint8_t tx[2] = { 0x80, WRCFG };   // Broadcast address + WRCFG command

	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi, wrbuf, 6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	HAL_Delay(5); // Espera tras escritura, si necesario
}

void startConversion(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin) {
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	SPIWrite(hspi, STCVAD); // Start cell voltage ADC
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
