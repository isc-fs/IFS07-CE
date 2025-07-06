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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CORRECTION		6
#define LOW_LTC_CORRECTION		6	// Calibration to account for voltage drop through buffer resistors and LTC6802 variations
#define HIGH_LTC_CORRECTION		6	// Typical value is about 6 for both of these, but may be +/-3mV in some cases
#define BASE_ID	300 // Starting ID used for BMS module messaging to/from EVMS
#define USE_29BIT_IDS	0	// Or 0 for 11-bit IDs
enum {
	BMS12_REQUEST_DATA, BMS12_REPLY1, BMS12_REPLY2, BMS12_REPLY3, BMS12_REPLY4
};

#define true 1
#define false 0

#define COMMS_TIMEOUT	16

uint8_t errorLTC1 = 0;
uint8_t errorLTC2 = 0;

// Inputs from hex rotary pot for module ID selection

#define MOD_ID_NUM1  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
#define MOD_ID_NUM2  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)  == GPIO_PIN_RESET)
#define MOD_ID_NUM4  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)  == GPIO_PIN_RESET)
#define MOD_ID_NUM8  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  == GPIO_PIN_RESET)

#define cellRegisters 18

#define maxCells 12

#define LTC6802_CS1_GPIO_PORT GPIOA
#define LTC6802_CS1_GPIO_PIN  GPIO_PIN_8

#define LTC6802_CS2_GPIO_PORT GPIOB
#define LTC6802_CS2_GPIO_PIN  GPIO_PIN_12

#define TEMPS 0 //For testing the ds18b20 readings disabling cell measurement

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

CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

uint8_t rxData[8]; //CAN receive buffer
uint8_t txData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // CAN transmit buffer

uint32_t TxMailBox;

volatile uint8_t dataRequestedL = false; // Low group, LTC #2
volatile uint8_t dataRequestedH = false; // High group, LTC #1
volatile uint8_t temperaturesRequested = false; //Temperatures
volatile uint8_t rawValuesRequested = false;
volatile uint8_t selfTestRequested = false;
volatile uint8_t selfTest2Requested = false;

uint16_t moduleID = 0;

uint32_t commsTimer = 0;

float temp[38]; // In deg C
int voltage[24]; // In millivolts

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
void readCellValues(SPI_HandleTypeDef *hspi, uint8_t cmd, uint8_t numRegisters,
		uint8_t *const buff);
uint8_t calculatePEC(uint8_t *data, uint8_t len);
void RunSelfTest(uint8_t *cellBytes1, uint8_t *cellBytes2, uint8_t testCommand);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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

	for (int n = 0; n < 24; n++)
		voltage[n] = 0;
	for (int n = 0; n < 38; n++)
		temp[n] = 0;

	uint8_t cellBytes1[cellRegisters];
	uint8_t cellBytes2[cellRegisters];
	for (int n = 0; n < cellRegisters; n++) {
		cellBytes1[n] = 0;
		cellBytes2[n] = 0;
	}
	int voltages[24][8];
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

	//Set 9bit resolution for all sensors (93.75ms max conversion time)

	for (int i = 0; i < sensor_count; i++) {
		TM_DS18B20_SetResolution(&OneWire1, device[i],
				TM_DS18B20_Resolution_9bits);
	}

	//Calculates the necesary can packets for sending all the temperatures
	uint8_t n_packets_temps = sensor_count / 8 + ((sensor_count % 8) ? 1 : 0);

	if (sensor_count > 0)
		ds18b20_flag = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		// Split up the 32-bit shuntBits variable into two 12-bit chunks for each LTC
		uint32_t shuntBitsL = 0; //shuntBits & 0x0FFF; // Lower 12 bits
		uint32_t shuntBitsH = 0; //shuntBits >> 12; // Upper 12 bits
#if !TEMPS
		//Configure LTC6802 1 (HV-)
		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_RESET);
		SPIWrite(&hspi1, WRCFG);
		SPIWrite(&hspi1, 0b00001001);
		SPIWrite(&hspi1, ((unsigned short) (shuntBitsL & 0x00FF)));
		SPIWrite(&hspi1, ((unsigned short) (shuntBitsL >> 8)));
		SPIWrite(&hspi1, 0b00000000);
		//SPIWrite(&hspi1, 0b11100000); //LTC6802-2 in HV- measures 9 cells, so cells from 10 to 12 are masked
		SPIWrite(&hspi1, 0b00000000);
		SPIWrite(&hspi1, 0b00000000);
		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_SET);

		//Delay_us(100);

		HAL_Delay(5);

		//Start voltage sampling
		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_RESET);

		SPIWrite(&hspi1, STCVAD);

		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_SET);

		HAL_Delay(15);

		//Configure LTC6802 2 (HV+)

		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_RESET);
		SPIWrite(&hspi2, WRCFG);
		SPIWrite(&hspi2, 0b00001001);
		SPIWrite(&hspi2, ((unsigned short) (shuntBitsH & 0x00FF)));
		SPIWrite(&hspi2, ((unsigned short) (shuntBitsH >> 8)));
		SPIWrite(&hspi2, 0b00000000);
		//SPIWrite(&hspi2, 0b11000000); //LTC6802-2 in HV- measures 10 cells, so cells from 11 to 12 are masked
		SPIWrite(&hspi2, 0b00000000);
		SPIWrite(&hspi2, 0b00000000);
		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_SET);

		//Delay_us(100);

		HAL_Delay(5);

		//Start voltage sampling
		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_RESET);

		SPIWrite(&hspi2, STCVAD);

		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_SET);

		HAL_Delay(15);

		//Read cell voltage registers HV-
		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_RESET);

		SPIWrite(&hspi1, RDCV);

		readCellValues(&hspi1, RDCV, cellRegisters, cellBytes1);

		HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
				GPIO_PIN_SET);

		Delay_us(100);

		//Read cell voltage registers HV+
		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_RESET);

		SPIWrite(&hspi2, RDCV);

		readCellValues(&hspi2, RDCV, cellRegisters, cellBytes2);

		HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
				GPIO_PIN_SET);
		Delay_us(100);

#endif

		//Start temperature sampling on all devices

		TM_DS18B20_StartAll(&OneWire1);
		while (!TM_DS18B20_AllDone(&OneWire1))
			;

		//Extract voltage data

		for (int n = 0; n < 12; n += 2) {
			voltages[n][counter] = (cellBytes1[n * 3 / 2]
					+ 256 * (cellBytes1[n * 3 / 2 + 1] & 0x0F)) * 3 / 2;
			voltages[n + 1][counter] =
					(((cellBytes1[n * 3 / 2 + 1] & 0xF0) >> 4)
							+ cellBytes1[n * 3 / 2 + 2] * 16) * 3 / 2;
		}

		for (int n = 0; n < 12; n += 2) {
			voltages[12 + n][counter] = (cellBytes2[n * 3 / 2]
					+ 256 * (cellBytes2[n * 3 / 2 + 1] & 0x0F)) * 3 / 2;
			voltages[12 + n + 1][counter] = (((cellBytes2[n * 3 / 2 + 1] & 0xF0)
					>> 4) + cellBytes2[n * 3 / 2 + 2] * 16) * 3 / 2;
		}

		//Extract temperature data

		for (int i = 0; i < sensor_count; i++) {
			if (TM_DS18B20_Read(&OneWire1, device[i], &temp[i])) {
				ds18b20_flag = 1;
			}
		}

		counter++;

		if (counter >= 8) {
			counter = 0;

			slowCounter++;

			if (slowCounter >= 4)
				slowCounter = 0;

			char notAllZeroVolts = false;
			for (int n = 0; n < 24; n++) // Calculate average voltage over last 8 samples, and update shunts if required
					{
				int average = 0;
				for (int c = 0; c < 8; c++)
					average += voltages[n][c] / 2;
				voltage[n] = average >> 2;

				int correction = LOW_LTC_CORRECTION;
				if (n >= 12)
					correction = HIGH_LTC_CORRECTION;

				if (voltage[n] > 0) {
					voltage[n] += correction; // With high inpedance input filters, they're reading about 8mV too low
					if (n == 0 || n == 12)
						voltage[n] -= correction / 2; // First cell has less drop due to single 3.3Kohm resistor in play
				}

				if (voltage[n] > 5000)
					voltage[n] = 0; // Probably no cells plugged in to power the LTC

				if (voltage[n] > 0)
					notAllZeroVolts = true;

				if (voltage[n] > shuntVoltage && shuntVoltage > 0)
					shuntBits |= (1 << n);
				else
					shuntBits &= ~(1 << n);
			}

			//ds18b20_flag = TM_OneWire_Reset(&OneWire1);
#if TEMPS
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, !ds18b20_flag);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
			HAL_Delay(500);
#endif
			//Update status LEDs
#if !TEMPS
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //Red LED off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //Green LED on
			if ((shuntBits != 0) & (slowCounter & 0x01))
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //RED flash if balancing
			else if (!notAllZeroVolts) { //Blink red if no cells detected
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
				if (slowCounter & 0x01)
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

			} else if (commsTimer == COMMS_TIMEOUT && slowCounter & 0x01) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //Blink green if no CAN comms
			}

#endif

		}

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
					txData[n * 2] = voltage[packet * 4 + n] >> 8;
					txData[n * 2 + 1] = voltage[packet * 4 + n] & 0xFF;
				}

				txHeader.DLC = 8;
				txHeader.IDE = CAN_ID_STD;
				txHeader.RTR = CAN_RTR_DATA;
				txHeader.StdId = moduleID + packet + 1;

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK) {
					Error_Handler();
				}

				HAL_Delay(1);

			}

		} else if (dataRequestedH) {
			dataRequestedH = false;
			commsTimer = 0;

			for (int packet = 0; packet < 3; packet++) {
				for (int n = 0; n < 4; n++) {
					txData[n * 2] = voltage[12 + packet * 4 + n] >> 8; // Top 8 bits
					txData[n * 2 + 1] = voltage[12 + packet * 4 + n] & 0xFF; // Bottom 8 bits
				}

				txHeader.DLC = 8;
				txHeader.IDE = CAN_ID_STD;
				txHeader.RTR = CAN_RTR_DATA;
				txHeader.StdId = moduleID + 10 + packet + 1;

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK) {
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK) {
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
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

				if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &TxMailBox)
						!= HAL_OK)
					Error_Handler();

				HAL_Delay(1);
			}
		}

		else {
			HAL_Delay(4);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

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
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
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
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_SPI2_Init(void)
{

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DQ_Pin|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : DQ_Pin PA8 PA10 PA15 */
  GPIO_InitStruct.Pin = DQ_Pin|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
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
	uint8_t tx[1 + numRegisters];
	uint8_t rx[1 + numRegisters];

	tx[0] = cmd;
	for (int i = 1; i < 1 + numRegisters; i++) {
		tx[i] = 0x00; // dummy bytes
	}

	HAL_SPI_TransmitReceive(hspi, tx, rx, 1 + numRegisters, HAL_MAX_DELAY);

	for (int i = 0; i < numRegisters; i++) {
		buff[i] = rx[i + 1];
	}
}

void readCellValues(SPI_HandleTypeDef *hspi, uint8_t cmd, uint8_t numRegisters,
		uint8_t *const buff) {
	SPIRead(hspi, cmd, numRegisters, buff);
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
	HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
			GPIO_PIN_RESET);
	SPIWrite(&hspi1, 0x04); // RDCV
	readCellValues(&hspi1, 0x04, 18, cellBytes1);
	HAL_GPIO_WritePin(LTC6802_CS1_GPIO_PORT, LTC6802_CS1_GPIO_PIN,
			GPIO_PIN_SET);

	HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
			GPIO_PIN_RESET);
	SPIWrite(&hspi2, 0x04);
	readCellValues(&hspi2, 0x04, 18, cellBytes2);
	HAL_GPIO_WritePin(LTC6802_CS2_GPIO_PORT, LTC6802_CS2_GPIO_PIN,
			GPIO_PIN_SET);
}

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
	while (1) {
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
