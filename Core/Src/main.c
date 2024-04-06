/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *	AUTO-GENERATED CODE:
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * --------------------
  *
  * USER-GENERATED CODE:
  * Copyright (c) 2024 Ergo Haavasalu, TalTech
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Si4468 Transceiver commands
#define Si4468_PART_INFO 0x01
#define Si4468_POWER_UP 0x02 // Add TCXO option (using external drive)!
#define Si4468_FUNC_INFO 0x10
#define Si4468_SET_PROPERTY 0x11
#define Si4468_GET_PROPERTY 0x12
#define Si4468_GPIO_PIN_CFG 0x13
#define Si4468_GET_ADC_READING 0x14
#define Si4468_FIFO_INFO 0x15
#define Si4468_PACKET_INFO 0x16
#define Si4468_IRCAL 0x17
#define Si4468_IRCAL_MANUAL 0x1A
#define Si4468_GET_INT_STATUS 0x20
#define Si4468_GET_PH_STATUS 0x21
#define Si4468_GET_MODEM_STATUS 0x22
#define Si4468_GET_CHIP_STATUS 0x23
#define Si4468_START_TX 0x31
#define Si4468_START_RX 0x32
#define Si4468_REQUEST_DEVICE_STATE 0x33
#define Si4468_CHANGE_STATE 0x34
#define Si4468_RX_HOP 0x36
#define Si4468_TX_HOP 0x37
#define Si4468_OFFLINE_RECAL 0x38
#define Si4468_READ_CMD_BUFF 0x44
#define Si4468_FRR_A_READ 0x50
#define Si4468_FRR_B_READ 0x51
#define Si4468_FRR_C_READ 0x53
#define Si4468_FRR_D_READ 0x57
#define Si4468_WRITE_TX_FIFO 0x66
#define Si4468_READ_RX_FIFO 0x77

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

uint8_t USB_RxBuf[1024] = {0};
uint8_t USB_RxBufIndex = 0; // Index of the last character in the USB Rx buffer
uint8_t USB_RxBufFull = 0;
uint8_t USB_RxBufOverflow = 0;

uint8_t ANSI_ColorsOn = 0;
uint8_t RF_AmpSupplyOn = 0;
uint8_t RF_AmpSupplyOnWarning = 1;

uint16_t USB_RxBufLen = 1024;
uint8_t USB_RxDataReadyFlag;

extern uint8_t USB_COM_Port_open;

uint8_t Reset = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* A callback function for USB received data handling.
 * NB! Must be kept short because it is triggered by an interrupt.
 */
void CDC_FS_RxDataReady_Callback(uint8_t * RxBuf, uint8_t Length){
	if (Length == 1){
		switch (*RxBuf){
		case '\r': // Marks the end of buffer
			CDC_Transmit_FS((uint8_t *) "\r\n", 2);
			USB_RxDataReadyFlag = 1; // The contents of the buffer are ready to be parsed
			break;
		case '\b': // BACKSPACE key (backspace for Minicom)
			if (USB_RxBufIndex > 0){
				USB_RxBufIndex--; // Take a step back in the buffer
			}
			break;
		case 0x7F: // DEL key (backspace for Picocom and Tio)
			if (USB_RxBufIndex > 0){
				USB_RxBufIndex--; // Take a step back in the buffer
			}
			break;
		default:
			if (USB_RxBufIndex < (USB_RxBufLen - 1)){
				USB_RxBuf[USB_RxBufIndex] = *RxBuf;
				USB_RxBufIndex++;
			}
			else if (USB_RxBufIndex == USB_RxBufLen - 1){
				USB_RxBuf[USB_RxBufIndex] = *RxBuf;
				USB_RxBufFull = 1;
			}
			else{
				USB_RxBufOverflow = 1;
			}
		}
	}
	else {
		// Copy the contents of the main USB buffer to a secondary buffer for parsing
		strncpy((char *) USB_RxBuf, (char *) RxBuf, Length);
		USB_RxBufLen = Length;
		USB_RxDataReadyFlag = 1;
	}
}

void USB_Rx_Parser(void){
	if (USB_RxDataReadyFlag){
		if (Reset){
			if (USB_RxBufIndex == 1){
				switch (*USB_RxBuf){
					case 'y':
						if (ANSI_ColorsOn){
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "\e[1m\e[31mRESTARTING!\r\n\e[37m\e[0m", 31);
						}
						else{
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "RESTARTING!\r\n", 13);
						}
						HAL_Delay(1);
						NVIC_SystemReset(); // Reset the device
					case 'n':
						Reset = 0;
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
						break;
					default:
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "Reset the device? (y/n): ", 25);
				}
			}
			else{
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Reset the device? (y/n): ", 25);
			}
		}
		else if (USB_RxBufIndex == 1){
			switch (*USB_RxBuf){
			case 'l':
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[36m\r\nLIST OF COMMANDS:\r\n\e[37m"
						"\tc - Enable ANSI terminal color codes\r\n"
						"\ti - Return the system info and settings\r\n"
						"\tp - Toggle the RF power amplifier 5 V supply\r\n"
						"\tr - Put the device into receive mode\r\n"
						"\tR - Reset the device\r\n"
						"\tt - Put the device into transmit mode\r\n\n"
						, 262);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\r\nLIST OF COMMANDS:\r\n"
							"\tc - Enable ANSI terminal color codes\r\n"
							"\ti - Return the system info and settings\r\n"
							"\tp - Toggle the RF power amplifier 5 V supply\r\n"
							"\tr - Put the device into receive mode\r\n"
							"\tR - Reset the device\r\n"
							"\tt - Put the device into transmit mode\r\n\n"
							, 252);
				}
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
				break;
			case 'c':
				ANSI_ColorsOn ^= 0x01; // Toggle the terminal color mode
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[32mANSI COLORS ACTIVATED!\e[37m\r\n", 36);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[31mANSI COLORS DEACTIVATED!\e[37m\r\n", 36);
				}
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
				break;
			case 'R':
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Reset the device? (y/n): ", 25);
				Reset = 1;
				break;
			case 'r':
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[32mReceive mode active!\e[37m\r\n", 32);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "Receive mode active!\r\n", 22);
				}
				break;
			case 't':
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[32mTransmit mode active!\e[37m\r\n", 33);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "Transmit mode active!\r\n", 23);
				}
				break;
			case 'i':
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[32m\r\nDevice info mode!\e[37m\r\n", 31);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "Device info mode!\r\n", 19);
				}
				break;
			case 'p':
				if (RF_AmpSupplyOnWarning){
					if (ANSI_ColorsOn){
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "\e[31m\e[1mWARNING!\e[0m\e[31m When the amplifier is turned on, the current\r\n"
								"consumption increases way above 500 mA. Make sure your USB port\r\n"
								"can handle this load. To proceed, repeat the command.\e[37m\r\n"
								, 198);
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
					}
					else{
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "WARNING! When the amplifier is turned on, the current\r\n"
								"consumption increases way above 500 mA. Make sure your USB port\r\n"
								"can handle this load. To proceed, repeat the command.\r\n"
								, 175);
						HAL_Delay(1);
						CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
					}
					RF_AmpSupplyOnWarning = 0;
				}
				else{
					RF_AmpSupplyOn ^= 0x01; // Toggle the RF amp flag
					if (RF_AmpSupplyOn){
						if (ANSI_ColorsOn){
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "\e[1m\e[32m# RF AMPLIFIER SUPPLY ON!\e[37m\r\n\e[0m", 45);
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
						}
						else{
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "# RF AMPLIFIER SUPPLY ON!\r\n", 27);
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
						}
						HAL_Delay(1);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Turn the RF amplifier ON
					}
					else{
						if (ANSI_ColorsOn){
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "\e[1m\e[31m# RF AMPLIFIER SUPPLY OFF!\e[37m\r\n\e[0m", 46);
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
						}
						else{
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "# RF AMPLIFIER SUPPLY OFF!\r\n", 28);
							HAL_Delay(1);
							CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
						}
						HAL_Delay(1);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Turn the RF amplifier OFF
						RF_AmpSupplyOnWarning = 1; // Reset the warning flag for next turn-on event
					}
				}
				break;
			default:
				if (ANSI_ColorsOn){
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "\e[1m\e[31mCOMMAND NOT FOUND!\e[37m\e[0m\r\n", 38);
				}
				else{
					HAL_Delay(1);
					CDC_Transmit_FS((uint8_t *) "COMMAND NOT FOUND!\r\n", 20);
				}
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Enter a command (\"l\" for a list of available commands): ", 56);
			}
		}
		else if((strncmp((char *)USB_RxBuf, "test", 4) == 0) && (USB_RxBufIndex == 4)){
			HAL_Delay(1);
			CDC_Transmit_FS((uint8_t *) "\r\nTested!\r\n", 11);
			HAL_Delay(1);
			CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
		}
		else{
			if (ANSI_ColorsOn){
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "\e[1m\e[31mCOMMAND NOT FOUND!\e[37m\e[0m\r\n", 38);
			}
			else{
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "COMMAND NOT FOUND!\r\n", 20);
			}
			HAL_Delay(1);
			CDC_Transmit_FS((uint8_t *) "Enter a command (\"l\" for a list of available commands): ", 56);
		}
		USB_RxDataReadyFlag = 0; // Clear the flag
		USB_RxBufIndex = 0; // Reset the index
	}
}

uint8_t Si4468_CmdTransmitReceive(uint8_t * TxBuf, uint8_t * RxBuf, uint8_t Length){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
	uint8_t result = HAL_SPI_TransmitReceive(&hspi1, TxBuf, RxBuf, Length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Push SPI NSS high
	if (result != HAL_OK) return 1;
	return 0;
}

uint8_t Si4468_CmdTransmit(uint8_t * RxBuf, uint8_t Length){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
	uint8_t result = HAL_SPI_Transmit(&hspi1, RxBuf, Length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Push SPI NSS high
	if (result != HAL_OK) return 1;
	return 0;
}

uint8_t Si4468_CmdReceive(uint8_t * RxBuf, uint8_t Length){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
	uint8_t result = HAL_SPI_Receive(&hspi1, RxBuf, Length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Push SPI NSS high
	if (result != HAL_OK) return 1;
	return 0;
}

uint8_t Si4468_CmdReadCmdReplyWhenReady(uint8_t * RxBuf, uint8_t Length){
	uint8_t CmdTxBuf[2] = {Si4468_READ_CMD_BUFF, 0};
	uint8_t CmdRxBuf[2] = {0, 0};
	uint8_t result;
	while(1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
		result = HAL_SPI_TransmitReceive(&hspi1, CmdTxBuf, CmdRxBuf, 2, HAL_MAX_DELAY);
		if (CmdRxBuf[1] == 0xFF) break;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Push SPI NSS high
		HAL_Delay(1);
	}
	result += HAL_SPI_Receive(&hspi1, RxBuf, Length, HAL_MAX_DELAY);
	if (result != HAL_OK) return 1;
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Si4468_TxBuf[64] = {0}; // For writing to Si4468 FIFO registers
	uint8_t Si4468_RxBuf[64] = {0}; // For reading from Si4468 FIFO registers
	uint8_t Si4468_CmdTxBuf[128], Si4468_CmdRxBuf[128];
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
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Set SPI NSS high
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Turn the amplifier OFF (drive EN low)

  // ### Si4468 TRANSCEIVER STARTUP
  // Perform a POR (Power on reset)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1); // A minimum of 10 us is required
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(10); // POR should not take more than 6 ms, but let's be safe...
  // Send a POWER_UP command to Si4468
  Si4468_CmdTxBuf[0] = Si4468_POWER_UP;
  Si4468_CmdTxBuf[1] = 0x01;
  Si4468_CmdTxBuf[2] = 0x01; // Using an external drive (TCXO) as clock source

  /* TCXO frequency is 30 MHz (30'000'000 Hz; in hex: 0x01C9C380).
   * NB! The byte order of the following hex value is reversed due to Endianness!
   * Si4468 expects most significant byte first, but STM32L412 seems to
   * be Little-Endian.
   */
  *((uint32_t *) &Si4468_CmdTxBuf[3]) = 0x80C3C901;
  Si4468_CmdTransmit(Si4468_CmdTxBuf, 7);

  /* Wait until Si4468 has finished the power-up sequence.
   * The transceiver will be ready when the returned value of
   * CTS (clear to send) byte will be equal to 0xFF
   */
  Si4468_CmdTxBuf[0] = Si4468_READ_CMD_BUFF;
  Si4468_CmdRxBuf[1] = 0;
  while(Si4468_CmdRxBuf[1] != 0xFF){
	  Si4468_CmdTransmitReceive(Si4468_CmdTxBuf, Si4468_CmdRxBuf, 2);
  }

  /* Read the "part info" of the device to make sure the initialization worked
   * and we have a good SPI communication going...
   */
  Si4468_CmdTxBuf[0] = Si4468_PART_INFO;
  Si4468_CmdRxBuf[1] = 0;
  Si4468_CmdTransmitReceive(Si4468_CmdTxBuf, Si4468_CmdRxBuf, 2);
  Si4468_CmdReadCmdReplyWhenReady(Si4468_CmdRxBuf, 8);

  // Wait for the COM port to open:
  while(!USB_COM_Port_open){
	  //
  };
  HAL_Delay(1000);
  // Send the welcome message:
  CDC_Transmit_FS((uint8_t *) "PQ9 COM module V1.1 by 213415IACB\r\n", 35);
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) "Copyright (c): Ergo Haavasalu 2024, TalTech\r\n", 45);
  HAL_Delay(1);
  CDC_Transmit_FS((uint8_t *) "Enter a command (\"l\" for a list of available commands): ", 56);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  USB_Rx_Parser();
	  /*
	  sprintf(USB_TxBuf, "%u\r\n", counter);
	  counter++;
	  CDC_Transmit_FS((uint8_t *)USB_TxBuf, strlen(USB_TxBuf));
	  HAL_Delay(5000);
	  */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&hlpuart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOST_EN_GPIO_Port, BOOST_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SYS_RST_Pin|TRANS_SDN_Pin|SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FWD_MEAS_AMP_EN_Pin|REV_MEAS_AMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOOST_EN_Pin */
  GPIO_InitStruct.Pin = BOOST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOST_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SYS_RST_Pin TRANS_SDN_Pin */
  GPIO_InitStruct.Pin = SYS_RST_Pin|TRANS_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRANS_NIRQ_Pin */
  GPIO_InitStruct.Pin = TRANS_NIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRANS_NIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LDO_PG_Pin */
  GPIO_InitStruct.Pin = LDO_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LDO_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FWD_MEAS_AMP_EN_Pin REV_MEAS_AMP_EN_Pin */
  GPIO_InitStruct.Pin = FWD_MEAS_AMP_EN_Pin|REV_MEAS_AMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
