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
#include "radio_config_Si4468.h"

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

#define Si4468_TX_STATE 0x07
#define Si4468_RX_STATE 0x08

#define ANSI_COLORS_OFF 0x00
#define ANSI_COLORS_ON 0x01

enum {
	MSG_WELCOME,
	MSG_CMD_LIST,
	MSG_ENTER_CMD,
	MSG_CMD_NOT_FOUND,
	MSG_COLORS_ACTIVE,
	MSG_COLORS_INACTIVE,
	MSG_RF_AMP_WARNING,
	MSG_RF_AMP_ON,
	MSG_RF_AMP_OFF,
	MSG_OPAMPS_ON,
	MSG_OPAMPS_OFF,
	MSG_MEAS_MODE_ON,
	MSG_MEAS_MODE_OFF,
	MSG_TX_MODE_ON,
	MSG_TX_MODE_OFF,
	MSG_RX_MODE_ON,
	MSG_RX_MODE_OFF,
	MSG_DEVICE_INFO
};

enum {
	STATE_IDLE,
	STATE_RX,
	STATE_TX,
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint32_t ADC_Raw_Results[2];

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
uint8_t ExitSignalReceived = 0;
uint8_t DenyReturnKey = 0;

uint8_t DeviceState = STATE_IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void USB_Rx_Parser(void);
uint8_t Si4468_CmdTransmitReceive(uint8_t * TxBuf, uint8_t * RxBuf, uint8_t Length);
uint8_t Si4468_CmdTransmit(uint8_t * RxBuf, uint8_t Length);
uint8_t Si4468_CmdReceive(uint8_t * RxBuf, uint8_t Length);
uint8_t Si4468_CmdReadCmdReplyWhenReady(uint8_t * RxBuf, uint8_t Length);
void Si4468_WaitForCTS(void);
void USB_CDC_TransmitPredefinedMessage(uint8_t ANSI_Color_State, uint8_t Select_Message);
uint8_t ParseUSBInputAsInteger(uint32_t * result);
double ConvertTxADCtoPower(uint32_t ADCrawResult);

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
			if (!DenyReturnKey){
				CDC_Transmit_FS((uint8_t *) "\r\n", 2);
				USB_RxDataReadyFlag = 1; // The contents of the buffer are ready to be parsed
			}
			break;
		case '\b': // BACKSPACE key (backspace for Minicom)
			if (USB_RxBufIndex > 0){
				USB_RxBufIndex--; // Take a step back in the buffer
			}
			break;
		case 0x0C:
			CDC_Transmit_FS((uint8_t *) "\e[2J\e[0;0HEnter a command: ", 27);
			break;
		case 'x':
			ExitSignalReceived = 1;
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
	char USB_MSG_TxBuf[1024];
	uint8_t Si4468_CmdTxBuf[128], Si4468_CmdRxBuf[128];
	if (USB_RxDataReadyFlag){
		if (USB_RxBufIndex == 0){
			// NO-OP
		}
		else if (Reset){
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
						USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_OFF, MSG_ENTER_CMD);
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
					USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_ON, MSG_CMD_LIST);
				}
				else{
					USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_OFF, MSG_CMD_LIST);
				}
				USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_OFF, MSG_ENTER_CMD);
				break;
			case 'm':
				uint16_t MeasuredTxPower = 0;
				double MeasuredTxPowerDouble = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Turn the forward meas. amplifier ON
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // Turn the reverse meas. amplifier ON
				HAL_Delay(100); // Wait for the amplifiers to turn on
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_OPAMPS_ON);
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_MEAS_MODE_ON);
				HAL_ADC_Start_DMA(&hadc1, ADC_Raw_Results, 2);
				HAL_Delay(100);
				HAL_ADC_Stop_DMA(&hadc1);
				//CDC_Transmit_FS((uint8_t *) "\e[A\e[A", 6);
				if (ANSI_ColorsOn){
					sprintf(USB_MSG_TxBuf, "\e[31m\tTx ADC raw: \e[37m%u    \r\n\e[32m\tRx ADC raw: \e[37m%u    \r\n", (uint16_t) ADC_Raw_Results[0], (uint16_t) ADC_Raw_Results[1]);
					CDC_Transmit_FS((uint8_t *)USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
				}
				else{
					sprintf(USB_MSG_TxBuf, "\tTx ADC raw: %u    \r\n\tRx ADC raw: %u    \r\n", (uint16_t) ADC_Raw_Results[0], (uint16_t) ADC_Raw_Results[1]);
					CDC_Transmit_FS((uint8_t *)USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
				}
				MeasuredTxPowerDouble = ConvertTxADCtoPower(ADC_Raw_Results[0]);
				MeasuredTxPower = (uint16_t) (0.5 + MeasuredTxPowerDouble); // convert to integer
				sprintf(USB_MSG_TxBuf, "\tEstimated Tx power [mW]: %d    \r\n", MeasuredTxPower);
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *)USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
				ExitSignalReceived = 0;
				DenyReturnKey = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Turn the forward meas. amplifier OFF
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // Turn the reverse meas. amplifier OFF
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_OPAMPS_OFF);
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_ENTER_CMD);
				break;
			case 'c':
				ANSI_ColorsOn ^= 0x01; // Toggle the terminal color mode
				if (ANSI_ColorsOn){
					USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_ON, MSG_COLORS_ACTIVE);
				}
				else{
					USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_ON, MSG_COLORS_INACTIVE);
				}
				USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_ON, MSG_ENTER_CMD);
				break;
			case 'R':
				HAL_Delay(1);
				CDC_Transmit_FS((uint8_t *) "Reset the device? (y/n): ", 25);
				Reset = 1;
				break;
			case 'r':
				DeviceState = STATE_RX;
				Si4468_CmdTxBuf[0] = Si4468_CHANGE_STATE;
				Si4468_CmdTxBuf[1] = Si4468_RX_STATE; // RX
				Si4468_CmdTransmitReceive(Si4468_CmdTxBuf, Si4468_CmdRxBuf, 2);
				Si4468_WaitForCTS();
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Turn the RF amplifier stage 1 OFF
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Turn the RF amplifier stage 2 OFF
				HAL_Delay(100);
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_RX_MODE_ON);
				break;
			case 't':
				DeviceState = STATE_TX;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Turn the RF amplifier stage 1 ON
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn the RF amplifier stage 2 ON
				HAL_Delay(100);
				Si4468_CmdTxBuf[0] = Si4468_CHANGE_STATE;
				Si4468_CmdTxBuf[1] = Si4468_TX_STATE; // TX
				Si4468_CmdTransmitReceive(Si4468_CmdTxBuf, Si4468_CmdRxBuf, 2);
				Si4468_WaitForCTS();
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_TX_MODE_ON);
				break;
			case 'i':
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_DEVICE_INFO);
				break;
			case 'p':
				if (RF_AmpSupplyOnWarning){
					USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_RF_AMP_WARNING);
					USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_ENTER_CMD);
					RF_AmpSupplyOnWarning = 0;
				}
				else{
					RF_AmpSupplyOn ^= 0x01; // Toggle the RF amp flag
					if (RF_AmpSupplyOn){
						USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_RF_AMP_ON);
						HAL_Delay(1);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Turn the RF amplifier stage 1 ON
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // Turn the RF amplifier stage 2 ON
					}
					else{
						USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_RF_AMP_OFF);
						HAL_Delay(1);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Turn the RF amplifier stage 1 OFF
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Turn the RF amplifier stage 2 OFF
						RF_AmpSupplyOnWarning = 1; // Reset the warning flag for next turn-on event
					}
				}
				break;
			default:
				uint32_t parsed_number = 0;
				uint8_t error = ParseUSBInputAsInteger(&parsed_number);
				if (error){
					USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_CMD_NOT_FOUND);
				}
				else{
					sprintf(USB_MSG_TxBuf, "Input was a number: %u    \r\n", (uint16_t)parsed_number);
					CDC_Transmit_FS((uint8_t *) USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
					if (DeviceState == STATE_TX){
						Si4468_CmdTxBuf[0] = Si4468_SET_PROPERTY;
						Si4468_CmdTxBuf[1] = 0x22; // Group
						Si4468_CmdTxBuf[2] = 0x04; // Number of properties
						Si4468_CmdTxBuf[3] = 0x00; // Index of the first property to be set
						Si4468_CmdTxBuf[4] = 0x08; //
						Si4468_CmdTxBuf[5] = (uint8_t) parsed_number; // TRANSCEIVER TX POWER LEVEL
						Si4468_CmdTxBuf[6] = 0x00; //
						Si4468_CmdTxBuf[7] = 0x1D; //
						Si4468_CmdTransmit(Si4468_CmdTxBuf, 8);
						Si4468_WaitForCTS();
						CDC_Transmit_FS((uint8_t *) "TX power adjusted!\r\n", 20);
					}
				}
			}
		}
		else if((strncmp((char *)USB_RxBuf, "test", 4) == 0) && (USB_RxBufIndex == 4)){
			HAL_Delay(1);
			CDC_Transmit_FS((uint8_t *) "\tEaster egg~! uwu\r\n", 19);
			HAL_Delay(1);
			CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
		}
		else{
			uint32_t parsed_number = 0;
			uint8_t error = ParseUSBInputAsInteger(&parsed_number);
			if (error){
				USB_CDC_TransmitPredefinedMessage(ANSI_ColorsOn, MSG_CMD_NOT_FOUND);
			}
			else{
				sprintf(USB_MSG_TxBuf, "Input was a number: %u    \r\n", (uint16_t)parsed_number);
				CDC_Transmit_FS((uint8_t *) USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
				if (DeviceState == STATE_TX){
					Si4468_CmdTxBuf[0] = Si4468_SET_PROPERTY;
					Si4468_CmdTxBuf[1] = 0x22; // Group
					Si4468_CmdTxBuf[2] = 0x04; // Number of properties
					Si4468_CmdTxBuf[3] = 0x00; // Index of the first property to be set
					Si4468_CmdTxBuf[4] = 0x08; //
					Si4468_CmdTxBuf[5] = (uint8_t) parsed_number & 0x7F;// TRANSCEIVER TX POWER LEVEL
					Si4468_CmdTxBuf[6] = 0x00; //
					Si4468_CmdTxBuf[7] = 0x1D; //
					Si4468_CmdTransmit(Si4468_CmdTxBuf, 8);
					Si4468_WaitForCTS();
					sprintf(USB_MSG_TxBuf, "TX power adjusted to: %u    \r\n", (uint8_t)parsed_number & 0x7F);
					CDC_Transmit_FS(USB_MSG_TxBuf, strlen(USB_MSG_TxBuf));
				}
			}
		}
		USB_RxDataReadyFlag = 0; // Clear the flag
		USB_RxBufIndex = 0; // Reset the index
	}
}

double ConvertTxADCtoPower(uint32_t ADCrawResult){
	return ((double)0.00002 * ADCrawResult * ADCrawResult) + ((double)0.0301 * ADCrawResult) + ((double)1.6865);
}

uint8_t ParseUSBInputAsInteger(uint32_t * result){
	uint8_t error = 0;
	uint32_t parsed_number = 0;
	uint32_t order = 1;
	// Try to parse the input as a decimal integer:
	for (int i = 0; i < USB_RxBufIndex; i++){
		if (USB_RxBuf[i] >= 48 && USB_RxBuf[i] <= 57){
			for (int j = USB_RxBufIndex - (i + 1); j > 0; j--){
				order *= 10;
			}
			parsed_number += (USB_RxBuf[i] - 48)*order;
			order = 1;
		}
		else{
			error = 1;
		}
	}
	*result = parsed_number;
	return error;
}

void USB_CDC_TransmitPredefinedMessage(uint8_t ANSI_Color_State, uint8_t Select_Message){
	HAL_Delay(1);
	switch(ANSI_Color_State){
	case ANSI_COLORS_OFF:
		switch(Select_Message){
		case MSG_WELCOME:
			CDC_Transmit_FS((uint8_t *) "\e[2J\e[0;0HPQ9 COM module V2.0 by 213415IACB\r\n"
					"Copyright (c): Ergo Haavasalu 2024, TalTech\r\n"
					"Enter a command (\"l\" for a list of available commands): ", 146);
			break;
		case MSG_CMD_LIST:
			CDC_Transmit_FS((uint8_t *) "\r\nLIST OF COMMANDS:\r\n"
					"\tc - Enable ANSI terminal color codes\r\n"
					"\ti - Return the system info and settings\r\n"
					"\tp - Toggle the RF power amplifier 5 V supply\r\n"
					"\tr - Put the device into receive mode\r\n"
					"\tR - Reset the device\r\n"
					"\tt - Put the device into transmit mode\r\n"
					"\tm - Display the measured Tx/Rx power levels\r\n\n", 298);
			break;
		case MSG_DEVICE_INFO:
			CDC_Transmit_FS((uint8_t *) "Device info:\r\n", 14);
			break;
		case MSG_ENTER_CMD:
			CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
			break;
		case MSG_RF_AMP_WARNING:
			CDC_Transmit_FS((uint8_t *) "WARNING! When the amplifier is turned on, the current\r\n"
					"consumption increases way above 500 mA. Make sure your USB port\r\n"
					"can handle this load. To proceed, repeat the command.\r\n", 175);
			break;
		case MSG_TX_MODE_ON:
			CDC_Transmit_FS((uint8_t *) "Transmit mode active!\r\n", 23);
			break;
		case MSG_TX_MODE_OFF:

			break;
		case MSG_RX_MODE_ON:
			CDC_Transmit_FS((uint8_t *) "Receive mode active!\r\n", 22);
			break;
		case MSG_RX_MODE_OFF:

			break;
		case MSG_CMD_NOT_FOUND:
			CDC_Transmit_FS((uint8_t *) "COMMAND NOT FOUND!\r\n"
					"Enter a command (\"l\" for a list of available commands): ", 76);
			break;
		case MSG_RF_AMP_ON:
			CDC_Transmit_FS((uint8_t *) "# RF AMPLIFIER SUPPLY ON!\r\n"
					"Enter a command: ", 44);
			break;
		case MSG_RF_AMP_OFF:
			CDC_Transmit_FS((uint8_t *) "# RF AMPLIFIER SUPPLY OFF!\r\n"
					"Enter a command: ", 45);
			break;
		}
		break;
	case ANSI_COLORS_ON:
		switch(Select_Message){
		case MSG_CMD_LIST:
			CDC_Transmit_FS((uint8_t *) "\e[36m\r\nLIST OF COMMANDS:\r\n\e[37m"
					"\tc - Enable ANSI terminal color codes\r\n"
					"\ti - Return the system info and settings\r\n"
					"\tp - Toggle the RF power amplifier 5 V supply\r\n"
					"\tr - Put the device into receive mode\r\n"
					"\tR - Reset the device\r\n"
					"\tt - Put the device into transmit mode\r\n"
					"\tm - Display the measured Tx/Rx power levels\r\n\n", 308);
			break;
		case MSG_DEVICE_INFO:
			CDC_Transmit_FS((uint8_t *) "\e[32m\r\nDevice info:\e[37m\r\n", 26);
			break;
		case MSG_ENTER_CMD:
			CDC_Transmit_FS((uint8_t *) "Enter a command: ", 17);
			break;
		case MSG_COLORS_ACTIVE:
			CDC_Transmit_FS((uint8_t *) "\e[32mANSI COLORS ACTIVATED!\e[37m\r\n", 36);
			break;
		case MSG_COLORS_INACTIVE:
			CDC_Transmit_FS((uint8_t *) "\e[31mANSI COLORS DEACTIVATED!\e[37m\r\n", 36);
			break;
		case MSG_RF_AMP_WARNING:
			CDC_Transmit_FS((uint8_t *) "\e[31m\e[1mWARNING!\e[0m\e[31m When the amplifier is turned on, the current\r\n"
					"consumption increases way above 500 mA. Make sure your USB port\r\n"
					"can handle this load. To proceed, repeat the command.\e[37m\r\n", 198);
			break;
		case MSG_TX_MODE_ON:
			CDC_Transmit_FS((uint8_t *) "\e[32mTransmit mode active!\e[37m\r\n", 33);
			break;
		case MSG_TX_MODE_OFF:

			break;
		case MSG_RX_MODE_ON:
			CDC_Transmit_FS((uint8_t *) "\e[32mReceive mode active!\e[37m\r\n", 32);
			break;
		case MSG_RX_MODE_OFF:

			break;
		case MSG_CMD_NOT_FOUND:
			CDC_Transmit_FS((uint8_t *) "\e[1m\e[31mCOMMAND NOT FOUND!\e[37m\e[0m\r\n"
					"Enter a command (\"l\" for a list of available commands): ", 94);
			break;
		case MSG_RF_AMP_ON:
			CDC_Transmit_FS((uint8_t *) "\e[1m\e[32m# RF AMPLIFIER SUPPLY ON!\e[37m\r\n\e[0m"
					"Enter a command: ", 62);
			break;
		case MSG_RF_AMP_OFF:
			CDC_Transmit_FS((uint8_t *) "\e[1m\e[31m# RF AMPLIFIER SUPPLY OFF!\e[37m\r\n\e[0m"
								"Enter a command: ", 63);
			break;
		}
		break;
	}
}

uint8_t Si4468_CmdTransmitReceive(uint8_t * TxBuf, uint8_t * RxBuf, uint8_t Length){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
	uint8_t result = HAL_SPI_TransmitReceive(&hspi1, TxBuf, RxBuf, Length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Push SPI NSS high
	if (result != HAL_OK) return 1;
	return 0;
}

uint8_t Si4468_CmdTransmit(uint8_t * TxBuf, uint8_t Length){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Pull SPI NSS low
	uint8_t result = HAL_SPI_Transmit(&hspi1, TxBuf, Length, HAL_MAX_DELAY);
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

void Si4468_WaitForCTS(void){
	uint8_t Si4468_CmdTxBuf[2], Si4468_CmdRxBuf[2];
	Si4468_CmdTxBuf[0] = Si4468_READ_CMD_BUFF;
	Si4468_CmdRxBuf[1] = 0;
	while(Si4468_CmdRxBuf[1] != 0xFF){
		Si4468_CmdTransmitReceive(Si4468_CmdTxBuf, Si4468_CmdRxBuf, 2);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Si4468_ConfigArray[] = RADIO_CONFIGURATION_DATA_ARRAY;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Set SPI NSS high
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // Turn the RF amplifier stage 2 OFF
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Turn the RF amplifier stage 1 OFF

  // ### Si4468 TRANSCEIVER STARTUP
  // Perform a POR (Power on reset)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1); // A minimum of 10 us is required
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(10); // POR should not take more than 6 ms, but let's be safe...

  /* Configure the Si4468 transceiver based on the settings in the header file
   * NB! To generate a custom config. header file, use the Silicon Lab's "Wireless Development Suite"
   *
   */
  uint16_t i = 0;
  uint16_t Si4468_ConfigArrayLength = sizeof(Si4468_ConfigArray)/sizeof(Si4468_ConfigArray[0]);
  while (Si4468_ConfigArray[i] != 0x00){ // The last byte in the autogen. array should indicate the end
	  Si4468_CmdTransmit(&Si4468_ConfigArray[i + 1], Si4468_ConfigArray[i]);
	  Si4468_WaitForCTS();
	  i += (Si4468_ConfigArray[i] + 1);
  }

  /* For testing purposes, put the Si4468 into Continuous Wave (CW) transmission mode
   *
   */
  Si4468_CmdTxBuf[0] = Si4468_SET_PROPERTY;
  Si4468_CmdTxBuf[1] = 0x20; // Group
  Si4468_CmdTxBuf[2] = 0x01; // Number of properties
  Si4468_CmdTxBuf[3] = 0x00; // Index of the first property to be set
  Si4468_CmdTxBuf[4] = 0x00; // Data
  Si4468_CmdTransmit(Si4468_CmdTxBuf, 5);
  Si4468_WaitForCTS();

  // Wait for the COM port to open:
  while(!USB_COM_Port_open){
	  //
  };
  HAL_Delay(1000);
  // Send the welcome message:
  USB_CDC_TransmitPredefinedMessage(ANSI_COLORS_OFF, MSG_WELCOME);

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, RF_AMP_EN1_Pin|RF_AMP_EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SYS_RST_Pin|TRANS_SDN_Pin|SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FWD_MEAS_AMP_EN_Pin|REV_MEAS_AMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RF_AMP_EN1_Pin RF_AMP_EN2_Pin */
  GPIO_InitStruct.Pin = RF_AMP_EN1_Pin|RF_AMP_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRANS_GPIO1_Pin TRANS_GPIO0_Pin LDO_PG_Pin */
  GPIO_InitStruct.Pin = TRANS_GPIO1_Pin|TRANS_GPIO0_Pin|LDO_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
