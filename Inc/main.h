/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	OK,
	TRANSMIT_TIMEOUT_CODE,
	RECEPTION_TIMEOUT_CODE,
	PAYLOAD_CRC_ERROR_CODE,
	CAD_NOT_DONE_CODE,
	CAD_NOT_DETECTED_CODE,
	RX_TIMEOUT_CODE,
	CAD_DETECTED_CODE,
	CAD_DONE_CODE,
	VALID_HEADER_CODE,
	INVALID_HEADER_CODE,
}ERROR_CODES;

/**
 * Possible states of SX1278 Module.
 */
typedef enum
{
	SLEEP = 0, 		/*!< Low-power mode. In this mode only SPI and configuration registers are accessible.\n Lora FIFO is not accessible.\n
						Note that this is the only mode permissible to switch between FSK/OOK mode and LoRa mode.*/
	STDBY,			/*!< both Crystal oscillator and Lora baseband blocks are turned on.RF part and PLLs are disabled.*/
	FSTX,			/*!< This is a frequency synthesis mode for transmission.\n The PLL selected for transmission is locked and active
						at the transmit frequency.\n The RF part is off.*/
	TX,				/*!< When activated the SX1276/77/78/79 powers all remaining blocks required for transmit, ramps the PA,
						transmits the packet and returns to Standby mode.*/
	FSRX,			/*!< This is a frequency synthesis mode for reception. The PLL selected for reception is locked and active at the
						receive frequency. The RF part is off*/
	RXCONTINUOUS,	/*!<When activated the SX1276/77/78/79 powers all remaining blocks required for reception, processing all
						received data until a new user request is made to change operating mode.*/
	RXSINGLE,		/*!< When activated the SX1276/77/78/79 powers all remaining blocks required for reception, remains in this
						state until a valid packet has been received and then returns to Standby mode.*/
	CAD,			/*!< When in CAD mode, the device will check a given channel to detect LoRa preamble signal*/
	MODE_ERROR		/*!< Invalid mode. This mode indicates that there could be an error in SPI reading or writing.*/
}Mode;

/**
 * The spread spectrum LoRaTM modulation is performed by representing each bit of payload information by multiple chips of information.\n
 * The rate at which the spread information is sent is referred to as the symbol rate (Rs),\n
 * the ratio between the nominal symbol rate and chip rate is the spreading factor and represents the number of symbols sent per bit of information.
 * The range of values accessible with the LoRaTM modem are shown in the following table.
 */
typedef enum
{
	SF_6 = 6,	/*!< Special setting, must be explicitly set by (add function here). Check datasheet for details 64 chips/symbol */
	SF_7,    	/*!< 128 chips / symbol */
	SF_8,    	/*!< 256 chips / symbol */
	SF_9,    	/*!< 512 chips / symbol */
	SF_10,   	/*!< 512 chips / symbol */
	SF_11,   	/*!< 512 chips / symbol */
	SF_12,    	/*!< 512 chips / symbol */
	SF_INVALID	/*!< Invalid value*/

}SpreadingFactor;

/**
 * Signal bandwidth enumeration, values are in [kHz].\n In the lower band (169 MHz), the 250 kHz and 500 kHz bandwidths are not supported.
 */
typedef enum
{
	BW_7_8 = 0,	/*!<7.8 kHz*/
	BW_10_4,	/*!<10.4 kHz*/
	BW_15_6,	/*!<15.6 kHz*/
	BW_20_8,	/*!<20.8 kHz*/
	BW_31_25,	/*!<31.25 kHz*/
	BW_41_7,	/*!<41.7 kHz*/
	BW_62_5,	/*!<62.5 kHz*/
	BW_125,		/*!<125 kHz*/
	BW_250,		/*!<250 kHz*/
	BW_500,		/*!<500 kHz*/
	BW_INVALID	/*!<Invalid bandwidth value*/
}Bandwidth;

/**
 * Available coding rates, others are reserved.
 */
typedef enum
{
	CR_4_5 = 1,	/*!<4/5*/
	CR_4_6,		/*!<4/6*/
	CR_4_7,		/*!<4/7*/
	CR_4_8,		/*!<4/8*/
	CR_INVALID	/*!<Invalid coding rate value*/
}CodingRate;

/**
 * This struct will help user configure parameters such as payload length, coding rate, and crc enable flags.\n
 * Note that header is only available in explicit mode. Will help in parsing of packet on receiver side.
 */
typedef struct Header
{
	uint8_t payload_length; /*!< Payload length. Value is defined in REG_PAYLOAD_LENGTH*/
	CodingRate cr;			/*!< Coding Rate. Value is defined in as bits [3-1] in REG_MODEM_CONFIG1. Can be set by setCodingRate function.*/
	uint8_t crc_enable;		/*!< Flag in header which indicates that CRC has to be checked.*/
}Header;

/**
 * LoRa packet. Consists of preamble, header, payload and optional CRC. Will help in parsing and analyzing of received data.
 */
typedef struct Packet
{
	uint16_t preamble_length; 	/*!< Preamble length, it is set by function setPreambleLength.*/
	Header header;				/*!< Header only available in explicit mode.*/
	uint8_t *payload;			/*!< Pointer to data buffer where values will be stored.*/
	uint16_t crc;				/*!< CRC value.*/
}Packet;

/**
 * @brief DIO0 pin modes of interrupt.
 *
 */
typedef enum
{
	RxDone0 = 0,			/**< Set as interrupt for RxDone on DIO0. */
	TxDone0,				/**< Set as interrupt for TxDone on DIO0. */
	CadDone0,				/**< Set as interrupt for CadDone on DIO0.*/
	DIO0_Error				/**< Invalid value for DIO0.*/
}DIO0_mode;

/**
 * @brief DIO1 pin modes for interrupt.
 *
 */
typedef enum
{
	RxTimeout1 = 0,			/**< Set as interrupt for RxDone on DIO1. */
	FhssChangeChannel1,		/**< Set as interrupt for TxDone on DIO1. */
	CadDetected1,			/**< Set as interrupt for CadDone on DIO1.*/
	DIO1_Error				/**< Invalid value for DIO1.*/
}DIO1_mode;

/**
 * @brief DIO3 pin modes for interrupt
 *
 */
typedef enum
{
	CadDone3 = 0,   		/**< Set as interrupt for CadDone on DIO3. */
	ValidHeader3,   		/**< Set as interrupt for ValidHeader on DIO3. */
	PayloadCrcError3,		/**< Set as interrupt for PayloadCrcError on DIO3. */
	DIO3_Error				/**< Invalid value for DIO3*/
}DIO3_mode;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * @brief Initializes SX1278 with passed parameters as LoRa module.
 * @param sf Spreading factor
 * @param freq Frequency
 * @return None
 */
void InitializeLora(SpreadingFactor sf,
					uint32_t freq,
					uint8_t cp,
					Bandwidth bw,
					CodingRate cr,
					uint8_t em,
					uint16_t pl,
					float timeout,
					float pwr);
/**
 * @brief Returns Mode in which LoRa module is currently in.
 * @return Mode from enum Mode.
 */
Mode getMode(void);

/**
 * @brief Sets current mode of operation for LoRa SX1278. Possible values are defined in enum Mode.
 * @param m Desired mode of operation.
 */
void writeMode(Mode m);

/**
 * @brief Resets SX1278 registers to default value.
 * @return none.
 */
void ResetLora(void);

/**
 * @brief Reads carrier antenna frequency of LoRa module.
 * @return Antenna frequency.
 */
uint32_t ReadLoraFrequency(void);

/**
 * @brief Sets the desired carrier frequency of antenna. Be careful about the frequency range, no checks for it.
 * @param freq Desired frequency in [Hz].
 */
void WriteLoraFrequency(uint32_t freq);

/**
 * @brief Gets current spreading factor, values are defined in SpreadingFactor enum.
 * @return Spreading factor.
 */
SpreadingFactor getSpreadingFactor(void);

/**
 *
 * @brief Sets desired spreading factor, values are defined in SpreadingFactor enum.\n
 * SF_6 value not supported by this function.
 * @param sf Desired spreading factor.
 */
void setSpreadingFactor(SpreadingFactor sf);

/**
 * @brief Sets SX1278 in desired mode of operation. Values defined in Mode enum.
 * @param m Mode.
 */
void writeMode(Mode m);

/**
 * @brief Returns current mode of operation for SX1278.
 * @return Mode.
 */
Mode getMode(void);

/**
 * @brief Returns transceiver power in [dBm].
 * @return Power.
 */
float getPower(void);

/**
 * @brief Sets transceiver power, minimal value -1 dBm, maximum value 17dBm.
 * @param pwr Desired transceiver power.
 */
void setPower(float pwr);

/**
 * @brief Sets current protection. Minimal value is 45 mA, maximum value is 240 mA.
 * @param current Desired current threshold.
 */
void setCurrentProtection(uint8_t current);

/**
 * @brief Reads set Current Protection threshold, will return 0 if it is not set.
 * @return Maximum allowed current
 */
uint8_t getCurrentProtection(void);

/**
 * @brief Returns current RSSI value.
 * @return Current RSSI value.
 */
uint8_t getRSSI(void);

/**
 * @brief Gets RSSI value of last packet
 * @return RSSI value of last packet.
 */
uint8_t getRSSILastPacket(void);

/**
 * @brief Sets signal bandwidth, available values defined in enum Bandwidth.
 * @param bw Desired bandwidth.
 */
void setBandwidth(Bandwidth bw);

/**
 * @brief Returns signal bit rate.
 * @return Bit rate.
 */
uint32_t getBitRate(void);

/**
 * @brief Returns signal bandwidth, values will correspond to the values in enum Bandwidth.
 * @return Signal bandwidth.
 */
Bandwidth getBandwidth(void);

/**
 * @brief Sets coding rate. Valid values are defined in CodingRate enum.
 * @param cr Coding rate.
 */
void setCodingRate(CodingRate cr);

/**
 * @brief Returns coding rate. Valid values are defined in CodingRate enum.
 * @return Coding rate.
 */
CodingRate getCodingRate(void);

/**
 * @brief Gets explicit mode bit from REG_MODEM_CONFIG_1.
 * @return Either 0 or 1.
 */
uint8_t getExplicitMode(void);

/**
 * @brief Sets explicit mode in REG_MODEM_CONFIG_1.
 * @param mode Either 0 or 1.
 */
void setExplicitMode(uint8_t mode);

/**
 * @brief Returns symbol rate (symbols per second). Value 0 is invalid value.
 * @return Rs
 */
uint16_t getSymbolRate(void);

/**
 * @brief Returns amount of air time for transmission (symbol duration).
 * @return Time in [s].
 */
float getTs(void);

/**
 * @brief Returns Preamble length, min value is 4, max is 65535
 * @return Preamble length.
 */
uint16_t getPreambleLength(void);

/**
 * @brief Sets Preamble length. Minimal value is 4, maximum value is 65535.
 * @param pl Length of Preamble.
 */
void setPreambleLength(uint16_t pl);

/**
 * @brief Retrieves SX1278 Version. Default value is 0x12.
 * @return SX1278 Version.
 */
uint8_t getSxVersion(void);

/**
 * @brief Sets desired timer timeout in [s], Maximum value is determined by multiplying 0x3FF * getTs().
 * @param timeout Desired timeout value in [s].
 */
void setTimeout(float timeout);

/**
 * @brief Returns timer timout value, used for reception, CAD and similar.
 * @return Timeout value in [s].
 */
float getTimeout(void);

/**
 * @brief Sets desired payload length, max value is 255, minimum value is 1.
 * @param length Payload length.
 */
void setPayloadLength(uint8_t length);

/**
 * @brief Returns payload length.
 * @return Payload length (in number of bytes).
 */
uint8_t getPayloadLength(void);

/**
 * @brief Enables or disables CRC check.
 * @param crc 1 for enable, 0 for disable.
 */
void setCRCEnable(uint8_t crc);

/**
 * @brief Returns CRC enable flag.
 * @return 1 if it is set, 0 if not.
 */
uint8_t getCRCEnable(void);

/**
 * @brief Creates LoRa packet with specified parameters.
 * @param pkt Packet to be formed, structure fields are defined in struct Packet.
 * @param payload pointer to buffer with data to be sent or received.
 */
void formPacket(Packet* pkt, uint8_t* payload);

/**
 * @brief Transmits the packet with assigned parameters
 * @param pkt Packet to be sent.
 */
ERROR_CODES transmit(const Packet* pkt, uint16_t timeout);

/**
 * @brief Receives LoRa packet of assigned length.
 * @param pkt Pointer to packet data to be received.
 * @param length Length of packet do be received.
 * @return
 */
ERROR_CODES receive(Packet* pkt);

/**
 * @brief Does CAD, and if it is successfull, will proceed with reception.
 *
 * @param pkt
 * @return ERROR CODES
 */
ERROR_CODES cadDetectionAndReceive(Packet *pkt);
/**
 * @brief Sets SX1278 for CAD done and CAD detection interrupt. All other interrupts are masked.
 *
 */
void setCADDetection(void);

/**
 * @brief Clears all IRQ flags.
 *
 */
void clearIRQ(void);

/**
 * @brief Sets SX1278 DIO pins and interrupt masks for transmission.
 */
void setTransmitForIRQ(void);

/**
 * @brief Try to send packet by handling TxDone interrupt on DIO0.\n
 * setTransmitForIRQ must be called before call of this function.
 * @param pkt Packet to be sent.
 * @param delay Delay in milliseconds.
 * @return OK if transmission was successful.
 */
ERROR_CODES transmitSingleThroghIRQ(Packet *pkt, uint32_t delay);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_5
#define DIO3_GPIO_Port GPIOC
#define DIO3_EXTI_IRQn EXTI9_5_IRQn
#define DIO0_Pin GPIO_PIN_6
#define DIO0_GPIO_Port GPIOC
#define DIO0_EXTI_IRQn EXTI9_5_IRQn
#define RESET_N_Pin GPIO_PIN_8
#define RESET_N_GPIO_Port GPIOC
#define NSS_Pin GPIO_PIN_9
#define NSS_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DIO1_Pin GPIO_PIN_8
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
/**FIFO read/write access*/
#define REG_FIFO						0x00
/**Operating mode & LoRaTM / FSK selection*/
#define REG_OP_MODE						0x01
/**RF Carrier Frequency, Most Significant Bits*/
#define REG_FRF_MSB						0x06
/**RF Carrier Frequency, Intermediate Bits*/
#define REG_FRF_MID						0x07
/**RF Carrier Frequency, Least Significant Bits*/
#define REG_FRF_LSB						0x08
/**PA selection and Output Power control*/
#define REG_PA_CONFIG					0x09
/**Control of PA ramp time, low phase noise PLL*/
#define	REG_PA_RAMP						0x0A
/**Over Current Protection control*/
#define REG_OCP							0x0B
/**LNA settings*/
#define REG_LNA							0x0C
/**FIFO SPI pointer*/
#define REG_FIFO_ADDR_PTR				0x0D
/**Start Tx data*/
#define	REG_FIFO_TX_BASE_ADDR			0x0E
/**Start Rx data*/
#define REG_FIFO_RX_BASE_ADDR			0x0F
/**Start address of last packet received*/
#define REG_FIFO_RX_CURR_ADDR			0x10
/**Optional IRQ flag mask*/
#define REG_IRQ_FLAGS_MASK				0x11
/**IRQ flags*/
#define REG_IRQ_FLAGS					0x12
/**Number of received bytes*/
#define	REG_RX_NB_BYTES					0x13
/**Number of valid headers received MSB*/
#define REG_RX_HEADER_CNT_VALUE_MSB		0x14
/**Number of valid headers received LSB*/
#define REG_RX_HEADER_CNT_VALUE_LSB		0x15
/**Number of valid packets received MSB*/
#define REG_RX_PACKET_CNT_VALUE_MSB		0x16
/**Number of valid packets received LSB*/
#define REG_RX_PACKET_CNT_VALUE_LSB		0x17
/**Live LoRaTM modem status*/
#define REG_MODEM_STAT					0x18
/**Espimation of last packet SNR*/
#define REG_PKT_SNR_VALUE				0x19
/**RSSI of last packet*/
#define REG_PKT_RSSI_VALUE				0x1A
/**Current RSSI*/
#define REG_RSSI_VALUE					0x1B
/**FHSS start channel*/
#define REG_HOP_CHANNEL					0x1C
/**Modem PHY config 1*/
#define REG_MODEM_CONFIG_1				0x1D
/**Modem PHY config 2*/
#define REG_MODEM_CONFIG_2				0x1E
/**Receiver timeout value*/
#define REG_SYMB_TIMEOUT_LSB			0x1F
/**Size of preamble MSB*/
#define REG_PREAMBLE_MSB				0x20
/**Size of preamble LSB*/
#define REG_PREAMBLE_LSB				0x21
/**LoRaTM payload length*/
#define REG_PAYLOAD_LENGTH				0x22
/**LoRaTM maximum payload length*/
#define REG_MAX_PAYLOAD_LENGTH			0x23
/**FHSS Hop period*/
#define REG_HOP_PERIOD					0x24
/**Address of last byte written in FIFO*/
#define REG_FIFO_RX_BYTE_ADDR			0x25
/**Modem PHY config 3*/
#define REG_MODEM_CONFIG_3				0x26
/**Estimated frequency error MSB*/
#define REG_FEI_MSB						0x28
/**Estimated frequency error MID*/
#define REG_FEI_MID						0x29
/**Estimated frequency error*/
#define REG_FEI_LSB						0x2A
/**Wideband RSSI measurement*/
#define REG_RSSI_WIDEBAND				0x2C
/**LoRa detection Optimize for SF6*/
#define REG_DETECT_OPTIMIZE				0x31
/**Invert LoRa I and Q signals*/
#define REG_INVERT_IQ					0x33
/**LoRa detection threshold for SF6*/
#define REG_DETECTION_THRESHOLD			0x37
/**LoRa Sync Word*/
#define REG_SYNC_WORD					0x39
/**Mapping of pins DIO0 to DIO3*/
#define REG_DIO_MAPPING_1				0x40
/**Mapping of pins DIO4 and DIO5, ClkOut frequency*/
#define	REG_DIO_MAPPING_2				0x41
/**Semtech ID relating the silicon revision*/
#define REG_VERSION						0x42
/**TCXO or XTAL input setting*/
#define REG_TCXO						0x4B
/**Higher power settings of the PA*/
#define REG_PA_DAC						0x4D
/**Stored temperature during the former IQ Calibration*/
#define REG_FORMER_TEMP					0x5B
/**Adjustment of the AGC thresholds*/
#define REG_AGC_REF						0x61
/**Adjustment of the AGC thresholds THRESH 1*/
#define REG_AGC_THRESH_1				0x62
/**Adjustment of the AGC thresholds THRESH 2*/
#define REG_AGC_THRESH_2				0x63
/**Adjustment of the AGC thresholds THRESH 3*/
#define REG_AGC_THRESH_3				0x64
/**Control of the PLL bandwidth*/
#define REG_PLL							0x70
/**
0 -> FSK/OOK Mode \n
1 -> LoRaTM Mode \n
This bit can be modified only in Sleep mode. A write operation on
other device modes is ignored.*/
#define LongRangeMode					7
/**
This bit operates when device is in Lora mode; if set it allows
access to FSK registers page located in address space
(0x0D:0x3F) while in LoRa mode\n
0 -> Access LoRa registers page 0x0D: 0x3F\n
1 -> Access FSK registers page (in mode LoRa) 0x0D: 0x3F\n
 */
#define AccessSharedReg					6
/**
Access Low Frequency Mode registers\n
0 -> High Frequency Mode (access to HF test registers)\n
1 -> Low Frequency Mode (access to LF test registers)\n
 */
#define LowFrequencyModeOn				3
/**
 * Device modes
000 -> SLEEP\n
001 -> STDBY\n
010 -> Frequency synthesis TX (FSTX)\n
011 -> Transmit (TX)\n
100 -> Frequency synthesis RX (FSRX)\n
101 -> Receive continuous (RXCONTINUOUS)\n
110 -> receive single (RXSINGLE)\n
111 -> Channel activity detection (CAD)
 */
#define Mode2							2
/**
 * Device modes
000 -> SLEEP\n
001 -> STDBY\n
010 -> Frequency synthesis TX (FSTX)\n
011 -> Transmit (TX)\n
100 -> Frequency synthesis RX (FSRX)\n
101 -> Receive continuous (RXCONTINUOUS)\n
110 -> receive single (RXSINGLE)\n
111 -> Channel activity detection (CAD)
111 -> Channel activity detection (CAD)
 */
#define Mode1							1
/**
 * Device modes
000 -> SLEEP\n
001 -> STDBY\n
010 -> Frequency synthesis TX (FSTX)\n
011 -> Transmit (TX)\n
100 -> Frequency synthesis RX (FSRX)\n
101 -> Receive continuous (RXCONTINUOUS)\n
110 -> receive single (RXSINGLE)\n
111 -> Channel activity detection (CAD)
 */
#define Mode0							0
/**
Selects PA output pin\n
0 -> RFO pin. Output power is limited to +14 dBm.\n
1 -> PA_BOOST pin. Output power is limited to +20 dBm
 */
#define PaSelect						7
/**
 * Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
 */
#define MaxPower6						6
/**
 * Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
 */
#define MaxPower5						5
/**
 * Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
 */
#define MaxPower4						4
/**
Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)\n
Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
 */
#define OutputPower3					3
/**
Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)\n
Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
 */
#define OutputPower2					2
/**
Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)\n
Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
 */
#define OutputPower1					1
/**
Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)\n
Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
 */
#define OutputPower0					0
/**
Rise/Fall time of ramp up/down in FSK\n
0000 -> 3.4 ms\n
0001 -> 2 ms\n
0010 -> 1 ms\n
0011 -> 500 us\n
0100 -> 250 us\n
0101 -> 125 us\n
0110 -> 100 us\n
0111 -> 62 us\n
1000 -> 50 us\n
1001 -> 40 us\n
1010 -> 31 us\n
1011 -> 25 us\n
1100 -> 20 us\n
1101 -> 15 us\n
1110 -> 12 us\n
1111 -> 10 us
 */
#define PaRamp3							3
/**
Rise/Fall time of ramp up/down in FSK\n
0000 -> 3.4 ms\n
0001 -> 2 ms\n
0010 -> 1 ms\n
0011 -> 500 us\n
0100 -> 250 us\n
0101 -> 125 us\n
0110 -> 100 us\n
0111 -> 62 us\n
1000 -> 50 us\n
1001 -> 40 us\n
1010 -> 31 us\n
1011 -> 25 us\n
1100 -> 20 us\n
1101 -> 15 us\n
1110 -> 12 us\n
1111 -> 10 us
 */
#define PaRamp2							2
/**
Rise/Fall time of ramp up/down in FSK\n
0000 -> 3.4 ms\n
0001 -> 2 ms\n
0010 -> 1 ms\n
0011 -> 500 us\n
0100 -> 250 us\n
0101 -> 125 us\n
0110 -> 100 us\n
0111 -> 62 us\n
1000 -> 50 us\n
1001 -> 40 us\n
1010 -> 31 us\n
1011 -> 25 us\n
1100 -> 20 us\n
1101 -> 15 us\n
1110 -> 12 us\n
1111 -> 10 us
 */
#define PaRamp1							1
/**
Rise/Fall time of ramp up/down in FSK\n
0000 -> 3.4 ms\n
0001 -> 2 ms\n
0010 -> 1 ms\n
0011 -> 500 us\n
0100 -> 250 us\n
0101 -> 125 us\n
0110 -> 100 us\n
0111 -> 62 us\n
1000 -> 50 us\n
1001 -> 40 us\n
1010 -> 31 us\n
1011 -> 25 us\n
1100 -> 20 us\n
1101 -> 15 us\n
1110 -> 12 us\n
1111 -> 10 us
 */
#define PaRamp0							0
/**
Enables overload current protection (OCP) for PA:\n
0  OCP disabled\n
1  OCP enabled
 */
#define OcpOn							5
/**
Trimming of OCP current:\n
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /\n
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to
240 mA)\n
Imax = 240mA for higher settings\n
Default Imax = 100mA
 */
#define OcpTrim4						4
/**
Trimming of OCP current:\n
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /\n
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to
240 mA)\n
Imax = 240mA for higher settings\n
Default Imax = 100mA
 */
#define OcpTrim3						3
/**
Trimming of OCP current:\n
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /\n
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to
240 mA)\n
Imax = 240mA for higher settings\n
Default Imax = 100mA
 */
#define OcpTrim2						2
/**
Trimming of OCP current:\n
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /\n
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to
240 mA)\n
Imax = 240mA for higher settings\n
Default Imax = 100mA
 */
#define OcpTrim1						1
/**
Trimming of OCP current:\n
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /\n
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to
240 mA)\n
Imax = 240mA for higher settings\n
Default Imax = 100mA
 */
#define OcpTrim0						0
/**
LNA gain setting:\n
000 -> not used\n
001 -> G1 = maximum gain\n
010 -> G2\n
011 -> G3\n
100 -> G4\n
101 -> G5\n
110 -> G6 = minimum gain\n
111 -> not used
 */
#define LnaGain7						7
/**
LNA gain setting:\n
000 -> not used\n
001 -> G1 = maximum gain\n
010 -> G2\n
011 -> G3\n
100 -> G4\n
101 -> G5\n
110 -> G6 = minimum gain\n
111 -> not used
 */
#define LnaGain6						6
/**
LNA gain setting:\n
000 -> not used\n
001 -> G1 = maximum gain\n
010 -> G2\n
011 -> G3\n
100 -> G4\n
101 -> G5\n
110 -> G6 = minimum gain\n
111 -> not used
 */
#define LnaGain5						5
/**
Low Frequency (RFI_LF) LNA current adjustment\n
00 -> Default LNA current\n
Other -> Reserved
 */
#define LnaBoostLf4						4
/**
Low Frequency (RFI_LF) LNA current adjustment\n
00 -> Default LNA current\n
Other -> Reserved
 */
#define LnaBoostLf3						3
/**
High Frequency (RFI_HF) LNA current adjustment\n
00 -> Default LNA current\n
11 -> Boost on, 150% LNA current
 */
#define LnaBoostHf1						1
/**
High Frequency (RFI_HF) LNA current adjustment\n
00 -> Default LNA current\n
11 -> Boost on, 150% LNA current
 */
#define LnaBoostHf0						0
/**Timeout interrupt mask: setting this bit masks the corresponding
IRQ in RegIrqFlags*/
#define RxTimeoutMask					7
/**Packet reception complete interrupt mask: setting this bit masks
the corresponding IRQ in RegIrqFlags*/
#define RxDoneMask						6
/**Payload CRC error interrupt mask: setting this bit masks the
corresponding IRQ in RegIrqFlags*/
#define PayloadCrcErrorMask				5
/**Valid header received in Rx mask: setting this bit masks the
corresponding IRQ in RegIrqFlags*/
#define ValidHeaderMask					4
/**FIFO Payload transmission complete interrupt mask: setting this
bit masks the corresponding IRQ in RegIrqFlags*/
#define TxDoneMask						3
/**CAD complete interrupt mask: setting this bit masks the
corresponding IRQ in RegIrqFlags*/
#define CadDoneMask						2
/**FHSS change channel interrupt mask: setting this bit masks the
corresponding IRQ in RegIrqFlags*/
#define FhssChangeChannelMask			1
/**Cad Detected Interrupt Mask: setting this bit masks the
corresponding IRQ in RegIrqFlags*/
#define CadDetectedMask					0
/**Timeout interrupt: writing a 1 clears the IRQ*/
#define RxTimeout						7
/**Packet reception complete interrupt: writing a 1 clears the IRQ*/
#define RxDone							6
/**Payload CRC error interrupt: writing a 1 clears the IRQ*/
#define PayloadCrcError					5
/**Valid header received in Rx: writing a 1 clears the IRQ*/
#define ValidHeader						4
/**FIFO Payload transmission complete interrupt: writing a 1 clears
the IRQ*/
#define TxDone							3
/**CAD complete: write to clear: writing a 1 clears the IRQ*/
#define CadDone							2
/**FHSS change channel interrupt: writing a 1 clears the IRQ*/
#define FhssChangeChannel				1
/**Valid Lora signal detected during CAD operation: writing a 1
clears the IRQ*/
#define CadDetected						0
/**Coding rate of last header received*/
#define RxCodingRate7					7
/**Coding rate of last header received*/
#define RxCodingRate6					6
/**Coding rate of last header received*/
#define RxCodingRate5					5
/** This bit in REG_MODEM_CONFIG3 must be set if symbol rate exceeds 16 ms*/
#define LowDataRateOptimize				3
/** Bit which enables CRC check, 0 for disable, 1 for enable.*/
#define RxPayloadCrcOn					2
/** Maximum LoRa payload length*/
#define MAX_PAYLOAD_LENGTH				255
/** Indicates that CAD has finished and detected channel activity*/
#define CAD_DONE_AND_DETECTED			((1 << CadDone) | (1 << CadDetected))
/** Flag which show if there was CRC error upon reception.*/
#define PAYLOAD_CRC_ERROR				(1 << PayloadCrcError)
/** Flag which indicates that reception timeout has occurred.*/
#define RX_TIMEOUT						(1 << RxTimeout)
/** Flag which indicates that Header vas valid upon reception.*/
#define VALID_HEADER					(1 << ValidHeader)
/** Indicates that reception is done.*/
#define RX_DONE							(1 << RxDone)
/** Maximum message length available in packet.*/
#define MAX_MSG_LENGTH					255
/** Setting this mask will mask CAD detection interrupt*/
#define CAD_DETECTED_MASK				(1 << CadDetectedMask)
/** Setting this mask will mask CAD done interrupt.*/
#define CAD_DONE_MASK					(1 << CadDoneMask)
/** Setting this mask will mask Rx Timout interrupt.*/
#define RX_TIMEOUT_MASK					(1 << RxTimeoutMask)
/** Setting this mask will mask Tx Done interrupt.*/
#define TX_DONE_MASK					(1 << TxDoneMask)
/** Setting this mask will mask Valid header interrupt.*/
#define VALID_HEADER_MASK				(1 <<  ValidHeaderMask)
/** Setting this mask will mask Payload CRC error interrupt.*/
#define PAYLOAD_CRC_ERROR_MASK			(1 << PayloadCrcErrorMask)
/** Setting this flag will mask Rx Done interrupt.*/
#define RX_DONE_MASK					(1 << RxDoneMask)
/** Setting this flag will mask FHSS channel change interrupt.*/
#define FHSS_CHANGE_CHANNEL_MASK		(1 << FhssChangeChannelMask)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
