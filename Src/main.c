/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_NUMBER_OF_COMMANDS		64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const uint32_t FREQ_DIVISION = 524288;
const uint32_t FREQ_MULTIPLIER = 32000000; // clock frequency
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint8_t DIO0_int = 0;
volatile uint8_t DIO1_int = 0;
volatile uint8_t DIO3_int = 0;
volatile uint8_t message_received = 0;
ERROR_CODES global_status = RECEPTION_TIMEOUT_CODE;
//uint32_t cad_detected = 0;
//uint32_t valid_header = 0;
//uint32_t rx_done = 0;

static char DmaTxBuffer[MAX_UART_SIZE];
static char DmaRxBuffer[MAX_UART_SIZE];;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief SPI interface in order to write one byte to given address
 * @param address Register address
 * @param data Pointer to data to be written.
 */
static void SPIWriteSingle(uint8_t address, uint8_t *data);
/**
 * @brief SPI interface in order to write multiple bytes of data to given address. Address increments for each byte to be written.
 * @param address Address of registers to be written.
 * @param data Buffer to be written.
 * @param data_size Size of buffer.
 */
static void SPIWriteBurst(uint8_t address, uint8_t *data, uint8_t data_size);
/**
 * @brief SPI interface to retrieve value from given address, data returned is one byte.
 * @param address Register address.
 * @return Data stored in given address.
 */
static uint8_t SPIReadSingle(uint8_t address);
/**
 * @brief SPI interface to retrieve multiple values starting from given address. Address increments for each byte.
 * @param address Starting register address
 * @param data Values are stored in buffer
 * @param data_length Size of data to be retrieved in bytes.
 */
static void SPIReadBurst(uint8_t address, uint8_t *data, uint8_t data_length);
/**
 * @brief Converts values from Bandwidth enum to real frequency values. Returns 0 if assigned value is invalid.
 * @param bw Bandwidth number form Bandwidth enum.
 * @return Frequency for given Bandwidth number.
 */
static uint32_t convertBandwidth(Bandwidth bw);

/**
 * @brief Returns value in which DIO0 is set as interrupt.
 * @return RxDone, TxDone, CadDone or error state for DIO0.
 */
static DIO0_mode getDIO0Mode(void);

/**
 * @brief Returns value in which DIO1 is set as interrupt.
 * @return	RxTimout, FhssChangeChannel, CadDetected1 or error state for DIO1
 */
static DIO1_mode getDIO1Mode(void);

/**
 * @brief Returns value in which DIO3 is set as interrupt.
 * @return CadDone, ValidHeader, PayloadCrcError or error state for DIO3.
 */
static DIO3_mode getDIO3Mode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SPIWriteSingle(uint8_t address, uint8_t *data)
{
	uint8_t addr = address | 0x80;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &addr, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, data, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static void SPIWriteBurst(uint8_t address, uint8_t *data, uint8_t data_size)
{
	uint8_t addr = address | 0x80;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &addr, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, data, data_size, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static uint8_t SPIReadSingle(uint8_t address)
{
	uint8_t read_data = 0x00;
	uint8_t addr = address;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &addr, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi1, &read_data, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	return read_data;
}

static void SPIReadBurst(uint8_t address, uint8_t *data, uint8_t data_length)
{
	uint8_t addr = address;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &addr, 1, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi1, data, data_length, 500);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

}

static uint32_t convertBandwidth(Bandwidth bw)
{
	uint32_t ret = 0;

	switch(bw)
	{
		case BW_7_8:
			ret = 7800;
			break;

		case BW_10_4:
			ret = 10400;
			break;

		case BW_15_6:
			ret = 15600;
			break;

		case BW_20_8:
			ret = 20800;
			break;

		case BW_31_25:
			ret = 31250;
			break;

		case BW_41_7:
			ret = 41700;
			break;

		case BW_62_5:
			ret = 62500;
			break;

		case BW_125:
			ret = 125000;
			break;

		case BW_250:
			ret = 250000;
			break;

		case BW_500:
			ret = 500000;
			break;

		default:
			ret = 0;
	}

	return ret;
}

static DIO0_mode getDIO0Mode(void)
{
	DIO0_mode ret = DIO0_Error;
	uint8_t read = SPIReadSingle(REG_DIO_MAPPING_1);

	read &= 0xC0;
	read >>= 6;

	switch(read)
	{
		case 0x00:
			ret = RxDone0;
		break;

		case 0x01:
			ret = TxDone0;
		break;

		case 0x02:
			ret = CadDone0;
		break;

		default:
			ret = DIO0_Error;
		break;
	}

	return ret;
}


static DIO1_mode getDIO1Mode(void)
{
	DIO1_mode ret = DIO1_Error;
	uint8_t read = SPIReadSingle(REG_DIO_MAPPING_1);

	read &= 0x30;
	read >>= 4;

	switch(read)
	{
		case 0x00:
			ret = RxTimeout1;
		break;

		case 0x01:
			ret = FhssChangeChannel1;
		break;

		case 0x02:
			ret = CadDetected1;
		break;

		default:
			ret = DIO1_Error;
		break;
	}

	return ret;
}

static DIO3_mode getDIO3Mode(void)
{
	DIO3_mode ret = DIO3_Error;
	uint8_t read = SPIReadSingle(REG_DIO_MAPPING_1);

	read &= 0x03;

	switch(read)
	{
		case 0x00:
			ret = CadDone3;
		break;

		case 0x01:
			ret = ValidHeader3;
		break;

		case 0x02:
			ret = PayloadCrcError3;
		break;

		default:
			ret = DIO3_Error;
		break;
	}

	return ret;
}

static void splitAddress(char *addr_string, const uint8_t address)
{
	if (address < 10) {
		addr_string[0] = '0';
		addr_string[1] = '0';
		sprintf(&addr_string[2], "%d", address);
	}
	else if (address < 100) {
		addr_string[0] = '0';
		sprintf(&addr_string[1], "%d", address);
	}
	else {
		sprintf(addr_string, "%d", address);
	}
}

void ResetLora(void)
{
	HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(10); // 100 us is enough, we wait for 1 ms
	HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin, GPIO_PIN_SET);
	HAL_Delay(100);	// 5ms is enough, we wait for 10

}

uint32_t ReadLoraFrequency(void)
{
	uint8_t data[3];
	uint32_t ret = 0; // should be handled by software in future
	float freq = 0;

	SPIReadBurst(REG_FRF_MSB, data, 3);

	freq = (data[0] << 16) | (data[1] << 8) | data[2];
	freq /= FREQ_DIVISION;
	freq *= FREQ_MULTIPLIER;

	ret = (uint32_t)freq; // explicit casting

	return ret;
}

void WriteLoraFrequency(uint32_t freq)
{
	uint8_t data[3] = {0};
	float a = freq;
	uint32_t read_freq = ReadLoraFrequency();

	// default value
	if ((freq == 433000000) && (read_freq == freq))
	{
		return;
	}

	a = (float)freq/FREQ_MULTIPLIER;
	a *= FREQ_DIVISION;

	freq = (uint32_t)a;

	data[0] = (freq & 0x00ff0000) >> 16;
	data[1] = (freq & 0x0000ff00) >> 8;
	data[2] = (freq & 0x000000ff);

	SPIWriteBurst(REG_FRF_MSB, data, 3);
}

SpreadingFactor getSpreadingFactor(void)
{
	SpreadingFactor sf = SF_INVALID;
	uint8_t data = SPIReadSingle(REG_MODEM_CONFIG_2);
	data &= 0xF0;
	data >>= 4;

	switch(data) {
		case 0x06:
			sf = SF_6;
		break;

		case 0x07:
			sf = SF_7;
		break;

		case 0x08:
			sf = SF_8;
		break;

		case 0x09:
			sf = SF_9;
		break;

		case 0x0A:
			sf = SF_10;
		break;

		case 0x0B:
			sf = SF_11;
		break;

		case 0x0C:
			sf = SF_12;
		break;

		default:
			sf = SF_INVALID;
		break;
	}

	return sf;
}

void setSpreadingFactor(SpreadingFactor sf)
{
	uint8_t data;
	uint8_t read;

	if (sf == SF_6) {
		sf = SF_7;
	}

	read =  getSpreadingFactor();

	// default value
	if ((read == sf) && (read == SF_7))
	{
		return;
	}

	data = (sf << 4) | (read & 0x0F);
	SPIWriteSingle(REG_MODEM_CONFIG_2, &data);
}

void writeMode(Mode m)
{
	Mode current_mode = MODE_ERROR;
	uint8_t reg_val;

	current_mode = getMode();

	if (current_mode == m) {
		return;
	}
	else {

		reg_val = SPIReadSingle(REG_OP_MODE);
		reg_val &= ~(1 << Mode2);
		reg_val &= ~(1 << Mode1);
		reg_val &= ~(1 << Mode0);

		switch(m) {
			case SLEEP:
				reg_val |= 0x00;
			break;

			case STDBY:
				reg_val |= 0x01;
			break;

			case FSTX:
				reg_val |= 0x02;
			break;

			case TX:
				reg_val |= 0x03;
			break;

			case FSRX:
				reg_val |= 0x04;
			break;

			case RXCONTINUOUS:
				reg_val |= 0x05;
			break;

			case RXSINGLE:
				reg_val |= 0x06;
			break;

			case CAD:
				reg_val |= 0x07;
			break;

			default:
				return;
			}

		SPIWriteSingle(REG_OP_MODE, &reg_val);

		}
}

Mode getMode(void)
{
	Mode ret = MODE_ERROR;
	uint8_t reg_value = 0xff;
	uint8_t mask = 0x03;

	reg_value = SPIReadSingle(REG_OP_MODE);
	reg_value &= mask;

	ret = reg_value;

	return ret;
}

float getPower(void)
{
	uint8_t read, maxPower, outputPower, Pa;
	float ret = 0;
	float MaximumPower;

	read = SPIReadSingle(REG_PA_CONFIG);

	Pa = (read >> 7) & 0x01;
	outputPower = read & 0x0f;

	if (Pa) {
		ret = 17 - (15 - outputPower);
	}
	else {
		maxPower = (read & 0x70) >> 4;
		MaximumPower = (float)(10.8 + 0.6*maxPower);
		ret = MaximumPower - (15 - outputPower);
	}

	return ret;
}

void setPower(float pwr)
{
	uint8_t read = SPIReadSingle(REG_PA_CONFIG);
	uint8_t data, Pa = 0, outputPower = 0, maxPower = 0;
	const float Pmax = 13.2; //default value from registers

	maxPower = read & 0x70;
	Pa = (read >> PaSelect) & 0x01;

	if (pwr > 20) {
		pwr = 20;
		Pa = 1;
	}
	else if ((pwr > 14) && (pwr <= 20)) {
		Pa = 1;
	}
	else if (pwr < -1) {
		pwr = -1;
		Pa = 0;
	}

	if (Pa) {
		outputPower = (uint8_t)(pwr - 2);
	}
	else {
		outputPower = (uint8_t)((float)(pwr + 15 - Pmax));
		if (outputPower > 15) {
			outputPower = 15;
		}
	}

	outputPower &= 0x0f;

	data = (Pa << 7) | maxPower | outputPower;
	SPIWriteSingle(REG_PA_CONFIG, &data);
}

void setCurrentProtection(uint8_t current)
{
	uint8_t OCP_ON = (1 << OcpOn);
	uint8_t OCP_TRIM = 0, data = 0;
	float intermediate = 0;

	if (current < 45) {
		current = 45;
	}
	else if (current > 240) {
		current = 240;
	}

	if (current < 120) {
		intermediate = (float)((current - 45)/5);
		OCP_TRIM = (uint8_t)intermediate;
	}
	else if ((current > 130) && (current <= 240)) {
		intermediate = (float)((current + 30)/10);
		OCP_TRIM = (uint8_t)intermediate;
	}

	data = OCP_ON | OCP_TRIM;
	SPIWriteSingle(REG_OCP, &data);

}

uint8_t getCurrentProtection(void)
{
	uint8_t ret = 0; // 0 means no current protection
	uint8_t read = 0, OCP_TRIM = 0;

	read = SPIReadSingle(REG_OCP);

	if (((read & 0x20) >> OcpOn) == 0) {
		return ret;
	}

	OCP_TRIM = read & 0x1f;

	if (OCP_TRIM <= 15) {
		ret = (uint8_t)(45 + (OCP_TRIM*5));
	}
	else if ((OCP_TRIM > 15) && (OCP_TRIM <=27)) {
		ret = (uint8_t)(-30 + (10*OCP_TRIM));
	}
	else {
		ret = 0;
	}

	return ret;
}

int getRSSILastPacket(void)
{
	uint8_t low_frequency, read;
	int ret;

	low_frequency = SPIReadSingle(REG_OP_MODE);

	low_frequency &= 0x08;
	low_frequency >>= LowFrequencyModeOn;

	read = SPIReadSingle(REG_PKT_RSSI_VALUE);

	if (low_frequency) {
		ret = -164 + read;
	}
	else {
		ret = -157 + read;
	}

	return ret;
}

int getRSSI(void)
{
	uint8_t low_frequency, read;
	int ret;

	low_frequency = SPIReadSingle(REG_OP_MODE);

	low_frequency &= 0x08;
	low_frequency >>= LowFrequencyModeOn;

	read = SPIReadSingle(REG_RSSI_VALUE);

	if (low_frequency) {
		ret = -164 + read;
	}
	else {
		ret = -157 + read;
	}

	return ret;
}

void setBandwidth(Bandwidth bw)
{
	uint8_t read, data;

	if ((bw >= BW_INVALID) || (bw < BW_7_8)) {
		return;
	}

	read = SPIReadSingle(REG_MODEM_CONFIG_1);
	read &= 0x0f;

	data = (bw << 4) | read;

	SPIWriteSingle(REG_MODEM_CONFIG_1, &data);
}

Bandwidth getBandwidth(void)
{
	uint8_t read = SPIReadSingle(REG_MODEM_CONFIG_1);
	Bandwidth ret = BW_INVALID;

	read &= 0xf0;
	read >>= 4;

	if (read < BW_INVALID) {
		ret = read;
	}

	return ret;
}

void setCodingRate(CodingRate cr)
{
	uint8_t read, data;

	if ((cr >= CR_INVALID) || (cr < CR_4_5)) {
		return;
	}

	read = SPIReadSingle(REG_MODEM_CONFIG_1);
	read &= 0xf1;

	data = read | (cr << 1);

	SPIWriteSingle(REG_MODEM_CONFIG_1, &data);

}

CodingRate getCodingRate(void)
{
	uint8_t read;
	CodingRate ret = CR_INVALID;

	read = SPIReadSingle(REG_MODEM_CONFIG_1);
	read &= 0x0E;
	read >>= 1;

	if ((read >= CR_4_5) && (read < CR_INVALID)) {
		ret = read;
	}

	return ret;
}

uint8_t getExplicitMode(void)
{
	uint8_t read;

	read = SPIReadSingle(REG_MODEM_CONFIG_1);
	read &= 0x01;

	return read;
}

void setExplicitMode(uint8_t mode)
{
	uint8_t read, data;

	if ((mode != 0) || (mode != 1)) {
		return;
	}

	read = SPIReadSingle(REG_MODEM_CONFIG_1);
	read &= 0xFE;

	data = read | mode;
	SPIWriteSingle(REG_MODEM_CONFIG_1, &data);

}

uint16_t getSymbolRate(void)
{
	uint16_t ret = 0;
	uint16_t spreading_factor;
	uint32_t bandwidth;
	Bandwidth bw = getBandwidth();
	SpreadingFactor sf = getSpreadingFactor();

	if ((sf >= SF_6) && (sf < SF_INVALID)) {
		spreading_factor = 2 << (sf - 1);
	}
	else {
		spreading_factor = 0;
	}

	bandwidth = convertBandwidth(bw);

	if ((spreading_factor == 0) || (bandwidth == 0)){
		return ret;
	}

	ret = (uint16_t)((float)(bandwidth/spreading_factor));

	return ret;
}

uint32_t getBitRate(void)
{
	uint32_t ret = 0;
	SpreadingFactor sf = getSpreadingFactor();
	uint32_t bandwidth = convertBandwidth(getBandwidth());
	uint16_t spreading_factor = 2 << (sf - 1);
	CodingRate cr = getCodingRate();

	ret = (uint32_t)((float)((sf * (bandwidth / spreading_factor)) * 4 / (4 + cr)) );

	return ret;
}

float getTs(void)
{
	float ret = 0;
	float sr = getSymbolRate();

	if (sr > 0) {
		ret = (float)(1/sr);
	}

	return ret;
}

void setTimeout(float timeout)
{
	float max_timeout = (float)(0x3FF * getTs());
	uint16_t symb_timeout = 0;
	uint8_t read, data;

	if (timeout > max_timeout) {
		timeout = max_timeout;
	}
	else if (timeout <= 0) {
		return;
	}

	read = SPIReadSingle(REG_MODEM_CONFIG_3);

	if (getTs() > 0.016) {
		data = read | (1 << LowDataRateOptimize);
	} else {
		data = read & ~(1 << LowDataRateOptimize);
	}

	SPIWriteSingle(REG_MODEM_CONFIG_3, &data);

	symb_timeout = (uint16_t)(timeout/(getTs()));

	read = SPIReadSingle(REG_MODEM_CONFIG_2);
	data = read & 0xFC;

	data = read | ( (symb_timeout & 0x0300) >> 8) ;
	SPIWriteSingle(REG_MODEM_CONFIG_2, &data);
	data = (uint8_t)(symb_timeout & 0x00FF);
	SPIWriteSingle(REG_SYMB_TIMEOUT_LSB, &data);
}

float getTimeout(void)
{
	float ret = 0;
	uint8_t lsb, msb, read;
	uint16_t val;

	lsb = SPIReadSingle(REG_SYMB_TIMEOUT_LSB);
	read = SPIReadSingle(REG_MODEM_CONFIG_2);
	msb = read & 0x03;

	val = (msb << 8) | lsb;

	ret = (float)( val * getTs());

	return ret;
}

uint16_t getPreambleLength(void)
{
	uint16_t ret = 0;
	uint8_t lsb = SPIReadSingle(REG_PREAMBLE_LSB);
	uint8_t msb = SPIReadSingle(REG_PREAMBLE_MSB);

	ret = ((msb << 8) | lsb) + 4;

	return ret;
}

void setPreambleLength(uint16_t pl)
{
	if ((pl < 6) || (pl > 65539)) {
		return;
	}

	pl -= 4;

	uint8_t lsb = pl & 0x00ff;
	uint8_t msb = (pl & 0xff00) >> 8;
	uint8_t data[2] = {msb, lsb};

	SPIWriteBurst(REG_PREAMBLE_MSB, data, 2);
}

uint8_t getSxVersion(void)
{
	return SPIReadSingle(REG_VERSION);
}

uint8_t getCRCEnable(void)
{
	uint8_t ret;
	uint8_t read = SPIReadSingle(REG_MODEM_CONFIG_2);

	read &= 0x04;
	ret = read >> RxPayloadCrcOn;

	return ret;
}

void setCRCEnable(uint8_t crc)
{
	uint8_t data, read;

	if ((crc < 0) || (crc > 1)) {
		return;
	}

	read =  SPIReadSingle(REG_MODEM_CONFIG_2);

	if (crc == 1) {
		data = read | (1 << RxPayloadCrcOn);
	}
	else {
		data = read & ~(1 << RxPayloadCrcOn);
	}

	SPIWriteSingle(REG_MODEM_CONFIG_2, &data);
}

uint8_t getPayloadLength(void)
{
	return SPIReadSingle(REG_PAYLOAD_LENGTH);
}

void setPayloadLength(uint8_t length)
{
	uint8_t read;

	read = SPIReadSingle(REG_MAX_PAYLOAD_LENGTH);

	if (length > read) {
		length = read;
	}
	else  if (length < 1) {
		length = 1;
	}

	SPIWriteSingle(REG_PAYLOAD_LENGTH, &length);
}

void formPacket(Packet* pkt, uint8_t* payload , const uint8_t length, const uint8_t dst_address)
{
	if (length > 0) {
		uint8_t min;
		min = (getPayloadLength() >= length ? length : getPayloadLength());
		pkt->length = min;
		setPayloadLength(min);
		pkt->dst_address = dst_address;
	}
	else {
		pkt->length = getPayloadLength();
		pkt->dst_address = 0; // does not matter much for receiver.
	}

	pkt->payload = payload;
	pkt->src_address = MY_ADDRESS;

}

void setTransmitForIRQ(void)
{
	uint8_t data = 0xFF; // masks all interrupts

	writeMode(STDBY); // go in standby to set registers and DIO
	// Mask all other interrupts except TxDone
	data &= ~TX_DONE_MASK; // Enable interrupt only For TxDone
	SPIWriteSingle(REG_IRQ_FLAGS_MASK, &data);

	data = 0x7F; // DIO0 =  TxDone mode
	SPIWriteSingle(REG_DIO_MAPPING_1, &data);
	clearIRQ(); // clear any pending interrupts
	// stay in Standby because message has to be preapred.
}

ERROR_CODES transmitSingleThroghIRQ(Packet *pkt, uint32_t delay, const uint8_t address)
{
	ERROR_CODES ret = TRANSMIT_TIMEOUT_CODE;
	uint8_t read;
	char src_address[3], dst_address[3];

	writeMode(STDBY);
	read = SPIReadSingle(REG_FIFO_TX_BASE_ADDR);	//Gets the base address of FIFO Transmit.
	SPIWriteSingle(REG_FIFO_ADDR_PTR, &read);		//Writes that address for start of FIFO pointer
	splitAddress(src_address, pkt->src_address);
	SPIWriteBurst(REG_FIFO, (uint8_t*)src_address, sizeof(src_address)); 	// set source address
	splitAddress(dst_address, pkt->dst_address);
	SPIWriteBurst(REG_FIFO, (uint8_t*)dst_address, sizeof(dst_address)); 	// set destination address
	SPIWriteBurst(REG_FIFO, pkt->payload, pkt->length);
	writeMode(TX);

	while(1) { // we must wait for TxDone or timeout to occur.
		if (DIO0_int) { //if TxDone
			DIO1_int = 0; // Clear flag
			clearIRQ(); // clear pending interrupts.
			writeMode(STDBY);
			ret = OK;
			break;
		}
		else if (--delay == 0) {
			// timeout occured
			writeMode(STDBY);
			ret = TRANSMIT_TIMEOUT_CODE;
			clearIRQ();
			break;
		}
		else {
			HAL_Delay(1);
		}
	}

	return ret;
}

ERROR_CODES transmit(const Packet* pkt, uint16_t timeout)
{

	Mode mode;
	uint8_t read;
	char src_address[3], dst_address[3];

	//clear TxDone interrupt if it is pending for some reason.
	read = SPIReadSingle(REG_IRQ_FLAGS);
	read |= (1 << TxDone);
	SPIWriteSingle(REG_IRQ_FLAGS, &read);
	mode = getMode();								//Gets current mode in which SX1278 is in.
	writeMode(STDBY);								//Must go in Standby mode in order to set FIFO register.
	read = SPIReadSingle(REG_FIFO_TX_BASE_ADDR);	//Gets the base address of FIFO Transmit.
	SPIWriteSingle(REG_FIFO_ADDR_PTR, &read);		//Writes that address for start of FIFO pointer
	splitAddress(src_address, pkt->src_address);
	SPIWriteBurst(REG_FIFO, (uint8_t*)src_address, sizeof(src_address)); 	// set source address
	splitAddress(dst_address, pkt->dst_address);
	SPIWriteBurst(REG_FIFO, (uint8_t*)dst_address, sizeof(dst_address)); 	// set destination address
	SPIWriteBurst(REG_FIFO, pkt->payload, pkt->length);
	writeMode(TX);									//Go in transmit mode

	while(1) {
		read = SPIReadSingle(REG_IRQ_FLAGS);
		if ((read & (1 << TxDone)) != 0) {
			read = 0;
			read = (1 << TxDone);
			SPIWriteSingle(REG_IRQ_FLAGS, &read); 	//Transmition complete. Clear TxDone interrupt.
			writeMode(mode);						//Go to previous mode.
			sprintf(DmaTxBuffer, "Sending: %s\r\n", pkt->payload);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)DmaTxBuffer, strlen(DmaTxBuffer));
			return OK;
		}
		else if (--timeout == 0) {
			return TRANSMIT_TIMEOUT_CODE;
			clearIRQ();
		}
		HAL_Delay(1);
	}
}

ERROR_CODES cadDetectionAndReceive(Packet *pkt)
{
	uint8_t read;
	uint8_t ret = CAD_NOT_DONE_CODE;

	//read = 0xFF;
	//SPIWriteSingle(REG_IRQ_FLAGS, &read);
	writeMode(CAD);

	while (1) {

		read = SPIReadSingle(REG_IRQ_FLAGS);

		if ((read & CAD_DONE_AND_DETECTED) == CAD_DONE_AND_DETECTED) { // CadDone and CadDetected
				clearIRQ();
				writeMode(STDBY);
				ret = receive(pkt);
				break;
		}
		else {
				clearIRQ();
				writeMode(CAD);	// go back to CAD mode
				break;
			}

		}
	return ret;
}



ERROR_CODES receive(Packet* pkt)
{

	uint8_t number_of_bytes, min, read;

	number_of_bytes = SPIReadSingle(REG_RX_NB_BYTES);
	read = SPIReadSingle(REG_FIFO_RX_BASE_ADDR);	// Gets base address of RX.
	SPIWriteSingle(REG_FIFO_ADDR_PTR, &read);		// Sets FIFO pointer to start of RX buffer.
	read = 0xff;
	SPIWriteSingle(REG_IRQ_FLAGS, &read); 			// clear any interrupts
	writeMode(RXSINGLE);						// Ready to receive packet

	while(1) {

		read = SPIReadSingle(REG_IRQ_FLAGS);

		if ((read & RX_DONE) == RX_DONE) {

			if ((read & PAYLOAD_CRC_ERROR) == PAYLOAD_CRC_ERROR) {
				clearIRQ();
				writeMode(CAD);	//message bad, back to listen mode
				return PAYLOAD_CRC_ERROR_CODE;
			}
			else if ((read & VALID_HEADER) == VALID_HEADER) {
				// Message good, maybe check header also?
				clearIRQ();
				read = SPIReadSingle(REG_FIFO_RX_CURR_ADDR);
				SPIWriteSingle(REG_FIFO_ADDR_PTR, &read);
				number_of_bytes = getPayloadLength(); // explicit mode
				min = (pkt->length >= number_of_bytes) ? number_of_bytes : pkt->length;
				// clear buffer of size length
				for (int i = 0; i < min; i++) {
					pkt->payload[i] = 0;
				}

				SPIReadBurst(REG_FIFO, pkt->payload, min);
				writeMode(STDBY);
				return OK;
			}

		}
		else if ((read & RX_TIMEOUT) == RX_TIMEOUT) {
			clearIRQ();
			// Back to listen mode
			writeMode(CAD);
			return RECEPTION_TIMEOUT_CODE;
		}
	}
}

void setCADDetection(void)
{
	uint8_t data = 0xFF; // masks all interrupts

	writeMode(STDBY); // go in standby to set registers and DIO
	// Mask all other interrupts except CAD
	// First must be CAD done, then CAD detected,
	// In order to start receiving process.
	data &= ~CAD_DONE_MASK & ~CAD_DETECTED_MASK; // Enable interrupts for CAD Done and CAD Detected
	SPIWriteSingle(REG_IRQ_FLAGS_MASK, &data);

	data = 0xAF; // DIO0 = Cad done = 10; DIO1 = Cad Detected = 10
	SPIWriteSingle(REG_DIO_MAPPING_1, &data);
	clearIRQ(); // clear any pending interrupts
	HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	writeMode(CAD);	// go to CAD mode
}

void setRxSingle(void)
{
	uint8_t data = 0xFF;	// mask all interrupts

	writeMode(STDBY);		// go in standby so registers can be written
	// Rx Single mode should be used when time for reception is known.
	// For example after Cad Detection, it is wise to use Rx Single mode
	// for reception.

	data &= ~RX_TIMEOUT_MASK & ~RX_DONE_MASK & ~VALID_HEADER_MASK; // Enable interrupts for RxTimeout and RxDone and valid header
	//data &= ~RX_TIMEOUT_MASK & ~RX_DONE_MASK & ~PAYLOAD_CRC_ERROR_MASK; // Enable interrupts for RxTimeout and RxDone and payloadCRCError.
	SPIWriteSingle(REG_IRQ_FLAGS_MASK, &data);

	data = 0x0D; // for valid header
	//data = 0x0E; // for payload crc error.
	SPIWriteSingle(REG_DIO_MAPPING_1, &data); // Set DIO mapping for RxTimout, RxDone, and CRC error (maybe for Valid header instead).
	clearIRQ(); // clear any pending interrupts
	writeMode(RXSINGLE); // go to RxSingle mode and wait for message.
}

void clearIRQ(void)
{
	uint8_t data = 0xFF;

	SPIWriteSingle(REG_IRQ_FLAGS, &data);
}

void setSyncWord(uint8_t sync)
{
	SPIWriteSingle(REG_SYNC_WORD, &sync);
}

void InitializeLora(SpreadingFactor sf,
					uint32_t freq,
					uint8_t cp,
					Bandwidth bw,
					CodingRate cr,
					uint8_t em,
					uint16_t pl,
					float timeout,
					float pwr)
{
	uint8_t send;

	// must go to sleep first
	writeMode(SLEEP);
	HAL_Delay(100);

	// set LoRa mode
	send = 0x80;
	SPIWriteSingle(REG_OP_MODE, &send);
	writeMode(STDBY);
	//set Spreading Factor
	setSpreadingFactor(sf);

	//set Frequency
	WriteLoraFrequency(freq);

	//set Current protection
	setCurrentProtection(cp);

	//set Bandwidth
	setBandwidth(bw);

	//set Coding Rate
	setCodingRate(cr);

	//set Explicit Mode
	setExplicitMode(em);

	//set Preamble length
	setPreambleLength(pl);

	//set Timeout in [s]
	setTimeout(timeout);

	// set Power
	setPower(pwr);

	// by default always set to maximum available data size
	// because receiver does not know length of packet
	// sender will correct by its actual length in
	// formPacket function.
	setPayloadLength(MAX_PACKET_SIZE);

	clearIRQ();
	writeMode(STDBY);

}
char data_received[255] = {0};
Packet pkt_receive;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ResetLora();
  HAL_Delay(100);

  // do not put big preamble for transmitter, receiver can have it on max val, if preamble of
  // transmitter is not known. Also set bigger timeout for receiver if distance is long.
  // or signal strength is low.
  InitializeLora(SF_7, 433000000, 240, BW_125, CR_4_5, 0, 65000,  10, 20);
  HAL_Delay(100);

  setCRCEnable(1);
  // packet size always 6 bytes for addresses + size of data to be sent.
  formPacket(&pkt_receive, (uint8_t*)data_received, 0, 0);
  setCADDetection();
  //setTransmitForIRQ();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)DmaRxBuffer, MAX_UART_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  clearIRQ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_Delay(500);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RESET_N_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DIO3_Pin DIO0_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin|DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_N_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RESET_N_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ERROR_CODES ret;

	if (GPIO_Pin == DIO0_Pin) {
		if (CadDone0 == getDIO0Mode()) { 	// if the pin was set for CadDone
			clearIRQ();
			setCADDetection();
			ret = CAD_DONE_CODE;
		}
		else if (RxDone0 == getDIO0Mode()) {

			//rx_done++;
			uint8_t read, number_of_bytes, min, data;
			char src_address[3], dst_address[3];

			writeMode(STDBY);
			data = 0xff; // mask all interrupts while message is extracted
			SPIWriteSingle(REG_IRQ_FLAGS_MASK, &data);

			read = SPIReadSingle(REG_FIFO_RX_CURR_ADDR);
			SPIWriteSingle(REG_FIFO_ADDR_PTR, &read);
			number_of_bytes = SPIReadSingle(REG_RX_NB_BYTES); // explicit mode
			min = (pkt_receive.length = number_of_bytes) ? number_of_bytes : pkt_receive.length;
			pkt_receive.length = min;

			SPIReadBurst(REG_FIFO, (uint8_t*)src_address, 3); // first 3 bytes are always source address.
			SPIReadBurst(REG_FIFO, (uint8_t*)dst_address, 3); // 2nd 3 bytes are always destination address
			pkt_receive.src_address = atoi(src_address);
			pkt_receive.dst_address = atoi(dst_address);

			if ( (MY_ADDRESS == pkt_receive.dst_address) || (BROADCAST_ADDRESS == pkt_receive.dst_address) ) {
				// we are addressed, take the message.
				for (int i = 0; i < min; i++) {
					pkt_receive.payload[i] = 0;
				}

				SPIReadBurst(REG_FIFO, (uint8_t*)pkt_receive.payload, min); // read the rest of data
			  	 sprintf(DmaTxBuffer, "%s\r\n", data_received);
				 HAL_UART_Transmit_DMA(&huart2, (uint8_t*)DmaTxBuffer, strlen(DmaTxBuffer));
				clearIRQ();
				setCADDetection(); // wait for new reception
				ret = OK;
			}
			else {
				clearIRQ();
				setCADDetection();
				ret = WRONG_ADDRESS_CODE;
			}
		}
	}

	if (GPIO_Pin == DIO1_Pin) {
		if (CadDetected1 == getDIO1Mode()) {
			//cad_detected++;
			clearIRQ();
			setRxSingle();
			ret = CAD_DETECTED_CODE;
		}
		else if(RxTimeout1 == getDIO1Mode()) {
			clearIRQ();
			setCADDetection();
		}
	}

	if (GPIO_Pin == DIO3_Pin) {
		if (PayloadCrcError3 == getDIO3Mode()) { // there was CRC error.
			clearIRQ();
			setCADDetection();
			ret = PAYLOAD_CRC_ERROR_CODE;
		}
		else if (ValidHeader3 == getDIO3Mode()) {
			//valid header interrupt
			//valid_header++;
		}
	}
	global_status = ret;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	 __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2) {

		if (Size < MAX_UART_SIZE) {
			message_received = 1;
		}
		/*
		oldPos = Size;
		if (Size <= MAX_UART_SIZE) {
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)DmaRxBuffer, Size); // echo

			oldPos = newPos;
			if ( (oldPos + Size) > MAX_UART_SIZE) {
				uint16_t datatocopy = MAX_UART_SIZE - oldPos;
				memcpy(UartRxBuffer + oldPos, DmaRxBuffer, datatocopy);

				oldPos = 0;
				memcpy(UartRxBuffer, DmaRxBuffer + datatocopy, (Size - datatocopy));
				newPos = Size - datatocopy;
			}
			else {
				memcpy(UartRxBuffer + oldPos, DmaRxBuffer, Size);
				newPos = oldPos + Size;
			}
		}*/
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)DmaRxBuffer, MAX_UART_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	}
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
