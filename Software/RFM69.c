/**
  ******************************************************************************
  * @file           : RFM69.c
  * @brief          : partial port of: https://github.com/LowPowerLab/RFM69
  ******************************************************************************
  * @attention
  * all rights belong to LowPowerLab
  * Author: MTFinding
  ******************************************************************************
*/

#include <main.h>
#include <RFM69.h>
#include <stdbool.h>
#include <string.h>

// GLOBALS
_Bool _isRFM69HW = true;
_Bool _spyMode;
static volatile _Bool _haveData;
static uint8_t DATA[RF69_MAX_DATA_LEN+1]; // RX/TX payload buffer, including end of string NULL char
static uint8_t DATALEN;
static uint16_t SENDERID;
static uint16_t TARGETID; // should match _address
static uint8_t PAYLOADLEN;
static uint8_t ACK_REQUESTED;
static uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
static int16_t RSSI; // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
static uint8_t _mode; // should be protected?
uint8_t _powerLevel = 20;
uint16_t _address;

// FUNCTIONS
uint8_t RFM69_readReg(uint8_t addr)
{
	uint8_t sendval = addr & 0x7F;
	uint8_t regval;

	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &regval, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);
	return regval;
}

void RFM69_writeReg(uint8_t addr, uint8_t value)
{
	uint8_t sendval[2] = {addr | 0x80, value};

	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, sendval, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);

}

int16_t RFM69_readRSSI(_Bool forceTrigger)
{

	int16_t rssi = 0;

	if (forceTrigger)
	{
		// RSSI trigger not needed if DAGC is in continuous mode
		RFM69_writeReg(REG_RSSICONFIG, RF_RSSI_START);
		while ((RFM69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
	}

	rssi = -RFM69_readReg(REG_RSSIVALUE);
	rssi >>= 1;
	return rssi;
}

uint32_t millis()
{
	return HAL_GetTick();
}

void RFM69_setHighPowerRegs(_Bool enable)
{
	if (!_isRFM69HW || _powerLevel<20) enable=false;
	RFM69_writeReg(REG_TESTPA1, enable ? 0x5D : 0x55);
	RFM69_writeReg(REG_TESTPA2, enable ? 0x7C : 0x70);
}

void RFM69_setMode(uint8_t newMode)
{
	if (newMode == _mode)
		return;

	switch (newMode) {
	case RF69_MODE_TX:
		RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
		if (_isRFM69HW) RFM69_setHighPowerRegs(true);
		break;
	case RF69_MODE_RX:
		RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
		if (_isRFM69HW) RFM69_setHighPowerRegs(false);
		break;
	case RF69_MODE_SYNTH:
		RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
		break;
	case RF69_MODE_STANDBY:
		RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
		break;
	case RF69_MODE_SLEEP:
		RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
		break;
	default:
		return;
	}

	// we are using packet mode, so this check is not really needed
	// but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
	while (_mode == RF69_MODE_SLEEP && (RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

	_mode = newMode;
}

_Bool RFM69_canSend()
{
	if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && RFM69_readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
//	if (_mode == RF69_MODE_RX) // if signal stronger than -100dBm is detected assume channel activity

	{
		RFM69_setMode(RF69_MODE_STANDBY);
		return true;
	}
	return false;
}

void RFM69_receiveBegin()
{
	DATALEN = 0;
	SENDERID = 0;
	TARGETID = 0;
	PAYLOADLEN = 0;
	ACK_REQUESTED = 0;
	ACK_RECEIVED = 0;
#if defined(RF69_LISTENMODE_ENABLE)
	RF69_LISTEN_BURST_REMAINING_MS = 0;
#endif
	RSSI = 0;
	if (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
		RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
	RFM69_setMode(RF69_MODE_RX);
}

// internal function - interrupt gets called when a packet is received
void RFM69_interruptHandler()
{
	if (_mode == RF69_MODE_RX && (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
	{
		RFM69_setMode(RF69_MODE_STANDBY);

		HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RESET);
		uint8_t sendval = REG_FIFO & 0x7F;
		HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);

		uint8_t recval;
		HAL_SPI_Receive(&hspi1, &recval, 1, HAL_MAX_DELAY);
		PAYLOADLEN = recval;

		PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
		HAL_SPI_Receive(&hspi1, &recval, 1, HAL_MAX_DELAY);
		TARGETID = recval;

		HAL_SPI_Receive(&hspi1, &recval, 1, HAL_MAX_DELAY);
		SENDERID = recval;

		HAL_SPI_Receive(&hspi1, &recval, 1, HAL_MAX_DELAY);
		uint8_t CTLbyte = recval;

		TARGETID |= ((uint16_t)CTLbyte & 0x0C) << 6; //10 bit address (most significant 2 bits stored in bits(2,3) of CTL byte
		SENDERID |= ((uint16_t)CTLbyte & 0x03) << 8; //10 bit address (most sifnigicant 2 bits stored in bits(0,1) of CTL byte

		if(!(_spyMode || TARGETID == _address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in spy mode
				|| PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
		{
			PAYLOADLEN = 0;
			HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);
			RFM69_receiveBegin();
			return;
		}

		DATALEN = PAYLOADLEN - 3;
		ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
		ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag
		uint8_t _pl = _powerLevel; //interruptHook() can change _powerLevel so remember it
		//		interruptHook(CTLbyte);    // TWS: hook to derived class interrupt function 			-- NOT IMPLEMENTED YET

		for (uint8_t i = 0; i < DATALEN; i++)
		{
			HAL_SPI_Receive(&hspi1, &recval, 1, HAL_MAX_DELAY);
			DATA[i] = recval;
		}

		DATA[DATALEN] = 0; // add null at end of string // add null at end of string
		HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);
		RFM69_setMode(RF69_MODE_RX);
		if (_pl != _powerLevel) RFM69_setPowerLevel(_powerLevel); //set new _powerLevel if changed
	}
	RSSI = RFM69_readRSSI(false);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
_Bool RFM69_receiveDone()
{
	if (_haveData) {
		_haveData = false;
		RFM69_interruptHandler();
	}
	if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
	{
		RFM69_setMode(RF69_MODE_STANDBY); // enables interrupts
		return true;
	}
	else if (_mode == RF69_MODE_RX) // already in RX no payload yet
	{
		return false;
	}
	RFM69_receiveBegin();
	return false;
}

void RFM69_sendFrame(uint16_t toAddress, const void* buffer, uint8_t bufferSize, _Bool requestACK, _Bool sendACK)
{
	//NOTE: overridden in RFM69_ATC!
	RFM69_setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
	while ((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
	//writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
	if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

	// control byte
	uint8_t CTLbyte = 0x00;
	if (sendACK)
		CTLbyte = RFM69_CTL_SENDACK;
	else if (requestACK)
		CTLbyte = RFM69_CTL_REQACK;

	if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; //assign last 2 bits of address if > 255
	if (_address > 0xFF) CTLbyte |= (_address & 0x300) >> 8;   //assign last 2 bits of address if > 255

	// write to FIFO
	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RESET);

	uint8_t sendval = REG_FIFO | 0x80;
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
	sendval = bufferSize + 3;
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
	sendval = (uint8_t)toAddress;
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
	sendval = (uint8_t)_address;
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
	sendval = CTLbyte;
	HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);


	for (uint8_t i = 0; i < bufferSize; i++)
	{
		sendval = ((uint8_t*) buffer)[i];
		HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);

	}

	HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);

	// no need to wait for transmit mode to be ready since its handled by the radio
	RFM69_setMode(RF69_MODE_TX);
//	uint32_t txStart = millis();
//	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
	while ((RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for PacketSent
	RFM69_setMode(RF69_MODE_STANDBY);
}

void RFM69_send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, _Bool requestACK)
{
	RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	uint32_t now = millis();
	while (!RFM69_canSend() && millis() - now < RF69_CSMA_LIMIT_MS){
		RFM69_receiveDone();
	}
	RFM69_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// should be polled immediately after sending a packet with ACK request
_Bool RFM69_ACKReceived(uint16_t fromNodeID)
{
	if (RFM69_receiveDone())
		return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
	return false;
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
_Bool RFM69_ACKRequested()
{
	return ACK_REQUESTED && (TARGETID == _address);
}

// should be called immediately after reception in case sender wants ACK
void RFM69_sendACK(const void* buffer, uint8_t bufferSize)
{
	ACK_REQUESTED = 0;   // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
	uint16_t sender = SENDERID;
	int16_t _RSSI = RSSI; // save payload received RSSI value
	RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	uint32_t now = millis();
	while (!RFM69_canSend() && millis() - now < RF69_CSMA_LIMIT_MS){
		RFM69_receiveDone();
	}
	SENDERID = sender;    // TWS: Restore SenderID after it gets wiped out by receiveDone()
	RFM69_sendFrame(sender, buffer, bufferSize, false, true);
	RSSI = _RSSI; // restore payload RSSI
}

// to increase the chance of getting a packet across, call this function instead of send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5..8ms at 50kbps@915MHz
_Bool RFM69_sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
	uint32_t sentTime;
	for (uint8_t i = 0; i <= retries; i++)
	{
		RFM69_send(toAddress, buffer, bufferSize, true);
		sentTime = millis();
		while (millis() - sentTime < retryWaitTime)
		{
			if (RFM69_ACKReceived(toAddress)) return true;
		}
	}
	return false;
}

void RFM69_setPowerLevel(uint8_t powerLevel)
{
	uint8_t PA_SETTING;
	if (_isRFM69HW) {
		if (powerLevel>23) powerLevel = 23;
		_powerLevel =  powerLevel;

		//now set Pout value & active PAs based on _powerLevel range as outlined in summary above
		if (_powerLevel < 16) {
			powerLevel += 16;
			PA_SETTING = RF_PALEVEL_PA1_ON; // enable PA1 only
		} else {
			if (_powerLevel < 20)
				powerLevel += 10;
			else
				powerLevel += 8;
			PA_SETTING = RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON; // enable PA1+PA2
		}
		RFM69_setHighPowerRegs(true); //always call this in case we're crossing power boundaries in TX mode
	} else { //this is a W/CW, register value is the same as _powerLevel
		if (powerLevel>31) powerLevel = 31;
		_powerLevel =  powerLevel;
		PA_SETTING = RF_PALEVEL_PA0_ON; // enable PA0 only
	}

	//write value to REG_PALEVEL
	RFM69_writeReg(REG_PALEVEL, PA_SETTING | powerLevel);
}

void RFM69_setHighPower(_Bool _isRFM69HW_HCW)
{
	_isRFM69HW = _isRFM69HW_HCW;
	RFM69_writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON); //disable OverCurrentProtection for HW/HCW
	RFM69_setPowerLevel(_powerLevel);
}

void RFM69_encrypt(const char* key)
{
#if defined(RF69_LISTENMODE_ENABLE)
	_haveEncryptKey = key;
#endif

	RFM69_setMode(RF69_MODE_STANDBY);
	uint8_t validKey = key != 0 && strlen(key)!=0;

	if (validKey)
	{

#if defined(RF69_LISTENMODE_ENABLE)
		memcpy(_encryptKey, key, 16);
#endif

		HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RESET);
		uint8_t sendval = REG_AESKEY1 | 0x80;
		HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);

		for (uint8_t i = 0; i < 16; i++)
		{
			uint8_t sendval = key[i];
			HAL_SPI_Transmit(&hspi1, &sendval, 1, HAL_MAX_DELAY);
		}
		HAL_GPIO_WritePin(RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, SET);
	}
	RFM69_writeReg(REG_PACKETCONFIG2, (RFM69_readReg(REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

_Bool RFM69_initialize(uint8_t freqBand, uint16_t nodeID, uint8_t networkID)
{

	const uint8_t CONFIG[][2] =
	{
			/* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
			/* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
			/* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
			/* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
			/* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
			/* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

			/* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
			/* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
			/* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

			// looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
			// +17dBm and +20dBm are possible on RFM69HW
			// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
			// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
			// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
			///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
			///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

			// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
			/* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
			//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
			/* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
			/* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
			/* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
			/* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
			///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
			/* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
			/* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
			/* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
			//* 0x31 */ { REG_SYNCVALUE3, 0xAA },
			//* 0x31 */ { REG_SYNCVALUE4, 0xBB },
			/* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
			/* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
			///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
			/* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
			/* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_OFF | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
			//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
			/* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
			{255, 0}
	};

	uint32_t start = millis();
	uint8_t timeout = 50;

	do RFM69_writeReg(REG_SYNCVALUE1, 0xAA); while (RFM69_readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
	if (millis()-start >= timeout) return false;
	start = millis();
	do RFM69_writeReg(REG_SYNCVALUE1, 0x55); while (RFM69_readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);
	if (millis()-start >= timeout) return false;

	for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
		RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);

	RFM69_encrypt(0);

	RFM69_setHighPower(_isRFM69HW);
	RFM69_setMode(RF69_MODE_STANDBY);

	// wait for ModeReady
	start = millis();
	while (((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout);
	if (millis()-start >= timeout) return false;

	_address = nodeID;
//	PAYLOADLEN = 0;

	HAL_Delay(10); // wait 10ms to give RFM69 some time to relax

	return true;
}

// return the frequency (in Hz)
uint32_t RFM69_getFrequency()
{
	return RF69_FSTEP * (((uint32_t) RFM69_readReg(REG_FRFMSB) << 16) + ((uint16_t) RFM69_readReg(REG_FRFMID) << 8) + RFM69_readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void RFM69_setFrequency(uint32_t freqHz)
{
	uint8_t oldMode = _mode;
	if (oldMode == RF69_MODE_TX) {
		RFM69_setMode(RF69_MODE_RX);
	}
	freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
	RFM69_writeReg(REG_FRFMSB, freqHz >> 16);
	RFM69_writeReg(REG_FRFMID, freqHz >> 8);
	RFM69_writeReg(REG_FRFLSB, freqHz);
	if (oldMode == RF69_MODE_RX) {
		RFM69_setMode(RF69_MODE_SYNTH);
	}
	RFM69_setMode(oldMode);
}

uint8_t* RFM69_returnData()
{
	return DATA;
}

void RFM69_havedata(_Bool havedata)
{
	_haveData = havedata;
}

void RFM69_isSpy(_Bool amiaspy)
{
	_spyMode = amiaspy;
}

void RFM69_reset()
{
	HAL_GPIO_WritePin(RFM69_RST_GPIO_Port, RFM69_RST_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RFM69_RST_GPIO_Port, RFM69_RST_Pin, RESET);
	HAL_Delay(10);
}

_Bool RFM69_isMode(uint8_t mode){
	if(_mode == mode) return true;
	return false;
}

uint8_t RFM69_DataLen(){
	return DATALEN;
}
