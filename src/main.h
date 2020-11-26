#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <SoftwareSerial.h>
#include "NDEF/NDEF_Message.h"

#pragma once
#include <stdint.h>

// ============================================================================

#define UID_LENGTH 7			// byte size of NTAG UID
#define BLOCK_SIZE 16			// block size in bytes
#define BLOCK_COUNT 16			// number of blocks to read from the card
#define TOTAL_BLOCKS 86			// total number of NDEF blocks
#define BYTES_PER_BLOCK 4		// number of bytes per block
#define SERIAL_BAUD_RATE 115200 // serial port baud rate
#define COMMS_LED 7				// Configurable
#define WAIT_FOR_CARD_MS 100	// how long to wait for a card before continuing
#define COMMAND_BYTES 6			// how many NDEF records should we ordinarily expect?
#define SYSTEM_TIMEOUT 30000	// reset system to default after this mS
#define COMMAND_TIMEOUT 1000	// reset system to default after this mS
#define NEXT_SCAN_DELAY 1000	// how long to wait before the next scan
#define CMWR2_RECORDS 9			// number of NDEF records in a CMWR 2 sensor
#define CMWR3_RECORDS 12		// number of NDEF records in a CMWR 3 sensor
#define SERIAL_PORT_BYTE 3		// which byte position in the NFC response references the COM port
#define PLEASE_WAIT 0x2e		// full stop character

// ============================================================================

#define NTAG_IC_TYPE 12				 // NTAG byte which describes the actual card type
#define NTAG_CAPABILITY_CONTAINER 14 // NTAG byte which details the total number of user bytes available
#define NTAG_DEFAULT_PAGE_CLEAR 16	 // how many pages should be cleared by default before a write action
#define NTAG_MAX_RECORD_BYTES 24	 //	maximum number of characters per NDEF record

// ============================================================================

#define HARDWARE_IDENTIFIER "SKF_INSIGHT_RAIL" // hardware identifier flag
#define SKF_NTAG_PREFIX "<<-"				   // starting brace for valid NDEF payload
#define SKF_NTAG_SUFFIX "->>"				   // ending brace for valid NDEF payload
#define INVALID_NDEF "INVALID NDEF RECORD"	   // no valid NDEF records could be found

// ============================================================================

#define NTAG_213_IC 0x12 // byte code (payload[1]) that identifies and NTAG-213 card
#define NTAG_215_IC 0x3e // byte code (payload[1]) that identifies and NTAG-215 card
#define NTAG_216_IC 0x6d // byte code (payload[1]) that identifies and NTAG-216 card

// ============================================================================

#define PN532_SCK (13)	// SPI pin SCLK
#define PN532_MOSI (10) // SPI pin MOSI
#define PN532_MISO (12) // SPI pin MISO
#define PN532_SS (11)	// SPI pin SS

// ============================================================================

// configure and initialise the NFC reader
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// ============================================================================

uint8_t READER_TIMEOUT[5] = {0x2a, 0x54, 0x2f, 0x0d, 0x0a};

// ============================================================================

uint8_t AKN[3] = {0x00, 0x0d, 0x0a};

// ============================================================================

uint8_t ACKN[6] = {0x41, 0x43, 0x4b, 0x00, 0x0d, 0x0a};   // simple acknowledge
uint8_t CRTG[6] = {0x43, 0x52, 0x54, 0x47, 0x0d, 0x0a};   // continuous read
uint8_t ORTG[6] = {0x4f, 0x52, 0x54, 0x47, 0x0d, 0x0a};   // read once on each receipt of this command
uint8_t PNDM[6] = {0x50, 0x4e, 0x44, 0x4d, 0x0d, 0x0a};   // post NDEF message to the the PN532
uint8_t CLTG[6] = {0x43, 0x4c, 0x54, 0x47, 0x0d, 0x0a};   // clear all TAG contents
uint8_t ADDNDR[6] = {0x41, 0x44, 0x44, 0x4e, 0x44, 0x52}; // add an NDEF record to the NDEF message

// ============================================================================

uint8_t NFCN[6] = {0x4e, 0x46, 0x43, 0x00, 0x0d, 0x0a}; // NFC response payload for an ACKN (with serial port no.)

// ============================================================================

uint8_t NDEF_EN_RECORD_EXTRA_PAGE_BYTES = 0x05;
uint8_t NDEF_EN_RECORD_TNF = 0x03;
uint8_t INVALID_UID = 0xff;

// ============================================================================

uint8_t CMWR_1[6] = {0x43, 0x4d, 0x57, 0x52, 0x20, 0x31};								   // CMWR 1 sensor hardware type (not supported)
uint8_t CMWR_2[6] = {0x43, 0x4d, 0x57, 0x52, 0x20, 0x32};								   // CMWR 2 sensor hardware type
uint8_t CMWR_3[6] = {0x43, 0x4d, 0x57, 0x52, 0x20, 0x33};								   // CMWR 3 sensor hardware type
uint8_t FIELD_HARDWARE_CMWR2[5] = {0x68, 0x77, 0x76, 0x6e, 0x3a};						   // "hwnv" label for CMWR3 sensors
uint8_t FIELD_HARDWARE_CMWR3[9] = {0x68, 0x61, 0x72, 0x64, 0x77, 0x61, 0x72, 0x65, 0x3a}; // hardware label for CMWR3 sensors

// ============================================================================

/// <summary>
/// Describes each of the commands that this reader supports
/// </summary>
enum PN532_command : uint8_t
{
	None,
	Acknowledge,
	ClearTag,
	ReadOnce,
	ReadContinuous,
	AddNdefRecord,
	WriteNdefMessage,
};

/// <summary>
/// Describes each of the supported Insight Rail sensor types
/// </summary>
enum InsightSenor
{
	Unknown,
	CMWR1,
	CMWR2,
	CMWR3
};

/// <summary>
/// Describes each of the supported card types
/// </summary>
enum IC
{
	NDEF_213,
	NDEF_215,
	NDEF_216
};

// ============================================================================

#pragma region METHOD PROTOTYPES
/// <summary>
/// Process byte level commands from the connected BLE client
/// </summary>
void ReadSerialCommands();

/// <summary>
/// process the command received over the BLE SoftSerial I/O
/// </summary>
void ProcessSerialCommand(PN532_command, uint8_t *);

/// <summary>
/// Get the received command type
/// </summary>
/// <param name="buffer">byte array to search against</param>
PN532_command GetCommandType(uint8_t *);

/// <summary>
/// Process AT commands received over BLUETOOTH
/// </summary>
/// <param name="commandBuffer">pointer to the AT command buffer</param>
/// <param name="bytes">number of AT bytes to read</param>
void ProcessATCommand(uint8_t *, size_t);

/// <summary>
/// Appends a received NDEF record to an existing NDEF message
/// </summary>
void AddNdefRecordToMessage(void);

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
void ConnectToReader(void);

/// <summary>
/// Clear TAG contents or write complete NDEF message
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message body</param>
void ExecuteReaderCommands(uint8_t *, uint8_t *);

/// <summary>
/// Clears and overwrites the complete contents of the NDEF message block
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="clearCard">clear card before write action</param>
void WriteNdefMessagePayload(uint8_t *, bool);

/// <summary>
/// Completely wipes an NTAG card of all contents
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
void ClearTheCard(uint8_t *);

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
void WriteMessageToSerial(uint8_t *, uint8_t *);

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
uint8_t Read_PN532(uint8_t *, uint8_t *);

/// <summary>
/// Reset the reader after RTOS timeout
/// </summary>
void ResetReader(void);

/// <summary>
/// Determine the Insight sensor type (CMWR2, CMWR3 or UNKNOWN)
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
InsightSenor GetSensorType(NDEF_Message *);

/// <summary>
/// How many pages are required to cover all message bytes?
/// </summary>
/// <param name="byteCount">number of message bytes</param>
int getPageCount(int);

/// <summary>
/// Toggle the LED ON or OFF every time this method is called
/// </summary>
/// <param name="period">true for toggle else false for LED OFF</param>
void ToggleLED(bool);

/// <summary>
/// Flashes the COMMS LED
/// </summary>
/// <param name="period">milliseconds to illuminate for</param>
void FlashLED(int, int);

/// <summary>
/// Searches for a needle in a haystack, where both needle and haystack are byte arrays
/// </summary>
/// <param name="haystack">pointer to n bytes of the haystack</param>
/// <param name="needle">pointer to n bytes of the needle</param>
/// <param name="haystack_length">number of elements in the haystack</param>
/// <param name="needle_length">number of elements in the needles</param>
/// <returns>TRUE if needle is found in haystack</returns>
bool needleSearch(uint8_t *, uint8_t *, int, int);

/// <summary>
/// INTERUPT SERVICE ROUTINE
/// </summary>
ISR(TIMER1_OVF_vect);
#pragma endregion

// ============================================================================

