/*
=============================================================================
Reads and writes NDEF records using an NXP PN532 and ARDUNINO NANO

This software was created to allow an Arduino Nano 328P to communicate with
an ADAFRUIT or SUNFOUNDER PN532 NFC card using a simple series of commands
issued to the Arduino over a USB serial link.

All libraries used are the property of their respective authors and are each 
covered by their own license agreements 

Copyright (c) 2020, SKF UK Ltd

Redistribution and use in source and binary forms, with or without
modification, is not permitted under any circumstances
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REVISION 1.0 April 2020
==============================================================================
*/

#include "main.h"

// ============================================================================

#pragma region PRIVATE MEMBERS
PN532_command command;		   // reader command enum
unsigned long currentTime = 0; // current time (for TIMEOUT management)
bool _commandReceived = false; // command been received over serial port?
bool _enableTimeouts = false;  // enable TIMEOUTS (for WRITE or ONE SHOT)?
bool _blockReader = false;	   // block access to the reader hardware?

// references the UID from the TAG to block multiple reads
uint8_t _headerdata[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// create a new NDEF message
NDEF_Message *ndef_message = new NDEF_Message();
#pragma endregion

// ============================================================================

#pragma region MAIN APPLICATION LOOP
/// <summary>
/// Setup the ARDUINO
/// </summary>
void setup(void)
{
	// set the LED pin
	pinMode(COMMS_LED, OUTPUT);

	// three flashes to confirm the reader is active
	for (int i = 0; i < 3; ++i)
	{
		FlashLED(100, 150);
	}

	// initialise the serial port
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println(F(HARDWARE_IDENTIFIER));

	// reset the command flags
	// hardwareCtrl.commandReceived = false;
	// hardwareCtrl.oneShotRead = false;

	// set TIMER1 for a ONE second event timer
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 3036;
	TCCR1B |= (1 << CS12);
	TIMSK1 |= (1 << TOIE1);

	// initiate connection to the PN532 board
	nfc.begin();

	// configure board to read RFID tags
	nfc.SAMConfig();
}

/// <summary>
/// APPLICATION SUPER LOOP
/// </summary>
void loop(void)
{
	// intercept and process any commands received over the serial port
	ReadSerialCommands();

	// communicate with the PN532
	ConnectToReader();
}
#pragma endregion

// ============================================================================

/// <summary>
/// Process byte level commands from the connected BLE client
/// </summary>
void ReadSerialCommands()
{
	//
	// was a command received over the serial port? We initally start with this
	// set to FALSE. This ensures that any received commands are processed before
	// we attempt to connect to the PN532 reader
	//
	if (!_commandReceived)
	{
		// create a temporary buffer in the heap
		uint8_t *commandBuffer = new uint8_t[COMMAND_BYTES];

		// if not, and we have enabled ONE SHOT READING then latch the processor here
		while ((Serial.available() == 0) & (command == ReadOnce))
		{
			// flash the status LED as a debug reference
			FlashLED(1, 50);
		}

		// flash the status LED as a debug reference
		FlashLED(1, 0);

		// reset the command byte
		command = None;

		// read the contents of the serial port and return the command byte
		while (Serial.available() > 0)
		{
			//
			// the internal command flag basically tells the reader not to wait
			// for any data coming in from the serial port. These commands are
			// typically used to return the reader to return to some state type
			// after an external command has been received and Processed.
			//
			Serial.readBytes(commandBuffer, COMMAND_BYTES);

			// and what (internal or external) command was that?
			command = GetCommandType(commandBuffer);

			// did we receive a command?
			_commandReceived = command != None;

			//if we're writing a message, then append each record
			if (command == AddNdefRecord)
			{
				AddNdefRecordToMessage();
				ResetReader();
			}

			// process the received commands
			ProcessSerialCommand(command, commandBuffer);

			// flash the status LED as a debug reference
			FlashLED(2, 30);
		}

		// release the command buffer from the heap
		delete[] commandBuffer;
	}
}

/// <summary>
/// process the command received over the BLE SoftSerial I/O
/// </summary>
void ProcessSerialCommand(PN532_command cmnd, uint8_t *buffer)
{
	// ok, now process that command
	switch (cmnd)
	{
	case WriteNdefMessage:
		currentTime = millis();
		_enableTimeouts = true;
		Serial.write(PNDM, COMMAND_BYTES);
		break;
	case ReadOnce:
		currentTime = millis();
		_enableTimeouts = true;
		Serial.write(ORTG, COMMAND_BYTES);
		break;
	case ReadContinuous:
		ResetReader();
		Serial.write(CRTG, COMMAND_BYTES);
		break;
	case ClearTag:
		currentTime = millis();
		_enableTimeouts = true;
		Serial.write(CLTG, COMMAND_BYTES);
		break;
	case Acknowledge:
		NFCN[SERIAL_PORT_BYTE] = buffer[SERIAL_PORT_BYTE];
		Serial.write(NFCN, COMMAND_BYTES);
		ResetReader();
		FlashLED(50, 0);
		break;
	default:
		break;
	}
}

/// <summary>
/// Get the received command type
/// </summary>
/// <param name="buffer">byte array to search against</param>
PN532_command GetCommandType(uint8_t *buffer)
{
	int bufferSize = sizeof(buffer);
	if (memcmp(buffer, ADDNDR, 6) == 0)
	{
		return AddNdefRecord;
	}
	else if (memcmp(buffer, PNDM, bufferSize) == 0)
	{
		return WriteNdefMessage;
	}
	else if (memcmp(buffer, ORTG, bufferSize) == 0)
	{
		return ReadOnce;
	}
	else if (memcmp(buffer, CRTG, bufferSize) == 0)
	{
		return ReadContinuous;
	}
	else if (memcmp(buffer, ACKN, bufferSize) == 0)
	{
		return Acknowledge;
	}
	else if (memcmp(buffer, CLTG, bufferSize) == 0)
	{
		return ClearTag;
	}
	else
	{
		return None;
	}
}

/// <summary>
/// Appends a received NDEF record to an existing NDEF message
/// </summary>
void AddNdefRecordToMessage()
{
	// create a new ndef record string buffer
	char *ndefRecord = new char[NTAG_MAX_RECORD_BYTES];

	// clear the serial read buffer contents
	memset(ndefRecord, 0, NTAG_MAX_RECORD_BYTES);
	uint8_t index = 0;

	// load from the serial stream
	while (Serial.available() > 0)
	{
		if (index < NTAG_MAX_RECORD_BYTES)
		{
			ndefRecord[index] = Serial.read();
			++index;
		}
	}

	// remove any line terminators from this string
	for (int i = 0; i < index; ++i)
	{
		if ((ndefRecord[i] == 0x0d) | (ndefRecord[i] == 0x0a))
		{
			ndefRecord[i] = 0x00;
			index--;
		}
	}

	// add an NDEF text record to the NDEF message
	ndef_message->addTextRecord(ndefRecord);

	// post feedback
	Serial.write(ndefRecord, index);

	// terminate with a CR and LF
	Serial.write(0x0d);
	Serial.write(0x0a);

	delete[] ndefRecord;
}

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
void ConnectToReader(void)
{
	// if the reader is blocked, then bypass this method completely
	if (!_blockReader)
	{
		uint8_t pagedata[TOTAL_BLOCKS * BYTES_PER_BLOCK];
		uint8_t headerdata[16];

		// read the card
		uint8_t uidLength = Read_PN532(pagedata, headerdata);

		// what sensor type are we dealing with?
		InsightSenor sensorType = Unknown;

		// if the UID is valid, then the data should be OK
		if (uidLength == UID_LENGTH)
		{
			// make a temporary copy of the received UID
			uint8_t *uidRecord = new uint8_t[UID_LENGTH];
			uidRecord[0] = headerdata[0];
			uidRecord[1] = headerdata[1];
			uidRecord[2] = headerdata[2];
			uidRecord[3] = headerdata[4];
			uidRecord[4] = headerdata[5];
			uidRecord[5] = headerdata[6];
			uidRecord[6] = headerdata[7];

			//
			// if this is the same UID then we don't process this - otherwise we end
			// up in a never ending loop of reading the TAG and sending the data back
			//
			if (memcmp(_headerdata, uidRecord, UID_LENGTH) != 0)
			{
				for (uint8_t i = 0; i < UID_LENGTH; ++i)
				{
					_headerdata[i] = uidRecord[i];
				}

				// reset any command that might have been received
				_commandReceived = false;

				// does this message contain a valid NDEF record?
				if (pagedata[0] == NDEF_EN_RECORD_TNF)
				{
					//create the NDEF payload
					NDEF_Message message = NDEF_Message(&pagedata[2], pagedata[1]);

					//
					// make sure we have at least one NDEF message that we
					// can write out over the USB serial port
					//
					if (message.getRecordCount() > 0)
					{
						// if we have at least one NDEF record then write this to USB
						WriteMessageToSerial(pagedata, headerdata);

						// now (for InsightSenor) we should try and determine the sensor type
						sensorType = GetSensorType(&message);
					}
				}
				else
				{
					Serial.println(INVALID_NDEF);
				}

				//
				// if this is a CMWR2 Insight sensor, then we need to abort
				// right here (the CMWR2 will be corrupted if written to)
				//
				if (((sensorType == CMWR1) | (sensorType == CMWR2)) && ((command == ClearTag) | (command == WriteNdefMessage)))
				{
					// force a timeout reset after ONE SECOND
					currentTime = millis() - (SYSTEM_TIMEOUT - COMMAND_TIMEOUT);

					// notify the user
					Serial.println("COMMAND NOT COMPATIBLE WITH CMWR2");

					// we're done!
					return;
				}

				// clear TAG contents or write complete NDEF message
				ExecuteReaderCommands(headerdata, pagedata);
			}

			// release this object and leave method right here!
			delete[] uidRecord;
			return;
		}
		else if (uidLength == INVALID_UID)
		{
			Serial.println(INVALID_NDEF);

			// clear TAG contents or write complete NDEF message
			ExecuteReaderCommands(headerdata, pagedata);
		}

		// if we've reached this point then we need to reset the received UID
		for (uint8_t i = 0; i < UID_LENGTH; ++i)
		{
			_headerdata[i] = 0x00;
		}
	}
}

/// <summary>
/// Clear TAG contents or write complete NDEF message
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message body</param>
void ExecuteReaderCommands(uint8_t *headerdata, uint8_t *pagedata)
{
	// is this an NTAG card?
	bool isNTAG = ((pagedata[1] == NTAG_213_IC) | (pagedata[1] == NTAG_215_IC) | (pagedata[1] == NTAG_216_IC));

	// process the two supported commands (CLEAR and WRITE NDEF MESSAGE)
	if (command == ClearTag)
	{
		_blockReader = true;
		ClearTheCard(headerdata);
	}
	else if (command == WriteNdefMessage)
	{
		_blockReader = true;
		WriteNdefMessagePayload(headerdata, isNTAG);
		ndef_message->dropAllRecords();
	}
	else if ((command == ReadOnce) | (command == ReadContinuous))
	{
		return;
	}

	// force a timeout reset after ONE SECOND
	currentTime = millis() - (SYSTEM_TIMEOUT - COMMAND_TIMEOUT);
}

/// <summary>
/// Clears and overwrites the complete contents of the NDEF message block
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="clearCard">clear card before write action</param>
void WriteNdefMessagePayload(uint8_t *headerdata, bool clearCard)
{
	// create the page buffer
	uint8_t pageBuffer[BYTES_PER_BLOCK] = {0, 0, 0, 0};

	// default page clear count
	int pages = NTAG_DEFAULT_PAGE_CLEAR;

	//
	// if this is an NTAG card then we need to clear some of the
	// initial pages before we write back to them.
	//
	if (clearCard)
	{
		// clear either the default number (16) or all pages on the card
		for (uint8_t i = 4; i < pages + 4; i++)
		{
			memset(pageBuffer, 0, 4);
			nfc.ntag2xx_WritePage(i, pageBuffer);
			ToggleLED(true);
		}
	}

	// how many bytes are now in this new message
	uint8_t totalBytes = ndef_message->getEncodedSize() + NDEF_EN_RECORD_EXTRA_PAGE_BYTES;

	// allocate memory for a working buffer
	byte buffer[totalBytes];

	// ensure all memory is initialised with the value 0x00
	memset(buffer, 0, totalBytes);

	// the buffer now contains everything from byte #2 onwards
	ndef_message->encode(buffer + 2);

	// insert the two NTAG header bytes (TNF + total bytes)
	buffer[0] = NDEF_EN_RECORD_TNF;
	buffer[1] = (uint8_t)(ndef_message->getEncodedSize());

	// how many pages will it take to write this to the card?
	pages = getPageCount(totalBytes);

	// initialse the page indexes
	uint8_t page = BYTES_PER_BLOCK;
	uint8_t offset = 0;

	// write to each of the required pages
	for (int i = 0; i < pages; i++)
	{
		memcpy(pageBuffer, buffer + offset, BYTES_PER_BLOCK);
		nfc.ntag2xx_WritePage(page, pageBuffer);
		++page;
		offset += BYTES_PER_BLOCK;
		ToggleLED(true);
		Serial.write(PLEASE_WAIT);
	}

	// reset the LED
	ToggleLED(false);
}

/// <summary>
/// Completely wipes an NTAG card of all contents
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
void ClearTheCard(uint8_t *headerdata)
{
	// create the page buffer
	uint8_t pageBuffer[BYTES_PER_BLOCK] = {0, 0, 0, 0};

	// default page clear count
	int pages = headerdata[NTAG_CAPABILITY_CONTAINER] * 2;

	// clear either the default number (16) or all pages on the card
	for (uint8_t i = 4; i < pages + 4; i++)
	{
		memset(pageBuffer, 0, 4);
		nfc.ntag2xx_WritePage(i, pageBuffer);
		ToggleLED(true);
		Serial.write(PLEASE_WAIT);
	}

	// reset the LED
	ToggleLED(false);
}

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
void WriteMessageToSerial(uint8_t *pagedata, uint8_t *headerdata)
{
	// return all collected bytes
	Serial.print(SKF_NTAG_PREFIX);
	Serial.write(headerdata, 16);
	Serial.write(pagedata, pagedata[1] + 3);
	Serial.println(SKF_NTAG_SUFFIX);

	// flush the USB receiver
	Serial.flush();
}

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
uint8_t Read_PN532(uint8_t *pagedata, uint8_t *headerdata)
{
	uint8_t uidLength = 0;
	uint8_t success;

	// buffer for a single blocj
	uint8_t data[BLOCK_SIZE];

	// Buffer to store the returned UID
	uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};

	//
	// Wait for an NTAG2XX card.  When one is found 'uid' will be populated with
	// the UID, and uidLength will indicate the size of the UUID (normally 7 bytes)
	//
	success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, WAIT_FOR_CARD_MS);

	// did we get a valid UID from this card?
	if (success & (uidLength == UID_LENGTH))
	{
		// illuminate the status LED
		ToggleLED(true);

		// read the header block
		if (nfc.mifareclassic_ReadDataBlock(0, data))
		{
			memcpy(headerdata, data, BLOCK_SIZE);
		}

		// reset the block index
		uint8_t block = 4;

		// set the next block
		for (uint8_t i = 0; i < BLOCK_COUNT; i++)
		{
			//
			// try and read the next block. If successful then append
			// the received block to the complete page array
			//
			if (nfc.mifareclassic_ReadDataBlock(block, data))
			{
				// if reader contents are corrupted, then abort here
				if (i == 0 && ((data[1] == 0) | (data[2] == 0) | (data[3] == 0)))
				{
					uidLength = INVALID_UID;
					break;
				}

				// build the return payload
				memcpy(pagedata + (i * BLOCK_SIZE), data, BLOCK_SIZE);
			}

			// increment the block index
			block += BYTES_PER_BLOCK;
		}
	}

	// disable the status LED
	ToggleLED(false);

	// return the number UID bytes
	return uidLength;
}

// ============================================================================

#pragma region PRIVATE SUPPORT METHODS
void ResetReader()
{
	// reset the current command type
	command = None;

	// re-enable reading by the PN532
	_blockReader = false;

	// reenable the command processing
	_commandReceived = false;
}

/// <summary>
/// Determine the Insight sensor type (CMWR2, CMWR3 or UNKNOWN)
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
InsightSenor GetSensorType(NDEF_Message *message)
{
	// go through all NDEF messages looking for named hardware
	for (unsigned int i = 0; i < message->getRecordCount(); ++i)
	{
		NDEF_Record record = message->getRecord(i);
		uint8_t buffer[record.getPayloadLength()];
		record.getPayload(buffer);

		if (needleSearch(buffer, CMWR_31, record.getPayloadLength(), 7))
		{
			return CMWR3;
		}
		else if (needleSearch(buffer, CMWR_23, record.getPayloadLength(), 7))
		{
			return CMWR3;
		}
		else if (needleSearch(buffer, CMWR_21, record.getPayloadLength(), 7))
		{
			return CMWR2;
		}
		else if (needleSearch(buffer, CMWR_22, record.getPayloadLength(), 7))
		{
			return CMWR2;
		}
		else if (needleSearch(buffer, CMWR_IIGO, record.getPayloadLength(), 9))
		{
			return CMWR2;
		}
		else if (needleSearch(buffer, CMWR_1, record.getPayloadLength(), 6))
		{
			return CMWR2;
		}
		else if (needleSearch(buffer, CMWR_2, record.getPayloadLength(), 6))
		{
			return CMWR2;
		}
	}
	return Unknown;
}

/// <summary>
/// How many pages are required to cover all message bytes?
/// </summary>
/// <param name="byteCount">number of message bytes</param>
int getPageCount(int byteCount)
{
	int pages = byteCount / BYTES_PER_BLOCK;
	int pagesModulo = byteCount % BYTES_PER_BLOCK;
	if (pagesModulo > 0)
	{
		++pages;
	}
	return pages;
}

/// <summary>
/// Toggle the LED ON or OFF every time this method is called
/// </summary>
/// <param name="period">true for toggle else false for LED OFF</param>
void ToggleLED(bool enableToggle)
{
	// if we're forcing the LED to be OFF, then do so here
	if (!enableToggle)
	{
		digitalWrite(COMMS_LED, LOW);
	}

	else
	{
		// otherwise just toggle the existing state
		if (digitalRead(COMMS_LED) == HIGH)
		{
			digitalWrite(COMMS_LED, LOW);
		}
		else
		{
			digitalWrite(COMMS_LED, HIGH);
		}
	}
}

/// <summary>
/// Flashes the COMMS LED
/// </summary>
/// <param name="period">milliseconds to illuminate for</param>
void FlashLED(int onPeriod, int offPeriod)
{
	digitalWrite(COMMS_LED, HIGH);
	delay(onPeriod);
	digitalWrite(COMMS_LED, LOW);
	delay(offPeriod);
}

/// <summary>
/// Searches for a needle in a haystack, where both needle and haystack are byte arrays
/// </summary>
/// <param name="haystack">pointer to n bytes of the haystack</param>
/// <param name="needle">pointer to n bytes of the needle</param>
/// <param name="haystack_length">number of elements in the haystack</param>
/// <param name="needle_length">number of elements in the needles</param>
/// <returns>TRUE if needle is found in haystack</returns>
bool needleSearch(uint8_t *haystack, uint8_t *needle, int haystack_length, int needle_length)
{
	int index = 0;
	int tallies = 0;

	// go through all bytes in the haystack (the search reference)
	for (int i = 0; i < haystack_length; i++)
	{
		// reset the number of byte tallies
		tallies = 0;
		index = 0;

		// now go through all bytes in the needle
		for (int k = 0; k < needle_length; k++)
		{
			//
			// first check the search is actually valid. If it's not
			// then just fail and return a FALSE result
			//
			if (k + i > haystack_length)
			{
				return false;
			}

			//
			// if we've already matched at least one byte, then increment
			// the index position of the needle array
			//
			if (index > 0)
			{
				++index;
			}

			//
			// if the current byte in the needle array matches the current
			// byte in the haystack array, then we need to increment the tally
			//
			if (needle[k] == haystack[k + i])
			{
				// we only incement the index counter the once
				if (index == 0)
				{
					++index;
				}

				// increment the tally
				++tallies;
			}

			// if everything matches at this point then return TRUE
			if ((tallies == needle_length) && (tallies == index))
			{
				return true;
			}
		}
	}
	return false;
}
#pragma endregion

// ============================================================================

#pragma region PROCESS INTERUPTS
/// <summary>
///  INTERUPT SERVICE ROUTINE
/// </summary>
/// <param name="TIMER1_OVF_vect">raise on TIMER1 overflow</param>
ISR(TIMER1_OVF_vect)
{
	// initial TIMER1 counter value (rounds to nearest second)
	TCNT1 = 3036;

	if (_enableTimeouts)
	{
		if (millis() > currentTime + SYSTEM_TIMEOUT)
		{
			// we're done with timeouts for the moment
			_enableTimeouts = false;

			// return feedback
			FlashLED(5, 0);
			Serial.write(READER_TIMEOUT, 5);

			// reset the current time
			currentTime = millis();

			// reset the reader
			ResetReader();
		}
	}
	else
	{
		// reset the current time to prevent a TIMEOUT being raised
		currentTime = millis();
	}
}
#pragma endregion
