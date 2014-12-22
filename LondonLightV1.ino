/*
LONDON LIGHT LED GLOBE		David Henshaw, December 2014-
v1. Basic Bluetooth and NeoMatrix code (12/14)
*/

/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution

This version uses the internal data queing so you can treat it like Serial (kinda)!
*********************************************************************/

#include <SPI.h>					
#include "Adafruit_BLE_UART.h"		// Bluetooth Low Enegy
#include <Adafruit_GFX.h>			// Neopixels graphics library
#include <Adafruit_NeoMatrix.h>		// Neopixels library for making shapes
#include <Adafruit_NeoPixel.h>		// Neopixels fundamentals

#ifndef PSTR
#define PSTR							// Make Arduino Due happy
#endif
#define PIN 6							// data pin to NeoPixel LED strip ** check

// MATRIX DECLARATION - See NeoMatrix documentation for details
// https://learn.adafruit.com/adafruit-neopixel-uberguide/neomatrix-library 
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(12, 12, 1, 1, PIN,
	NEO_TILE_TOP + NEO_TILE_LEFT + NEO_TILE_ROWS + NEO_TILE_ZIGZAG +
	NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
	NEO_GRB + NEO_KHZ800);

#define BLACK    0x0000									// Standard color definitions
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

// BLE unit: Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);	// initiate BLE as a serial device

//Variables
String incomingCommand = "";	// String sent from device to Arduino over Bluetooth
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void setup(void)
{
	Serial.begin(115200);
	while (!Serial); // Leonardo/Micro should wait for serial init
	
	Serial.println(F("BLE nRF8001 echo"));
	BTLEserial.setDeviceName("LONGLB1"); /* 7 characters max! */
	BTLEserial.begin();

	matrix.begin();						// get matrix ready			
	matrix.setBrightness(20);			// keep at 20% - help minimize current draw
	randomSeed(analogRead(0));			// so we can have some degree of randomness
}
void loop()
{	
	BTLEserial.pollACI();								// Tell the nRF8001 to do whatever it should be working on. Check this as much as possible
	aci_evt_opcode_t status = BTLEserial.getState();	// Ask what is our current status
	checkBLEStatus(status);								// See if we're connected, disconnected, etc
	if (status == ACI_EVT_CONNECTED) checkForCommand();	// Has a command been sent to us? Results in incomingCommand
	checkSerialConsole();								// Next up, see if we have any data to get from the Serial console ** tweak to send responses to master (phone)
	
	if (incomingCommand.length() > 0) parseIncomingCommand();	// there is a command!
}

/* Functions follow...

*/
void checkBLEStatus(aci_evt_opcode_t status){
	// If the status changed....
	if (status != laststatus) {
		// print it out!
		if (status == ACI_EVT_DEVICE_STARTED) {
			Serial.println(F("* Advertising started"));
		}
		if (status == ACI_EVT_CONNECTED) {
			Serial.println(F("* Connected!"));
		}
		if (status == ACI_EVT_DISCONNECTED) {
			Serial.println(F("* Disconnected or advertising timed out"));
		}
		// OK set the last status change to this one
		laststatus = status;
	}
}
void checkForCommand() {
	incomingCommand = "";		// null
		// Lets see if there's any data for us!
		if (BTLEserial.available()) {
			Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
		}
		// OK while we still have something to read, get a character and print it out
		while (BTLEserial.available()) {
			char c = BTLEserial.read();
			Serial.print(c);

			// Add this character to the command string (format: 1 char command + 2 digit argument. e.g. C01, R19)
			incomingCommand = String(incomingCommand + c);
		}

		// Confirm incomingCommand
		Serial.println(incomingCommand);
}
void checkSerialConsole() {
	if (Serial.available()) {
		// Read a line from Serial
		Serial.setTimeout(100); // 100 millisecond timeout
		String s = Serial.readString();

		// We need to convert the line to bytes, no more than 20 at this time
		uint8_t sendbuffer[20];
		s.getBytes(sendbuffer, 20);
		char sendbuffersize = min(20, s.length());

		Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

		// write the data
		BTLEserial.write(sendbuffer, sendbuffersize);
	}
}
void parseIncomingCommand() {
	char commandParameter = incomingCommand.charAt(0);	// The index of the first character is 0
	switch (commandParameter) {
	case 1:
		// do something
		break;

	}
		

}
