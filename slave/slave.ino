
#include <Arduino.h>
#include "TinyWireS.h"
#include "HX711.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

#define I2C_SLAVE_ADDR 0x13

//These are Arduino pins!
#define I2C_CLK_PIN 2 //not directly used
#define I2C_DAT_PIN 0 //not directly used
#define HX_CLK_PIN 3
#define HX_DAT_PIN 4 //this is the external interrupt pin INT0!

//HX711's 3 bytes
int32_t sensorReading = 0;
volatile bool updating = false;
volatile bool waitingForLow = true;

HX711::HX711 hx(HX_CLK_PIN, HX_DAT_PIN);

inline uint8_t readPin() {
	return bitRead(*(volatile uint8_t*)PINB, digitalPinToBitMask(HX_DAT_PIN));
}

//bytes:
//0 - command
//1 - data
//... - data
//n - data

/*
void receiveEvent(uint8_t count) {

	//no bytes available
	if(!TinyWireS.available() || count > 2) {
		return;
	}

	const uint8_t cmd = TinyWireS.receive();
	const uint8_t dat = count == 2 ? TinyWireS.receive() : 0;

	//set gain
	if(cmd == 1) {
		//dat[0] == 0 = 128
		//dat[0] == 1 = 64
		//dat[1] == 2 = 32
	}

}
*/

void requestEvent() {

	updating = true;

	int32_t val = hx.getValue();

	//attiny is little-endian
	//so send the little bytes first
	TinyWireS.send((val >> (8 * 2)) & 0xff);
	TinyWireS.send((val >> (8 * 1)) & 0xff);
	TinyWireS.send((val >> (8 * 0)) & 0xff);
	
	updating = false;

}

void setup() {

	TinyWireS.begin(I2C_SLAVE_ADDR);
	//TinyWireS.onReceive(receiveEvent);
	TinyWireS.onRequest(requestEvent);

	hx.begin();

	bitSet(GIMSK, ISC01);
	bitClear(GIMSK, ISC00);

}

void loop() {
	TinyWireS_stop_check();
}


ISR(PCINT0_vect) {

	return;

	if(updating) {
		return;
	}

	updating = true;

	const uint8_t currentState = readPin();

	//pin has now gone high
	//waiting for it to go low
	if(!waitingForLow && currentState == 1) {
		waitingForLow = true;
		updating = false;
		return;
	}
	
	//found a falling edge
	//update the sensor
	//re-enable interrupts
	if(waitingForLow && currentState == 0) {
		sensorReading = hx.getValue();
		waitingForLow = false;
		updating = false;
		return;
	}

	updating = false;
	return;

}
