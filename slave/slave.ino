
/**
 * http://homemadehardware.com/img/attiny85_pinout.jpeg
 * 
 * 
 * 
 */

#include <avr/io.h>
#include "HX711.h"
#include "TinyWireS.h"
#include "PinChangeInterrupt.h"

#define CLOCK_PORT PORTB
#define CLOCK_DDR DDRB
#define CLOCK_PIN PORTB3

#define DATA_PORT PINB
#define DATA_DDR DDRB
#define DATA_PIN PINB4
#define DATA_INT 4

#define I2C_SLAVE_ADDR 0x6E


HX711::HX_VALUE sensorReading;
volatile bool shouldUpdateSensor = false;

HX711::HX711 hx(
	&CLOCK_PORT,
	&CLOCK_DDR,
	CLOCK_PIN,
	&DATA_PORT,
	&DATA_DDR,
	DATA_PIN);


void requestEvent() {

	const HX711::HX_VALUE val = sensorReading;

	//attiny is little-endian
	//so send the little bytes first
	TinyWireS.send((val >> (8 * 2)) & 0xff);
	TinyWireS.send((val >> (8 * 1)) & 0xff);
	TinyWireS.send((val >> (8 * 0)) & 0xff);

}

void onPinFalling() {
	disablePinChangeInterrupt(DATA_INT);
	shouldUpdateSensor = true;
}

void setup() {

	attachPinChangeInterrupt(
		DATA_INT,
		onPinFalling,
		FALLING);

	hx.begin();

	TinyWireS.begin(I2C_SLAVE_ADDR);
	TinyWireS.onRequest(requestEvent);

}

void loop() {

	if(shouldUpdateSensor) {
		sensorReading = hx.getValue();
		shouldUpdateSensor = false;
		enablePinChangeInterrupt(DATA_INT);
	}

	TinyWireS_stop_check();

}
