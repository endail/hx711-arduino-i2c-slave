
#include <Arduino.h>
#include <avr/io.h>
#include "HX711.h"
#include "PinChangeInterrupt.h"

#define HX_CLK_PIN 3 //arduino pin
#define HX_DAT_PIN 4 //arduino pin

int32_t sensorReading = 0;
bool lastReading;
volatile bool shouldUpdateSensor = false;
unsigned long previousMillis = 0;
const long interval = 1000;

HX711::HX711 hx(HX_CLK_PIN, HX_DAT_PIN);

void onPinFalling() {
	disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(HX_DAT_PIN));
	shouldUpdateSensor = true;
}

void setup() {

	while(!Serial);
	Serial.begin(115200);
	hx.begin();

	attachPinChangeInterrupt(
		digitalPinToPinChangeInterrupt(HX_DAT_PIN),
		onPinRising,
		FALLING);

	//PCIFR &= ~bit(digitalPinToPCICRbit(HX_DAT_PIN));
	//*digitalPinToPCMSK(HX_DAT_PIN) |= bit(digitalPinToPCMSKbit(HX_DAT_PIN));
	//*digitalPinToPCICR(HX_DAT_PIN) |= bit(digitalPinToPCICRbit(HX_DAT_PIN));

	//lastReading = digitalRead(HX_DAT_PIN) == HIGH;

}

void loop() {

	if(shouldUpdateSensor) {

		//get a new reading
		sensorReading = hx.getValue();

		//re-enable interrupts afterward
		shouldUpdateSensor = false;
		//*digitalPinToPCMSK(HX_DAT_PIN) |= bit(digitalPinToPCMSKbit(HX_DAT_PIN));
		enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(HX_DAT_PIN));

		//reset the last reading to whatever it is now
		//lastReading = digitalRead(HX_DAT_PIN);

	}

	unsigned long currentMillis = millis();

	if(currentMillis - previousMillis >= interval) {
		previousMillis = currentMillis;
		Serial.print("VAL: ");
		Serial.println(sensorReading);
	}

}

/*
ISR(PCINT2_vect) {

	//first, check if the sensor should be updated
	//if it should be, don't do anything more
	if(shouldUpdateSensor) {
		return;
	}

	//then get the current pin value
	const bool thisReading = digitalRead(HX_DAT_PIN);

	//now check if there is a transition from high to low
	if(lastReading == HIGH && thisReading == LOW) {
		
		//disable the interrupt
		//don't want any more interruptions until the sensor has
		//completed its current conversion
		*digitalPinToPCMSK(HX_DAT_PIN) &= ~bit(digitalPinToPCMSKbit(HX_DAT_PIN));

		//and set a flag to indicate it is time to update
		//the sensor
		shouldUpdateSensor = true;

		return;

	}

	//also update the pin value whenever possible
	lastReading = thisReading;

}
*/
