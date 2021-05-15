
#include "HX711.h"
#include "PinChangeInterrupt.h"

#define HX_CLK_PIN 3 //arduino pin
#define HX_DAT_PIN 4 //arduino pin

int32_t sensorReading = 0;
volatile bool shouldUpdateSensor = false;
unsigned long previousMillis = 0;
const long interval = 1000;

HX711::HX711 hx(HX_CLK_PIN, HX_DAT_PIN);

void onPinFalling() {
	disablePinChangeInterrupt(
		digitalPinToPinChangeInterrupt(HX_DAT_PIN));
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

}

void loop() {

	if(shouldUpdateSensor) {
		sensorReading = hx.getValue();
		shouldUpdateSensor = false;
		enablePinChangeInterrupt(
			digitalPinToPinChangeInterrupt(HX_DAT_PIN));
	}

	unsigned long currentMillis = millis();

	if(currentMillis - previousMillis >= interval) {
		previousMillis = currentMillis;
		Serial.println(sensorReading);
	}

}
