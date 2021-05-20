#include "HX711.h"
#include "PinChangeInterrupt.h"

#define CLOCK_PORT PORTD
#define CLOCK_DDR DDRD
#define CLOCK_PIN PORTD3

#define DATA_PORT PIND
#define DATA_DDR DDRD
#define DATA_PIN PIND4

//PCINT20, see https://github.com/NicoHood/PinChangeInterrupt/#pinchangeinterrupt-table
#define DATA_INT 20

HX711::HX_VALUE sensorReading;
volatile bool shouldUpdateSensor = false;
unsigned long previousMillis = 0;

HX711::HX711 hx(
	&CLOCK_PORT,
	&CLOCK_DDR,
	CLOCK_PIN,
	&DATA_PORT,
	&DATA_DDR,
	DATA_PIN);

void onPinFalling() {
	disablePinChangeInterrupt(DATA_INT);
	shouldUpdateSensor = true;
}

void setup() {

	while(!Serial);
	Serial.begin(115200);

	attachPinChangeInterrupt(
		DATA_INT,
		onPinFalling,
		FALLING);

	hx.begin();

}

void loop() {

	if(shouldUpdateSensor) {
		sensorReading = hx.getValue();
		shouldUpdateSensor = false;
		enablePinChangeInterrupt(DATA_INT);
	}

	unsigned long currentMillis = millis();

	if(currentMillis - previousMillis >= 1000) {
		previousMillis = currentMillis;
		Serial.println(sensorReading);
	}

}
