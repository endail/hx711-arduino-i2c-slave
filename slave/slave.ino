// MIT License
//
// Copyright (c) 2021 Daniel Robertson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * Uses the following libs:
 * https://github.com/rambo/TinyWire
 * https://github.com/NicoHood/PinChangeInterrupt
 * 
 * Derivitive of my own HX711 code
 * https://github.com/endail/hx711
 * 
 * 
 * Helpful links
 * http://homemadehardware.com/img/attiny85_pinout.jpeg
 * 
 * 
 * Function of this sketch:
 * 
 * 1. HX711's DOUT pin falls LOW when data is ready to be
 * retrieved. The pinchangeinterrupt lib watches for this
 * on the data pin and updates the sensorReading var
 * accordingly.
 * 
 * 2. To respond to I2C requests for most recent HX711
 * reading.
 */

#include <avr/atomic.h>
#include <avr/io.h>
#include "HX711.h"
#include "PinChangeInterrupt.h"
#include "TinyWireS.h"

#define CLOCK_PORT PORTB
#define CLOCK_DDR DDRB
#define CLOCK_PIN PORTB3

#define DATA_PORT PINB
#define DATA_DDR DDRB
#define DATA_PIN PINB4
#define DATA_INT 4

#define I2C_SLAVE_ADDR 0x6E
#define DEFAULT_CMD 0b01000001;
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE(16)
#endif

void onI2CRequest();
void onI2CReceive(uint8_t howMany);
void onPinFalling();
void transmitReading();
void processCommand(const uint8_t cmd);
void setup();
void loop();

enum class I2C_TX_TYPE {
    ONCE = 0,
    STREAM
};

//settings
HX711::Channel ch = HX711::Channel::A;
I2C_TX_TYPE transmitType = I2C_TX_TYPE::STREAM;

//i2c stuff
HX711::HX_VALUE sensorReading = 0;
volatile bool shouldSendReading = false;
volatile bool shouldUpdateSensor = false;

HX711::HX711 hx(
    &CLOCK_PORT,
    CLOCK_PIN,
    &DATA_PORT,
    DATA_PIN);


void setup() {

    //setup in/out pins
    CLOCK_DDR |= (1 << CLOCK_PIN);
    DATA_DDR &= ~(1 << DATA_PIN);

    attachPinChangeInterrupt(
        DATA_INT,
        onPinFalling,
        FALLING);

    hx.begin();

    processCommand(DEFAULT_CMD);

    TinyWireS.onReceive(onI2CReceive);
    TinyWireS.onRequest(onI2CRequest);
    TinyWireS.begin(I2C_SLAVE_ADDR);

}

void loop() {

    if(shouldUpdateSensor) {

        sensorReading = hx.getValue(ch);
        shouldUpdateSensor = false;
        enablePinChangeInterrupt(DATA_INT);

        if(transmitType == I2C_TX_TYPE::STREAM) {
            transmitReading();
        }

    }

    if(shouldSendReading) {
        transmitReading();
        shouldSendReading = false;
    }

    TinyWireS_stop_check();

}

void onI2CRequest() {
    transmitReading();
}

void onI2CReceive(uint8_t howMany) {

    if(!TinyWireS.available()) {
        return;
    }

    if(howMany > TWI_RX_BUFFER_SIZE) {
        return;
    }

    //only 1 byte is needed
    if(howMany != 1) {
        return;
    }

    const uint8_t cmd = TinyWireS.receive();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        processCommand(cmd);
    }

}

void onPinFalling() {
    disablePinChangeInterrupt(DATA_INT);
    shouldUpdateSensor = true;
}

void transmitReading() {

    //make a copy
    const HX711::HX_VALUE val = sensorReading;

    TinyWireS.send((val >> 16) & 0xff);
    TinyWireS.send((val >> 8)  & 0xff);
    TinyWireS.send( val        & 0xff);

}

void processCommand(const uint8_t cmd) {

    const uint8_t txType =      (cmd >> 0) & 0b0000001;
    const uint8_t gain =        (cmd >> 1) & 0b0000011;
    const uint8_t channel =     (cmd >> 3) & 0b0000001;
    const uint8_t bitF =        (cmd >> 4) & 0b0000001;
    const uint8_t byteF =       (cmd >> 5) & 0b0000001;
    const uint8_t pwr =         (cmd >> 6) & 0b0000001;
    const uint8_t oneShot =     (cmd >> 7) & 0b0000001;

    shouldSendReading = static_cast<bool>(oneShot);

    //if a one-shot reading is requested,
    //no longer process command
    if(shouldSendReading) {
        return;
    }

    transmitType = static_cast<I2C_TX_TYPE>(txType);
    hx.setGain(static_cast<HX711::Gain>(gain));
    ch = static_cast<HX711::Channel>(channel);
    hx.setBitFormat(static_cast<HX711::Format>(bitF));
    hx.setByteFormat(static_cast<HX711::Format>(byteF));
    
    if(pwr) {
        hx.powerUp();
    }
    else {
        hx.powerDown();
    }


}

