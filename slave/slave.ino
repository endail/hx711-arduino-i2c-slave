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
 * Sketch uses the following libraries. Install these first:
 * https://github.com/rambo/TinyWire
 * https://github.com/NicoHood/PinChangeInterrupt
 * 
 * A derivative of my own HX711 library, which is already
 * included alongside this sketch.
 * 
 * 
 * Function of this sketch:
 * 
 * 1. When the HX711's DOUT/DATA pin goes low, it indicates
 * data is ready to be retrieved from the HX711. This sketch
 * defines an interrupt for when this occurs. When interrupted,
 * a flag in the main loop will cause the data to be retrieved
 * and a copy of the HX711's reading will be stored.
 * 
 * 2. This sketch also operates an I2C master. It will respond
 * to requests by sending the number of milliseconds between
 * sending the response and when data was retrieved from the
 * HX711 to a maximum of UINT16_MAX. This is approximately
 * 65 seconds. An I2C slave can check this value to see how
 * old the HX711's reading is.
 * 
 * The I2C response is comprised of 5 bytes. The first two
 * bytes contain the timestamp described above as an unsigned
 * 16 bit integer. The last three bytes contain the HX711
 * reading value as a signed 24 bit integer.
 * 
 * 3. Various settings can be set with an I2C write operation
 * transmitted as one byte. The single byte contains various
 * bits to indicate settings. They are described below, with
 * bit numbers being 0-based (ie. the most significant bit is
 * bit 7, and least significant bit is bit 0).
 * 
 * Bits 6-7:
 *      not used
 * 
 * Bit 5: HX711 power setting
 *      0: power off the sensor
 *      1: power on the sensor (default)
 * 
 * Bit 4: HX711 byte format
 *      0: indicates HX711 is outputting bytes in MSB format (default)
 *      1: indicates HX711 is outputting bytes in LSB format
 * 
 * Bit 3: HX711 bit format
 *      0: indicates HX711 is outputting bits in MSB format (default)
 *      1: indicates HX711 is outputting bits in LSB format
 * 
 * Bit 2: HX711 channel
 *      0: read from channel A (default)
 *      1: read from channel B
 * 
 * Bits 0-1: HX711 gain
 *      0: set gain to 128 (default)
 *      1: set gain to 32
 *      2: set gain to 64
 * 
 * 
 * Notes:
 * 
 * 1. This sketch hardcodes the following important values:
 *      I2C slave address: 0x6e
 *      I2C clock pin connected to PB2 (Arduino pin 2)
 *      I2C data pin connected to PB0 (Arduino pin 0)
 *      HX711's clock pin connected to PB3 (Arduino pin 3)
 *      HX711's data pin connected to PB4 (Arduino pin 4)
 * 
 * 2. The ATtiny's external interrupt pin (INT0) is used by
 * the TinyWireS library. This is why the interrupt defined
 * in this sketch is attached to PCINT4.
 * 
 * The following pinout may be useful to identify the relevant
 * pins: http://homemadehardware.com/img/attiny85_pinout.jpeg
 * 
 * 3. This sketch SHOULD work on other AVR MCUs without too much
 * modification.
 */

#include <avr/io.h>
#include <util/atomic.h>
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

#define I2C_SLAVE_ADDR 0x6e
#define DEFAULT_CMD 0b00100000
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE (16)
#endif

void onI2CRequest();
void onI2CReceive(uint8_t howMany);
void onPinFalling();
void transmitReading();
void processCommand(const uint8_t cmd);
void setup();
void loop();

HX711::Channel ch = HX711::Channel::A;
HX711::HX_VALUE sensorReading = 0;
unsigned long whenLastRead = 0; //milliseconds
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

    hx.begin();

    processCommand(DEFAULT_CMD);

    attachPinChangeInterrupt(
        DATA_INT,
        onPinFalling,
        FALLING);

    TinyWireS.onReceive(onI2CReceive);
    TinyWireS.onRequest(onI2CRequest);
    TinyWireS.begin(I2C_SLAVE_ADDR);

}

void loop() {

    if(shouldUpdateSensor) {

        //at this stage, data pinchange interrupt is inactive
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            //even though the data pin pinchange interrupt
            //is disabled at this point, it is still
            //possible another interrupt could occur, particularly
            //in dealing with the I2C interface which uses the INT0
            //external interrupt. Further, whenLastRead should
            //really be as close as possible to when the sensor
            //reading was obtained
            sensorReading = hx.getValue(ch);
            whenLastRead = ::millis();
        }
        
        shouldUpdateSensor = false;
        enablePinChangeInterrupt(DATA_INT);

    }

    if(shouldSendReading) {
        transmitReading();
        shouldSendReading = false;
    }

    TinyWireS_stop_check();

}

void onI2CRequest() {
    shouldSendReading = true;
}

void onI2CReceive(uint8_t howMany) {

    //only one byte needed
    if(!TinyWireS.available() || howMany != 1) {
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

    HX711::HX_VALUE val;
    unsigned long nowMillis;

    //make a copy and then process
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = sensorReading;
        nowMillis = ::millis();
    }

    const unsigned long longDiff = static_cast<unsigned long>(
        nowMillis - whenLastRead
    );

    const uint16_t diff = min(UINT16_MAX, longDiff);

    //send time diff
    TinyWireS.send(static_cast<uint8_t>((diff >> 8) & 0xff));
    TinyWireS.send(static_cast<uint8_t>( diff       & 0xff));

    //send hx reading
    TinyWireS.send(static_cast<uint8_t>((val >> 16) & 0xff));
    TinyWireS.send(static_cast<uint8_t>((val >> 8)  & 0xff));
    TinyWireS.send(static_cast<uint8_t>( val        & 0xff));

}

void processCommand(const uint8_t cmd) {

    uint8_t gain =              (cmd >> 0) & 0b00000011;
    const uint8_t channel =     (cmd >> 2) & 0b00000001;
    const uint8_t bitF =        (cmd >> 3) & 0b00000001;
    const uint8_t byteF =       (cmd >> 4) & 0b00000001;
    const uint8_t pwr =         (cmd >> 5) & 0b00000001;

    //gain only has 3 selectable values, from 0 to 2
    //if the value from the slave is 3, default to 0
    if(gain == 3) {
        gain = 0;
    }

    ch = static_cast<HX711::Channel>(channel);
    hx.setBitFormat(static_cast<HX711::Format>(bitF));
    hx.setByteFormat(static_cast<HX711::Format>(byteF));

    if(pwr) {
        //if power on requested, set the gain AFTER powering on
        hx.powerUp();
        hx.setGain(static_cast<HX711::Gain>(gain));
    }
    else {
        
        //if power down requested, set the gain BEFORE powering down
        //this is probably useless, because the gain will reset after
        //powering back on
        hx.setGain(static_cast<HX711::Gain>(gain));
        hx.powerDown();

    }

}
