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

#include "HX711.h"
#include <Arduino.h>

namespace HX711 {

int32_t HX711::_convertFromTwosComplement(const int32_t val) {
    return -(val & 0x800000) + (val & 0x7fffff);
}

bool HX711::_readBit() const {

    volatile uint8_t* out = portOutputRegister(this->_clockPort);

    //first, clock pin is set high to make DOUT ready to be read from
    *out |= this->_clockPinBitmask;
    //digitalWrite(this->_clockPin, HIGH);

    //then delay for sufficient time to allow DOUT to be ready (0.1us)
    //this will also permit a sufficient amount of time for the clock
    //oin to remain high
    ::delayMicroseconds(1);
    
    //at this stage, DOUT is ready and the clock pin has been held
    //high for sufficient amount of time, so read the bit value
    //volatile uint8_t* in = portInputRegister(this->_dataPort);
    const bool bit = (*portInputRegister(this->_dataPort) & this->_dataPinBitmask) == this->_dataPinBitmask;
    //const bool bit = *portInputRegister(digitalPinToPort(this->_dataPin)) & digitalPinToBitMask(this->_dataPin);
    //const bool bit = digitalRead(this->_dataPin) == HIGH;
    //const bool bit = (*portInputRegister(digitalPinToPort(this->_dataPin)) & digitalPinToBitMask(this->_dataPin)) == digitalPinToBitMask(this->_dataPin);
    //Serial.println(*portInputRegister(this->_dataPort), BIN);

    //the clock pin then needs to be held for at least 0.2us before
    //the next bit can be read
    //bitClear(*portOutputRegister(digitalPinToPort(this->_clockPin)), digitalPinToBitMask(this->_clockPin));
    //digitalWrite(this->_clockPin, LOW);
    *out &= ~this->_clockPinBitmask;
    ::delayMicroseconds(1);

    return bit;

}

uint8_t HX711::_readByte() const {

    uint8_t val = 0;

    //8 bits per byte...
    for(uint8_t i = 0; i < 8; ++i) {
        if(this->_bitFormat == Format::MSB) {
            val <<= 1;
            val |= this->_readBit();
        }
        else {
            val >>= 1;
            val |= this->_readBit() * 0x80;
        }
    }

    return val;

}

void HX711::_readRawBytes(uint8_t* bytes) const {

    //delcare array of bytes of sufficient size
    //uninitialised is fine; they'll be overwritten
    uint8_t raw[_BYTES_PER_CONVERSION_PERIOD];

    //then populate it with values from the hx711
    for(uint8_t i = 0; i < _BYTES_PER_CONVERSION_PERIOD; ++i) {
        raw[i] = this->_readByte();
    }

    /**
     * The HX711 requires a certain number of "positive clock
     * pulses" depending on the set gain value.
     * Datasheet pg. 4
     * 
     * The expression below calculates the number of pulses
     * after having read the three bytes above. For example,
     * a gain of 128 requires 25 pulses: 24 pulses were made
     * when reading the three bytes (3 * 8), so only one
     * additional pulse is needed.
     */
    const uint8_t pulsesNeeded = 
        PULSES[static_cast<uint8_t>(this->_gain)] -
            8 * _BYTES_PER_CONVERSION_PERIOD;

    for(uint8_t i = 0; i < pulsesNeeded; ++i) {
        this->_readBit();
    }

    //if no byte pointer is given, don't try to write to it
    if(bytes == nullptr) {
        return;
    }

    /**
     * The HX711 will supply bits in big-endian format;
     * the 0th read bit is the MSB.
     * Datasheet pg. 4
     * 
     * If this->_byteFormat indicates the HX711 is outputting
     * bytes in LSB format, swap the first and last bytes
     * 
     * Remember, the bytes param expects an array of bytes
     * which will be converted to an int.
     */
    if(this->_byteFormat == Format::LSB) {
        const uint8_t swap = raw[0];
        raw[0] = raw[_BYTES_PER_CONVERSION_PERIOD - 1];
        raw[_BYTES_PER_CONVERSION_PERIOD - 1] = swap;
    }

    //finally, copy the local raw bytes to the byte array
    for(uint8_t i = 0; i < _BYTES_PER_CONVERSION_PERIOD; ++i) {
        bytes[i] = raw[i];
    }

}

HX_VALUE HX711::_readInt() const {

    uint8_t bytes[_BYTES_PER_CONVERSION_PERIOD];
    
    /**
     * Bytes are ready to be read from the HX711 when DOUT goes low.
     * Datasheet pg. 5
     * 
     * The - potential- issue is that DOUT going low does not appear to be
     * defined. It appears to occur whenever it is ready, whenever that is.
     */
    while(!this->isReady());
    
    /**
     * When DOUT goes low, there is a minimum of 0.1us until the clock pin
     * can go high. T1 in Fig.2.
     * Datasheet pg. 5
     * 0.1us == 100ns
     */
    ::delayMicroseconds(1);

    this->_readRawBytes(bytes);

    /**
     * An int (int32_t) is 32 bits (4 bytes), but
     * the HX711 only uses 24 bits (3 bytes).
     */
    const int32_t twosComp = (  (static_cast<int32_t>(      0 ) << 24) |
                                (static_cast<int32_t>(bytes[0]) << 16) |
                                (static_cast<int32_t>(bytes[1]) << 8)  |
                                 static_cast<int32_t>(bytes[2])         );

    return _convertFromTwosComplement(twosComp);

}

HX_VALUE HX711::_getChannelAValue() {

    /**
     * "Channel A can be programmed with a gain 
     * of 128 or 64..."
     * Datasheet pg. 1
     * 
     * Opt to default to 128
     */
    if(this->_gain == Gain::GAIN_32) {
        this->setGain(Gain::GAIN_128);
    }

    return this->_readInt();

}

HX_VALUE HX711::_getChannelBValue() {
    
    /**
     * "Channel B has a fixed gain of 32"
     * Datasheet pg. 1
     */
    if(this->_gain != Gain::GAIN_32) {
        this->setGain(Gain::GAIN_32);
    }

    return this->_readInt();

}

HX711::HX711(
    const uint8_t clockPin,
    const uint8_t dataPin)
        :   _clockPort(digitalPinToPort(clockPin)),
            _clockPinBitmask(digitalPinToBitMask(clockPin)),
            _dataPort(digitalPinToPort(dataPin)),
            _dataPinBitmask(digitalPinToBitMask(dataPin)) {
}

void HX711::begin() {

    volatile uint8_t* reg;
    
    //set data port+pin to input
    reg = portModeRegister(this->_dataPort);
    *reg &= ~this->_dataPinBitmask;

    //set clock port+pin to output
    reg = portModeRegister(this->_clockPort);
    *reg |= this->_clockPinBitmask;

    this->powerUp();

}

bool HX711::isReady() const {

    /**
     * HX711 will be "ready" when DOUT is low.
     * "Ready" means "data is ready for retrieval".
     * Datasheet pg. 4
     * 
     * This should be a one-shot test. Any follow-ups
     * or looping for checking if the sensor is ready
     * over time can/should be done by other calling code
     */
    return (*portInputRegister(this->_dataPort) & this->_dataPinBitmask) != this->_dataPinBitmask;

}

HX_VALUE HX711::getValue(const Channel c) {

    if(c == Channel::A) {
        return this->_getChannelAValue();
    }
    
    //else channel B
    return this->_getChannelBValue();

}

void HX711::setGain(const Gain gain) {

    this->_gain = gain;
    
    /**
     * A read must take place to set the gain at the
     * hardware level. See datasheet pg. 4 "Serial
     * Interface".
     */
    this->_readRawBytes();

}

Gain HX711::getGain() const {
    return this->_gain;
}

void HX711::powerDown() const {

    /**
     * "When PD_SCK pin changes from low to high
     * and stays at high for longer than 60µs, HX711
     * enters power down mode (Fig.3)."
     * Datasheet pg. 5
     */    
    volatile uint8_t* out = portOutputRegister(this->_clockPort);

    *out &= ~this->_clockPinBitmask;
    *out |= this->_clockPinBitmask;

    ::delayMicroseconds(60);

}

void HX711::powerUp() {
    
    /**
     * "When PD_SCK returns to low,
     * chip will reset and enter normal operation mode"
     * Datasheet pg. 5
     */
    *portOutputRegister(this->_clockPort) &= ~this->_clockPinBitmask;

    /**
     * "After a reset or power-down event, input
     * selection is default to Channel A with a gain of
     * 128."
     * Datasheet pg. 5
     * 
     * This means the following statement to set the gain
     * is needed ONLY IF the current gain isn't 128
     */
    if(this->_gain != Gain::GAIN_128) {
        this->setGain(this->_gain);
    }

}

};
