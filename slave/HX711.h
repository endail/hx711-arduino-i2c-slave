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

#ifndef HX711_HX711_H_E90FC36E_BABC_455B_8936_69964EB52238
#define HX711_HX711_H_E90FC36E_BABC_455B_8936_69964EB52238

#include <stdint.h>

namespace HX711 {

/**
 * Datasheet
 * https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf
 */

/**
 * The HX711 is a 24-bit ADC. Values it outputs will always be
 * treated as 32-bit integers and not floating point numbers.
 */
typedef int32_t HX_VALUE;

enum class Format {
    MSB = 0, //most significant bit
    LSB //least significant bit
};

enum class Channel {
    A = 0,
    B
};

//Datasheet pg. 4
enum class Gain {
    GAIN_128 = 0,
    GAIN_32 = 1,
    GAIN_64 = 2
};

class HX711 {

protected:

    /**
     * Used as a map to select to correct number of clock pulses
     * depending on the set gain
     * Datasheet pg. 4
     */
    const uint8_t _PULSES[3] = {
        25,
        26,
        27
    };

    //Datasheet pg. 5
    //HX711 is a 24-bit ADC (ie. 3 8-bit values = 24 bits)
    static const uint8_t _BYTES_PER_CONVERSION_PERIOD = 3;

    volatile uint8_t* _clockPort;
    const uint8_t _clockPin;

    volatile uint8_t* _dataPort;
    const uint8_t _dataPin;

    Gain _gain = Gain::GAIN_128;
    Format _bitFormat = Format::MSB;
    Format _byteFormat = Format::MSB;

    static int32_t _convertFromTwosComplement(const int32_t val);
    bool _readBit() const;
    uint8_t _readByte() const;
    void _readRawBytes(uint8_t* bytes = nullptr) const;
    HX_VALUE _readInt() const;

    HX_VALUE _getChannelAValue();
    HX_VALUE _getChannelBValue();

public:
    
    HX711(
		volatile uint8_t* clockPort,
        const uint8_t clockPin,
        volatile uint8_t* dataPort,
        const uint8_t dataPin);

    virtual ~HX711() = default;

	void begin();
    void setGain(const Gain gain);
    Gain getGain() const;
    bool isReady() const;
    HX_VALUE getValue(const Channel c = Channel::A);

    Format getBitFormat() const;
    Format getByteFormat() const;
    void setBitFormat(const Format f);
    void setByteFormat(const Format f);

    void powerDown() const;
    void powerUp();

};
};
#endif