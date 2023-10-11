#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include "ExtendedInputOutput.hpp"
#include "StaticSizeExtendedIOElement.hpp"
#include <AH/Containers/Array.hpp>
#include <stdlib.h>

BEGIN_AH_NAMESPACE

/**
 * @brief   A class for MCP3xxx ADCs.
 *
 * @tparam  N
 *          The number of analog pins. Only 1, 2, 4 and 8 are valid.
 *
 * @tparam  R
 *          The ADC resolution. 10 for MCP30xx and 12 for MCP32xx.
 *
 * @tparam  S
 *          The size of an SPI request to read out the complete data.
 *          MCP3001/3002/3201 need two bytes, the others three.
 *
* @tparam  SPIDriver
*          The SPI class to use. Usually, the default is fine.
*
 * @ingroup AH_ExtIO
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 1, uint8_t R = 10, size_t S = 2>
class MCP3xxx : public StaticSizeExtendedIOElement<N> {
    static_assert(N != 1 || N != 2 || N != 4 || N != 8,
                  "invalid number of pins");
    static_assert(R != 10 || R != 12, "invalid resolution");

  protected:
    /**
     * @brief   Create a new MCP3xxx ADC object.
     *          Can only be called from subclasses.
     *
     * @param   spi
     *          The SPI interface to use.
     * @param   latchPin
     *          The digital output pin connected to the latch pin (ST_CP or
     *          RCLK) of the shift register.
     */
    MCP3xxx(SPIDriver spi, pin_t latchPin = SS)
        : latchPin(latchPin),
          spi(std::forward<SPIDriver>(spi)) {}

  public:
    /**
     * @brief   Returns the resolution in bits of the ADC.
     */
    uint8_t getResolution() {
        return R;
    }

    /**
     * @brief   The pinMode function is not implemented because the mode is
     *          `INPUT` by definition.
     */
    void pinMode(pin_t pin, PinMode_t mode) override
        __attribute__((deprecated)) {
        (void)pin;
        (void)mode;
    }

    /**
     * @copydoc pinMode
     */
    void pinModeBuffered(pin_t pin, PinMode_t mode) override
        __attribute__((deprecated)) {
        (void)pin;
        (void)mode;
    }

    /**
     * @brief   Read the digital state of the given input.
     * 
     * @param   pin
     *          The multiplexer's pin number to read from.
     */
    PinStatus_t digitalRead(pin_t pin) override {
        return analogRead(pin) > (1 << (R-1)) ? HIGH : LOW;
    }

    /**
     * @copydoc digitalRead
     */
    PinStatus_t digitalReadBuffered(pin_t pin) override {
        return MCP3xxx<SPIDriver, N, R, S>::digitalRead(pin);
    }

    /**
     * @brief   Read the analog value of the given input.
     * 
     * @param   pin
     *          The ADC's pin number to read from.
     */
    analog_t analogRead(pin_t pin) override {
        return readADC(pin, true);
    }

    /**
     * @brief   Read the difference of two analog input.
     *
     * The pin argument is interpreted as a combination of two input pins.
     *
     * Pin | IN+ | IN- | MCP3
     * ----|-----|-----|------------
     * 0   | 0   | 1   | x02/x04,x08
     * 1   | 1   | 0   | x02/x04,x08
     * 2   | 2   | 3   | x04,x08
     * 3   | 3   | 2   | x04,x08
     * 4   | 4   | 5   | x08
     * 5   | 5   | 4   | x08
     * 6   | 6   | 7   | x08
     * 7   | 7   | 6   | x08
     *
     * Note that if IN+ equal is less tan IN-, the result is 0.
     * On MCP3x01 the value of the single analog pin will be returned.
     *
     * @param   pin
     *          See above.
     */
    analog_t differentialRead(pin_t pin) {
        return readADC(pin, false);
    }

    /**
     * @copydoc analogRead
     */
    analog_t analogReadBuffered(pin_t pin) override {
        return analogRead(pin);
    }

    /**
     * @brief   The digitalWrite function is not implemented because writing an
     *          output to an ADC is not useful.
     */
    void digitalWrite(pin_t, PinStatus_t) override // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}             // LCOV_EXCL_LINE

    /**
     * @copydoc digitalWrite
     */
    void digitalWriteBuffered(pin_t, PinStatus_t) override // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}                     // LCOV_EXCL_LINE

    /**
     * @brief   The analogWrite function is not implemented because writing an
     *          output to an ADC is not useful.
     */
    void analogWrite(pin_t, analog_t) override // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}         // LCOV_EXCL_LINE

    /**
     * @copydoc analogWrite
     */
    void analogWriteBuffered(pin_t, analog_t) override // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}                 // LCOV_EXCL_LINE

    /**
     * @brief   Initialize the ADC.
     *          Setup the SPI interface, set the CS pin to output mode.
     */
    void begin() override {
        ExtIO::pinMode(this->latchPin, OUTPUT);
        spi.begin();
    }

    /**
     * @brief   No periodic updating of the state is necessary, all actions are
     *          carried out when the user calls analogRead or digitalRead.
     */
    void updateBufferedOutputs() override {} // LCOV_EXCL_LINE

    /**
     * @brief   No periodic updating of the state is necessary, all actions are
     *          carried out when the user calls analogRead or digitalRead.
     */
    void updateBufferedInputs() override {} // LCOV_EXCL_LINE

  protected:
    const pin_t latchPin;

    /**
     * @brief   Read an analog value from the ADC.
     *
     * @param   pin
     *          The pin(s) to read from.
     *
     * @param   single
     *          Read in single (true) or differential mode.
     */
    analog_t readADC(pin_t pin, bool single) {
        spi.beginTransaction(settings);
        ExtIO::digitalWrite(latchPin, LOW);

        uint8_t request[S];
        uint8_t result[S];

        buildRequest(pin - MCP3xxx<SPIDriver, N, R, S>::getStart(),
                     single, request);

        for(size_t i = 0; i < S; i++) {
            result[i] = spi.transfer(request[i]);
        }

        ExtIO::digitalWrite(latchPin, HIGH);
        spi.endTransaction();

        return fixResult(result) & ((1 << N) - 1);
    }

    /**
     * @brief   Build an SPI request for reading a specific pin.
     *
     * @param   pin
     *          The pin to read from.
     *
     * @param   single
     *          Read in single (true) or differential mode.
     *
     * @param   data
     *          An array to write the request to.
     */
    void virtual buildRequest(uint8_t pin, bool single, uint8_t data[S]) = 0;

    /**
     * @brief   Fix the SPI response to an analog_t.
     *          Especially the MCP3x01 responses have duplicated bits.
     *
     * @param   data
     *          An array containing the SPI response.
     */
    analog_t virtual fixResult(uint8_t data[S]) = 0;

  private:
    SPIDriver spi;

  public:
    SPISettings settings{SPI_MAX_SPEED, MSBFIRST, SPI_MODE0};
};

/**
 * @brief   Class for MCP3001 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 1,
          uint8_t R = 10,
          size_t S = 2>
class MCP3001 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3001(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0;
        data[1] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return (((data[0] & 0x1f) << 8) | data[1]) >> 3;
    }
};

/**
 * @brief   Class for MCP3002 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 2,
          uint8_t R = 10,
          size_t S = 2>
class MCP3002 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3002(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x44;
        if(single) {
            data[0] |= 0x20;
        }
        data[0] = pin << 4;
        data[1] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[0] & 0x03) << 8) | data[1];
    }
};

/**
 * @brief   Class for MCP3004 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 4,
          uint8_t R = 10,
          size_t S = 3>
class MCP3004 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3004(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x01;
        data[1] = pin << 4;
        if(single) {
            data[1] |= 0x80;
        }
        data[2] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[1] & 0x03) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3008 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 8,
          uint8_t R = 10,
          size_t S = 3>
class MCP3008 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3008(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x01;
        data[1] = pin << 4;
        if(single) {
            data[1] |= 0x80;
        }
        data[2] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[1] & 0x03) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3201 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 1,
          uint8_t R = 12,
          size_t S = 2>
class MCP3201 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3201(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0;
        data[1] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return (((data[0] & 0x1F) << 8) | data[1]) >> 1;
    }
};

/**
 * @brief   Class for MCP3202 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 2,
          uint8_t R = 12,
          size_t S = 3>
class MCP3202 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3202(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x01;
        data[1] = 0x20 | (pin << 6);
        if(single) {
            data[1] |= 0x80;
        }
        data[2] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3204 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 4,
          uint8_t R = 12,
          size_t S = 3>
class MCP3204 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3204(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x04;
        if(single) {
            data[0] |= 0x02;
        }
        if (pin > 3) {
            data[0] |= 0x01;
        }
        data[1] = pin << 6;
        data[2] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3208 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint8_t N = 8,
          uint8_t R = 12,
          size_t S = 3>
class MCP3208 : public MCP3xxx<SPIDriver, N, R, S> {
  public:
    MCP3208(SPIDriver spi, pin_t latchPin = SS)
        : MCP3xxx<SPIDriver, N, R, S>(spi, latchPin) {}

  protected:
    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) override {
        data[0] = 0x04;
        if(single) {
            data[0] |= 0x02;
        }
        if (pin > 3) {
            data[0] |= 0x01;
        }
        data[1] = pin << 6;
        data[2] = 0;
    }

    analog_t fixResult(uint8_t data[S]) override {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()
