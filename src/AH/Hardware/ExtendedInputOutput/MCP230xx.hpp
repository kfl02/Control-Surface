#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include "ExtendedInputOutput.hpp"
#include "StaticSizeExtendedIOElement.hpp"
#include <AH/Containers/BitArray.hpp>

BEGIN_AH_NAMESPACE

/**
 * @brief   Class for MCP230xx I²C I/O expanders.
 *
 * @tparam  WireType
 *          The type of the I²C driver to use.
 *
 * @tparam  N
 *          The number of pin banks (1 = MCP23008, 2 = MCP230017)
 */
template <class WireType, uint16_t N>
class MCP230xx : public StaticSizeExtendedIOElement<N * 8> {
  static_assert(N != 1 || N != 2, "invalid number of banks");

  public:
    /**
     * @brief   Constructor.
     *
     * @param   wire
     *          The Wire/I²C interface to use.
     * @param   addressOffset
     *          A number between 0 and 7 reflecting the state of the I2C address
     *          pins of the MCP230xx.
     * @param   interruptPin
     *          The pin connected to the MCP230xx interrupt pin. This doesn't
     *          have to be an interrupt pin of the Arduino.
     *          The interrupt pin of the MCP230xx is configured as open-drain
     *          active-low output, and the interrupt pins of GPIO banks A and B
     *          are mirrored/OR'ed together. This means that you only need to
     *          connect a single interrupt pin for each MCP230xx, and you can
     *          connect the interrupt pins of multiple MCP230xxs to a single
     *          Arduino pin.
     */
    MCP230xx(WireType &wire, uint8_t addressOffset = 0,
             pin_t interruptPin = NO_PIN);

    void pinModeBuffered(pin_t pin, PinMode_t mode) override;
    void digitalWriteBuffered(pin_t pin, PinStatus_t status) override;
    PinStatus_t digitalReadBuffered(pin_t pin) override;
    analog_t analogReadBuffered(pin_t pin) override;
    void analogWriteBuffered(pin_t, analog_t) override;

    void begin() override;

    void updateBufferedOutputs() override;
    void updateBufferedInputs() override;
    /// Send the new pin modes to the chip after calling `pinModeBuffered`.
    void updateBufferedPinModes();

    /// Get the identifier of the given pin in register A.
    /// @param  p
    ///         Pin number in [0, 7]
    pin_t pinA(pin_t p) { return StaticSizeExtendedIOElement<N * 8>::pin(p); }

  private:
    constexpr static uint8_t I2C_BASE_ADDRESS = 0x20;

    WireType *wire;
    uint8_t address;
    pin_t interruptPin;

  private:
    bool pinModesDirty = true;
    BitArray<N * 8> bufferedPinModes;
    bool pullupsDirty = true;
    BitArray<N * 8> bufferedPullups;
    bool outputsDirty = true;
    BitArray<N * 8> bufferedOutputs;
    BitArray<N * 8> bufferedInputs;

  private:
    /// Check if any of the pins are configured as inputs.
    bool hasInputs() const;

    /// Write any data to the MCP230xx.
    template <size_t M>
    void writeI2C(const uint8_t (&values)[M]);

    /**
     * @brief  Write any data to the MCP230xx.
     *
     * @param   addr
     *          The address of the register to write to.
     * @param   values
     *          The values to write.
     */
    template <class... Args>
    void writeI2C(uint8_t addr, Args... values);
};

END_AH_NAMESPACE

#include "MCP230xx.ipp"

AH_DIAGNOSTIC_POP()