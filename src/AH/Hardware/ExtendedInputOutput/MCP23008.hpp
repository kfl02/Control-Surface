#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <AH/Hardware/ExtendedInputOutput/MCP230xx.hpp>

BEGIN_AH_NAMESPACE

/**
 * @brief   Class for MCP23008 I²C I/O expanders.
 *
 * @tparam  WireType
 *          The type of the I²C driver to use.
 */
template <class WireType>
class MCP23008 : public MCP230xx<WireType, 1> {
  public:
    /**
     * @brief   Constructor.
     *
     * @param   wire
     *          The Wire/I²C interface to use.
     * @param   addressOffset
     *          A number between 0 and 7 reflecting the state of the I2C address
     *          pins of the MCP23008.
     * @param   interruptPin
     *          The pin connected to the MCP23008 interrupt pin. This doesn't
     *          have to be an interrupt pin of the Arduino.
     *          The interrupt pin of the MCP23008 is configured as open-drain
     *          active-low output, and the interrupt pins of GPIO banks A and B
     *          are mirrored/OR'ed together. This means that you only need to
     *          connect a single interrupt pin for each MCP23008, and you can
     *          connect the interrupt pins of multiple MCP23008s to a single
     *          Arduino pin.
     */
    MCP23008(WireType &wire, uint8_t addressOffset = 0,
             pin_t interruptPin = NO_PIN)
        : MCP230xx<WireType, 1>(wire, addressOffset, interruptPin) {};
};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()