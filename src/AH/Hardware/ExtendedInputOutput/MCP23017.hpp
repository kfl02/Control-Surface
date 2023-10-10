#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <AH/Hardware/ExtendedInputOutput/MCP230xx.hpp>

BEGIN_AH_NAMESPACE

/**
 * @brief   Class for MCP23017 I²C I/O expanders.
 * 
 * @tparam  WireType 
 *          The type of the I²C driver to use.
 */
template <class WireType>
class MCP23017 : public MCP230xx<WireType, 2> {
  public:
    /**
     * @brief   Constructor.
     * 
     * @param   wire
     *          The Wire/I²C interface to use.
     * @param   addressOffset
     *          A number between 0 and 7 reflecting the state of the I2C address
     *          pins of the MCP23017.
     * @param   interruptPin
     *          The pin connected to the MCP23017 interrupt pin. This doesn't 
     *          have to be an interrupt pin of the Arduino.  
     *          The interrupt pin of the MCP23017 is configured as open-drain 
     *          active-low output, and the interrupt pins of GPIO banks A and B
     *          are mirrored/OR'ed together. This means that you only need to 
     *          connect a single interrupt pin for each MCP23017, and you can 
     *          connect the interrupt pins of multiple MCP23017s to a single 
     *          Arduino pin.
     */
    MCP23017(WireType &wire, uint8_t addressOffset = 0,
             pin_t interruptPin = NO_PIN)
        : MCP230xx<WireType, 2>(wire, addressOffset, interruptPin) {};

    /// Get the identifier of the given pin in register B.
    /// @param  p
    ///         Pin number in [0, 7]
    pin_t pinB(pin_t p) { return StaticSizeExtendedIOElement<16>::pin(p + 8); }
};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()