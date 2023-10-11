#pragma once

#include <AH/Settings/NamespaceSettings.hpp>
#include <AH/Settings/Warnings.hpp>

AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

AH_DIAGNOSTIC_EXTERNAL_HEADER()
#include <AH/Arduino-Wrapper.h>
AH_DIAGNOSTIC_POP()

#include <AH/Hardware/ExtendedInputOutput/ExtendedInputOutput.hpp>
#include <AH/Hardware/RegisterEncoders.hpp>
#include <AH/Hardware/MCP23xxxEncoders.hpp>

BEGIN_AH_NAMESPACE

/**
 * @brief   Class for reading 8 rotary encoders using a MCP23008 I²C port
 *          expander.
 *
 * Encoders are indexed by a number from 0 to 7. Encoder #0 is connected to pins
 * GPA0 and GPA1, Encoder #1 is connected to pins GPA2 and GPA3, ..., Encoder #7
 * is connected to pins GPB6 and GPB7.
 *
 * @tparam  WireType
 *          The type of the `Wire` I²C driver to use.
 * @tparam  EncoderPositionType
 *          The type used for saving the encoder positions. `int32_t` is the
 *          default because this matches the Encoder library. You can use small
 *          unsigned types such as `uint8_t` or `uint16_t` if you're just
 *          interrested in the deltas.
 * @tparam  InterruptSafe
 *          Make the `update` method safe to use inside of an interrupt.
 *          It makes the necessary variables `volatile` and disables interrupts
 *          while reading the positions from the main program.
 */
template <class WireType, class EncoderPositionType = int32_t,
          bool InterruptSafe = false>
class MCP23008Encoders : public MCP23xxxEncoders<WireType,
                                                 1,
                                                 EncoderPositionType,
                                                 InterruptSafe> {
  public:
    /**
     * @brief   Constructor.
     *
     * @param   wire
     *          The Wire/I²C interface to use.
     * @param   addr_offset
     *          A number between 0 and 7 reflecting the state of the I2C address
     *          pins of the MCP23xxx.
     * @param   interrupt_pin
     *          The pin connected to the MCP23xxx interrupt pin. This doesn't
     *          have to be an interrupt pin of the Arduino.
     *          The interrupt pin of the MCP23xxx is configured as open-drain
     *          active-low output, and the interrupt pins of GPIO banks A and B
     *          are mirrored/OR'ed together. This means that you only need to
     *          connect a single interrupt pin for each MCP23xxx, and you can
     *          connect the interrupt pins of multiple MCP23xxxs to a single
     *          Arduino pin.
     */
    MCP23008Encoders(WireType &wire, uint8_t addr_offset = 0,
                     pin_t interrupt_pin = NO_PIN)
        : MCP23xxxEncoders<WireType, 1, EncoderPositionType, InterruptSafe>(
              wire, addr_offset, interrupt_pin
          ) {};
}};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()
