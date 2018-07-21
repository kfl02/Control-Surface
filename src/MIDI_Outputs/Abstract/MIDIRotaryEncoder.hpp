#pragma once

#if not defined(Encoder_h_) && not defined(IDE)
#error                                                                         \
    "The PJRC Encoder library should be included before the Control-Surface "  \
    "library. (#include <Encoder.h>)"
#endif

#include <Encoder.h>
#include <Helpers/Array.hpp>
#include <MIDI_Outputs/Abstract/AbstractMIDIOutput.hpp>

using EncoderPinList = Array<uint8_t, 2>;

/**
 * @brief   An abstract class for rotary encoders that send MIDI events.
 */
template <class Sender>
class MIDIRotaryEncoder : public AbstractMIDIOutput {
  protected:
    /**
     * @brief   Construct a new MIDIRotaryEncoder.
     *
     * @todo    Documentation
     */
    MIDIRotaryEncoder(const EncoderPinList &pins, uint8_t address,
                      uint8_t channel, uint8_t speedMultiply,
                      uint8_t pulsesPerStep)
        : encoder{pins[0], pins[1]}, address(address), channel(channel),
          speedMultiply(speedMultiply), pulsesPerStep(pulsesPerStep) {}

// For tests only
#ifndef ARDUINO
    MIDIRotaryEncoder(const Encoder &encoder, uint8_t address, uint8_t channel,
                      uint8_t speedMultiply, uint8_t pulsesPerStep)
        : encoder{encoder}, address(address), channel(channel),
          speedMultiply(speedMultiply), pulsesPerStep(pulsesPerStep) {}
#endif

  public:
    void update() final override {
        long currentPosition = encoder.read();
        long difference = (currentPosition - previousPosition) / pulsesPerStep;
        if (difference) {
            Sender::send(difference * speedMultiply, channel, address);
            previousPosition += difference * pulsesPerStep;
        }
    }

  private:
    Encoder encoder;
    const uint8_t address;
    const uint8_t channel;
    const uint8_t speedMultiply;
    const uint8_t pulsesPerStep;

    long previousPosition = 0;
};