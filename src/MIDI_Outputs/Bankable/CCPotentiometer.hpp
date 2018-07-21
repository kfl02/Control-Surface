#pragma once

#include <MIDI_Outputs/Bankable/Abstract/MIDIFilteredAnalog.hpp>
#include <MIDI_Senders/ContinuousCCSender.hpp>

namespace Bankable {

class CCPotentiometer
    : public MIDIFilteredAnalogAddressable<ContinuousCCSender, 7> {
  public:
    CCPotentiometer(pin_t analogPin, uint8_t controller, uint8_t channel = 1)
        : MIDIFilteredAnalogAddressable<ContinuousCCSender, 7>(
              analogPin, controller, channel) {}
};

} // namespace Bankable