#include "MCP230xx.hpp"

BEGIN_AH_NAMESPACE

enum MCP230xxRegAddr {
    IODIR = 0x00,
    IODIRA = 0x00,
    IODIRB = 0x01,
    IPOL = 0x01,
    IPOLA = 0x02,
    IPOLB = 0x03,
    GPINTEN = 0x02,
    GPINTENA = 0x04,
    GPINTENB = 0x05,
    DEFVAL = 0x03,
    DEFVALA = 0x06,
    DEFVALB = 0x07,
    INTCON = 0x04,
    INTCONA = 0x08,
    INTCONB = 0x09,
    IOCON = 0x05,
    IOCONA = 0x0A,
    IOCONB = 0x0B,
    GPPU = 0x06,
    GPPUA = 0x0C,
    GPPUB = 0x0D,
    INTF = 0x07,
    INTFA = 0x0E,
    INTFB = 0x0F,
    INTCAP = 0x08,
    INTCAPA = 0x10,
    INTCAPB = 0x11,
    GPIO = 0x09,
    GPIOA = 0x12,
    GPIOB = 0x13,
    OLAT = 0x0A,
    OLATA = 0x14,
    OLATB = 0x15,
};

template <class WireType, uint16_t N>
MCP230xx<WireType, N>::MCP230xx(WireType &wire, uint8_t addressOffset,
                                pin_t interruptPin)
    : wire(&wire), address(I2C_BASE_ADDRESS | addressOffset),
      interruptPin(interruptPin) {
    // Input mode by default
    bufferedPinModes.setByte(0, 0xFF);

    if (N == 2) {
        bufferedPinModes.setByte(1, 0xFF);
    }
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::pinModeBuffered(pin_t pin, PinMode_t mode) {
    if (mode == INPUT) {
        pinModesDirty |= bufferedPinModes.get(pin) == 0;
        pullupsDirty |= bufferedPullups.get(pin) == 1;
        bufferedPinModes.set(pin);
        bufferedPullups.clear(pin);
    } else if (mode == OUTPUT) {
        pinModesDirty |= bufferedPinModes.get(pin) == 1;
        bufferedPinModes.clear(pin);
    } else if (mode == INPUT_PULLUP) {
        pinModesDirty |= bufferedPinModes.get(pin) == 0;
        pullupsDirty |= bufferedPullups.get(pin) == 0;
        bufferedPinModes.set(pin);
        bufferedPullups.set(pin);
    }
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::digitalWriteBuffered(pin_t pin,
                                                 PinStatus_t status) {
    bool boolstate = status == HIGH;
    outputsDirty |= bufferedOutputs.get(pin) != boolstate;
    bufferedOutputs.set(pin, boolstate);
}

template <class WireType, uint16_t N>
PinStatus_t MCP230xx<WireType, N>::digitalReadBuffered(pin_t pin) {
    return bufferedInputs.get(pin) ? HIGH : LOW;
}

template <class WireType, uint16_t N>
analog_t MCP230xx<WireType, N>::analogReadBuffered(pin_t pin) {
    return bufferedInputs.get(pin) ? (1 << ADC_BITS) - 1 : 0;
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::analogWriteBuffered(pin_t pin, analog_t value) {
    digitalWriteBuffered(pin, value >= (1 << (ADC_BITS - 1)) ? HIGH : LOW);
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::begin() {
    if (interruptPin != NO_PIN)
        ExtIO::pinMode(interruptPin, INPUT_PULLUP);
    // Set the IOCON register (configuration register)
    if (N == 2) {
        // First make sure to set BANK = 1 on MSCP23017 in register 0x0B,
        // which is not a valid register when BANK is already 1
        writeI2C(IOCONB, //
                 0b11100100);
        //         │││││││└─ Unimplemented
        //         ││││││└── INTPOL = Active-low
        //         │││││└─── ODR    = Open-drain output
        //         │││││              (overrides the INTPOL bit)
        //         ││││└──── HAEN   = Disables the MCP23S17 address pins
        //         │││└───── DISSLW = Slew rate enabled
        //         ││└────── SEQOP  = Sequential operation disabled,
        //         ││                 address pointer does not increment
        //         │└─────── MIRROR = The INT pins are internally connected
        //         └──────── BANK   = The registers are in different banks
        //                            (addresses are segregated)
    }
    // on MCP23008, IOCON is always at register 0x05
    // on MCP23017, IOCON is guaranteed to be at 0x05 with the previous write
    // to 0x0B, so now switch to BANK = 0 again
    writeI2C(IOCON, //
             0b01100100);
    //         │││││││└─ Unimplemented
    //         ││││││└── INTPOL = Active-low
    //         │││││└─── ODR    = Open-drain output (overrides the INTPOL bit)
    //         ││││└──── HAEN   = Disables the MCP23S17 address pins
    //         │││└───── DISSLW = Slew rate enabled
    //         ││└────── SEQOP  = Sequential operation disabled,
    //         ││                 address pointer does not increment
    //         │└─────── MIRROR = The INT pins are internally connected
    //         └──────── BANK   = The registers are in the same bank
    //                            (addresses are sequential)
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::updateBufferedOutputs() {
    updateBufferedPinModes();
    if (!outputsDirty)
        return;
    if (N == 1) {
        writeI2C(GPIO,                      //
                 bufferedOutputs.getByte(0));
    } else {
        writeI2C(GPIOA,                      //
                 bufferedOutputs.getByte(0), //
                 bufferedOutputs.getByte(1));
    }
    outputsDirty = false;
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::updateBufferedInputs() {
    // Only update if at least one pin is configured as input
    if (!hasInputs())
        return;
    // Only update if a pin change interrupt happened
    if (interruptPin != NO_PIN && ExtIO::digitalRead(interruptPin) == HIGH)
        return;
    writeI2C(GPIO * N); // GPIO * N is GPIO on MCP23008 and GPIOA on MCP23017
    wire->requestFrom(address, size_t(N));
    bufferedInputs.setByte(0, wire->read());
    if (N == 2) {
        bufferedInputs.setByte(1, wire->read());
    }
}

template <class WireType, uint16_t N>
void MCP230xx<WireType, N>::updateBufferedPinModes() {
    if (pinModesDirty) {
        if (N == 1) {
            writeI2C(IODIR,                      //
                     bufferedPinModes.getByte(0));
            writeI2C(GPINTEN,                    //
                     bufferedPinModes.getByte(0));
        } else {
            writeI2C(IODIRA,                      //
                     bufferedPinModes.getByte(0), //
                     bufferedPinModes.getByte(1));
            writeI2C(GPINTENA,                    //
                     bufferedPinModes.getByte(0), //
                     bufferedPinModes.getByte(1));
        }
        pinModesDirty = false;
    }
    if (pullupsDirty) {
        if (N == 1) {
            writeI2C(GPPU,                       //
                     bufferedPullups.getByte(0));
        } else {
            writeI2C(GPPUA,                      //
                     bufferedPullups.getByte(0), //
                     bufferedPullups.getByte(1));
        }
        pullupsDirty = false;
    }
}

template <class WireType, uint16_t N>
bool MCP230xx<WireType, N>::hasInputs() const {
    if (N == 1) {
        return bufferedPinModes.getByte(0) != 0;
    } else {
        return bufferedPinModes.getByte(0) != 0
               || bufferedPinModes.getByte(1) != 0;
    }
}

template <class WireType, uint16_t N>
template <size_t M>
void MCP230xx<WireType, N>::writeI2C(const uint8_t (&values)[M]) {
    this->wire->beginTransmission(address);
    this->wire->write(values, M);
    this->wire->endTransmission();
}

template <class WireType, uint16_t N>
template <class... Args>
void MCP230xx<WireType, N>::writeI2C(uint8_t addr, Args... values) {
    const uint8_t v[] = {addr, static_cast<uint8_t>(values)...};
    writeI2C(v);
}

END_AH_NAMESPACE
