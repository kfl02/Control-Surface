#include <Arduino.h>

#include "Selector.h"

using namespace ExtIO;

Selector *Selector::last = nullptr;
Selector *Selector::first = nullptr;

void Selector::update() {
    uint8_t newSetting = Setting;
    switch (mode) {
    case SINGLE_SWITCH:
    case SINGLE_SWITCH_LED: {
        newSetting = (uint8_t)(!digitalRead(dbButton1.pin));
    } break;

    case SINGLE_BUTTON:
    case SINGLE_BUTTON_LED: {
        if (debounceButton(dbButton1))
            newSetting = !Setting; // Toggle Setting between 0 and 1
    } break;

    case MULTIPLE_BUTTONS:
    case MULTIPLE_BUTTONS_LEDS: {
        for (uint8_t i = 0; i < nb_settings; i++) {
            if (digitalRead(switchPins[i]) == LOW) {
                newSetting = i;
                break;
            }
        }
    } break;

    case INCREMENT_DECREMENT_LEDS:
    case INCREMENT_DECREMENT: {
        if (debounceButton(dbButton1))
            newSetting = Setting < nb_settings - 1
                             ? Setting + 1
                             : 0; // Increment Setting number or wrap around
        if (debounceButton(dbButton2))
            newSetting =
                Setting == 0
                    ? nb_settings - 1
                    : Setting - 1; // Decrement Setting number or wrap around
    } break;

    case INCREMENT_LEDS:
    case INCREMENT: {
        if (debounceButton(dbButton1))
            newSetting = Setting < nb_settings - 1
                             ? Setting + 1
                             : 0; // Increment Setting number or wrap around
    } break;
    }

    if (newSetting != Setting) {
        setSetting(newSetting);
    } else if (firstRefresh) {
        updateImpl(Setting);
        firstRefresh = false;
    }
}

uint8_t Selector::getSetting() { return Setting; }
void Selector::setSetting(uint8_t newSetting) {
    updateLEDs(newSetting);
    Setting = newSetting;
    updateImpl(Setting);
}

const char *Selector::getMode() {
    switch (mode) {
    case SINGLE_BUTTON:
        return "SINGLE_BUTTON";
    case SINGLE_BUTTON_LED:
        return "SINGLE_BUTTON_LED";
    case SINGLE_SWITCH:
        return "SINGLE_SWITCH";
    case SINGLE_SWITCH_LED:
        return "SINGLE_SWITCH_LED";
    case MULTIPLE_BUTTONS:
        return "MULTIPLE_BUTTONS";
    case MULTIPLE_BUTTONS_LEDS:
        return "MULTIPLE_BUTTONS_LEDS";
    case INCREMENT_DECREMENT:
        return "INCREMENT_DECREMENT";
    case INCREMENT_DECREMENT_LEDS:
        return "INCREMENT_DECREMENT_LEDS";
    case INCREMENT:
        return "INCREMENT";
    case INCREMENT_LEDS:
        return "INCREMENT_LEDS";
    }
    return ""; // Keeps the compiler happy
}

void Selector::initLEDs() {
    for (uint8_t i = 0; i < nb_settings; i++)
        pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[0], HIGH);
}

void Selector::updateLEDs(uint8_t newSetting) {
    switch (mode) {
    case SINGLE_SWITCH_LED:
    case SINGLE_BUTTON_LED: {
        digitalWrite(ledPin, newSetting);
    } break;

    case MULTIPLE_BUTTONS_LEDS:
    case INCREMENT_DECREMENT_LEDS:
    case INCREMENT_LEDS: {
        digitalWrite(ledPins[Setting], LOW);
        digitalWrite(ledPins[newSetting], HIGH);
    } break;
    default:
        break;
    }
}

bool Selector::debounceButton(debouncedButton &button) {
    bool pressed = false;
    bool state = digitalRead(button.pin);

    int8_t stateChange = state - button.prevState;

    if (stateChange == falling) { // Button is pushed
        if (millis() - prevBounceTime > debounceTime) {
            pressed = true;
            prevBounceTime = millis();
        }
    }
    if (stateChange == rising) { // Button is released
        prevBounceTime = millis();
    }
    button.prevState = state;
    return pressed;
}