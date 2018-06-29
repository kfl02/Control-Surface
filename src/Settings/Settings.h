#ifndef SETTINGSWRAPPER_H
#error "Do not include this file directly, use the wrapper!"
#endif

/**
 * @file Settings.h
 * 
 * @brief All user settings and debugging options can be changed here.
 */

// ---------------------------- Debug Settings --------------------------- //
// ======================================================================= //

/* Print all debug messages */
// #define DEBUG

/* Print all incoming MIDI messages */
// #define DEBUG_MIDI_PACKETS

/* Print timing/profiling information */
// #define DEBUG_TIME

/* The default debug output */
#define DEBUG_OUT Serial1

// ---------------------------- User Settings ---------------------------- //
// ======================================================================= //

/** The factor for the analog filter: <br>
    Difference equation: @f$ y[n] = k*x[n] + (1-k)*y[n-1] @f$
    where @f$ k = \left(\frac{1}{2}\right)^{ANALOG\_FILTER\_SHIFT\_FACTOR} @f$ */
#define ANALOG_FILTER_SHIFT_FACTOR 2

/** The signed integer type to use for analog inputs during filtering. 
 * Should be at least 11 + 2*ANALOG_FILTER_SHIFT_FACTOR bits wide. 
 * (10 bits of analog pricision, 1 sign bit, and 2*ANALOG_FILTER_SHIFT_FACTOR bits
 * of fixed point precision for intermediate filter calculations. */
#define ANALOG_FILTER_TYPE int16_t

/** The debounce time for momentary push buttons in milliseconds */
#define BUTTON_DEBOUNCE_TIME 25

/** The interval between updating all inputs, in microseconds (µs). */
#define INPUT_UPDATE_INTERVAL 1000 // 1 ms

/** Don't parse System Exclusive messages 
 * @todo Make sure that this is implemented everywhere.
*/
#define IGNORE_SYSEX

/** The length of the maximum System Exclusive message 
    that can be received. The maximum length sent by
    the MCU protocol is 120 bytes. */
#define SYSEX_BUFFER_SIZE 128

/** The baud rate to use for Hairless MIDI */
#define HAIRLESS_BAUD 115200

/** The maximum frame rate of the displays */
#define MAX_FPS 30

// ======================================================================= //