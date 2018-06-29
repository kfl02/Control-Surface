#ifndef MIDI_Input_7SegmentDisplay_H_
#define MIDI_Input_7SegmentDisplay_H_

#include "MIDI_Input_Element.h"
#include "../ExtendedInputOutput/ExtendedInputOutput.h"

class MCU_7SegmentDisplay : public MIDI_Input_Element_CC
{
  public:
    MCU_7SegmentDisplay(uint8_t address, uint8_t length);
    ~MCU_7SegmentDisplay();

    bool updateImpl(const MIDI_message_matcher &midimsg);
    virtual void print() {}

  protected:
    const size_t length;
    char *textBuffer = nullptr;
};

//----------------------------------------------------------------------------------------------------------------------------------------//

class MCU_TimeDisplay : public MCU_7SegmentDisplay
{
  public:
    MCU_TimeDisplay();

    void update();
    void print();
    void getText(char *buff);
    void getBars(char *buff);
    void getBeats(char *buff);
    void getFrames(char *buff);
};

//----------------------------------------------------------------------------------------------------------------------------------------//

class MCU_AssignmentDisplay : public MCU_7SegmentDisplay
{
  public:
    MCU_AssignmentDisplay();

    void update();
    void print();
    void getText(char *buff);
};

#endif // MIDI_Input_7SegmentDisplay_H_