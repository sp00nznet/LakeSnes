
#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Input Input;

#include "snes.h"
#include "statehandler.h"

enum { inputDeviceNone = 0, inputDeviceController = 1, inputDeviceMouse = 2 };

struct Input {
  Snes* snes;
  uint8_t type; // inputDeviceController, inputDeviceMouse, etc.
  // latchline
  bool latchLine;
  // for controller
  uint16_t currentState; // actual state
  uint16_t latchedState;
  // for mouse
  uint8_t mouseButtons;      // bit 0 = left, bit 1 = right
  uint8_t mouseSpeed;        // 0=slow, 1=medium, 2=fast
  int16_t mouseAccumX;       // accumulated X displacement since last latch
  int16_t mouseAccumY;       // accumulated Y displacement since last latch
  uint32_t mouseLatchedData; // 32-bit latched mouse packet
  uint8_t mouseReadBit;      // serial read counter (0-31)
};

Input* input_init(Snes* snes);
void input_free(Input* input);
void input_reset(Input* input);
void input_handleState(Input* input, StateHandler* sh);
void input_latch(Input* input, bool value);
uint8_t input_read(Input* input);
void input_setMouse(Input* input, int16_t dx, int16_t dy, uint8_t buttons);

#endif
