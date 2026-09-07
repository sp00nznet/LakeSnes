
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "input.h"
#include "snes.h"
#include "statehandler.h"

Input* input_init(Snes* snes) {
  Input* input = malloc(sizeof(Input));
  input->snes = snes;
  input->type = inputDeviceController;
  input->currentState = 0;
  input->latchLine = false;
  input->latchedState = 0;
  input->mouseButtons = 0;
  input->mouseSpeed = 0;
  input->mouseAccumX = 0;
  input->mouseAccumY = 0;
  input->mouseLatchedData = 0;
  input->mouseReadBit = 0;
  input->scopeX = 0;
  input->scopeY = 0;
  input->scopeButtons = 0;
  input->scopeOffscreen = true;
  input->scopeLatched = false;
  return input;
}

void input_free(Input* input) {
  free(input);
}

void input_reset(Input* input) {
  input->latchLine = false;
  input->latchedState = 0;
  input->mouseAccumX = 0;
  input->mouseAccumY = 0;
  input->mouseLatchedData = 0;
  input->mouseReadBit = 0;
  input->scopeX = 0;
  input->scopeY = 0;
  input->scopeButtons = 0;
  input->scopeOffscreen = true;
  input->scopeLatched = false;
}

void input_handleState(Input* input, StateHandler* sh) {
  sh_handleBytes(sh, &input->type, &input->mouseButtons, &input->mouseSpeed, &input->mouseReadBit, &input->scopeButtons, NULL);
  sh_handleBools(sh, &input->latchLine, &input->scopeOffscreen, &input->scopeLatched, NULL);
  sh_handleWords(sh, &input->currentState, &input->latchedState, &input->scopeX, &input->scopeY, NULL);
  sh_handleWordsS(sh, &input->mouseAccumX, &input->mouseAccumY, NULL);
  sh_handleInts(sh, &input->mouseLatchedData, NULL);
}

static uint32_t mouse_build_packet(Input* input) {
  int dx = input->mouseAccumX;
  int dy = input->mouseAccumY;
  if(dx > 127) dx = 127;
  if(dx < -127) dx = -127;
  if(dy > 127) dy = 127;
  if(dy < -127) dy = -127;

  // Direction bit is a negative-sign flag: 1 = left / up, 0 = right / down.
  // (Verified against the Mario Paint ROM: with dx<0 reported as 0 the cursor
  // moved right, so 0 is the positive direction.)
  uint8_t xSign = (dx < 0) ? 1 : 0;
  uint8_t xMag = (dx < 0) ? (uint8_t)(-dx) : (uint8_t)dx;
  uint8_t ySign = (dy < 0) ? 1 : 0;
  uint8_t yMag = (dy < 0) ? (uint8_t)(-dy) : (uint8_t)dy;

  // 32-bit mouse packet, bit 31 read first (bit 31 = serial clock cycle 1).
  // Layout per the SNES Mouse protocol (snes.nesdev.org/wiki/Mouse):
  //   cycle 1-8   (bits 31-24)  zero
  //   cycle 9     (bit 23)      LEFT button
  //   cycle 10    (bit 22)      RIGHT button
  //   cycle 11-12 (bits 21-20)  sensitivity (0=slow, 1=medium, 2=fast)
  //   cycle 13-16 (bits 19-16)  controller ID nibble = 0001 for the mouse
  //                             (a standard pad reports 0000). This lands in
  //                             the low nibble of the auto-read word $4218,
  //                             and is how the game decides a mouse is present
  //                             and therefore clocks out the extra 16 movement
  //                             bits. Without it the mouse reads as a joypad:
  //                             buttons land on A/X and X/Y motion is never read.
  //   cycle 17    (bit 15)      Y sign: 1 = up (negative dy)
  //   cycle 18-24 (bits 14-8)   Y magnitude
  //   cycle 25    (bit 7)       X sign: 1 = left (negative dx)
  //   cycle 26-32 (bits 6-0)    X magnitude
  uint32_t data = 0;
  data |= ((uint32_t)(input->mouseButtons & 1)) << 23;        // left
  data |= ((uint32_t)((input->mouseButtons >> 1) & 1)) << 22; // right
  data |= ((uint32_t)(input->mouseSpeed & 3)) << 20;
  data |= ((uint32_t)1) << 16;  // controller ID 0001 = SNES Mouse
  data |= ((uint32_t)ySign) << 15;
  data |= ((uint32_t)yMag) << 8;
  data |= ((uint32_t)xSign) << 7;
  data |= ((uint32_t)xMag);
  return data;
}

static uint16_t scope_build_packet(Input* input) {
  // Super Scope 16-bit serial data (bit 0 = first read = auto-read bit 15):
  // Bits 0-3:  Noise (1 when on-screen, 0 when off)
  // Bit 4:     Fire
  // Bit 5:     Cursor
  // Bit 6:     Turbo
  // Bit 7:     Pause
  // Bit 8:     Offscreen flag
  // Bits 9-15: 0 (signature)
  uint16_t data = 0;
  if(!input->scopeOffscreen) data |= 0x000F; // noise bits indicate on-screen
  if(input->scopeButtons & 1) data |= 0x0010; // fire
  if(input->scopeButtons & 2) data |= 0x0020; // cursor
  if(input->scopeButtons & 4) data |= 0x0040; // turbo
  if(input->scopeButtons & 8) data |= 0x0080; // pause
  if(input->scopeOffscreen)   data |= 0x0100; // offscreen
  return data;
}

void input_latch(Input* input, bool value) {
  input->latchLine = value;
  if(input->latchLine) {
    if(input->type == inputDeviceMouse) {
      input->mouseLatchedData = mouse_build_packet(input);
      input->mouseAccumX = 0;
      input->mouseAccumY = 0;
      input->mouseReadBit = 0;
    } else if(input->type == inputDeviceSuperScope) {
      input->latchedState = scope_build_packet(input);
    } else {
      input->latchedState = input->currentState;
    }
  }
}

uint8_t input_read(Input* input) {
  if(input->type == inputDeviceMouse) {
    if(input->latchLine) {
      // While latch held, continuously re-latch and return first bit
      input->mouseLatchedData = mouse_build_packet(input);
      input->mouseReadBit = 0;
      return (input->mouseLatchedData >> 31) & 1;
    }
    if(input->mouseReadBit < 32) {
      uint8_t ret = (input->mouseLatchedData >> (31 - input->mouseReadBit)) & 1;
      input->mouseReadBit++;
      return ret;
    }
    return 1; // after 32 bits, return 1
  }
  // Standard controller and Super Scope both use 16-bit serial read
  if(input->type == inputDeviceSuperScope) {
    if(input->latchLine) input->latchedState = scope_build_packet(input);
  } else {
    if(input->latchLine) input->latchedState = input->currentState;
  }
  uint8_t ret = input->latchedState & 1;
  input->latchedState >>= 1;
  input->latchedState |= 0x8000;
  return ret;
}

void input_setMouse(Input* input, int16_t dx, int16_t dy, uint8_t buttons) {
  input->mouseAccumX += dx;
  input->mouseAccumY += dy;
  input->mouseButtons = buttons;
}

void input_setSuperScope(Input* input, uint16_t x, uint16_t y, uint8_t buttons) {
  input->scopeX = x;
  input->scopeY = y;
  input->scopeButtons = buttons;
  input->scopeOffscreen = (x >= 256 || y >= 224);
}
