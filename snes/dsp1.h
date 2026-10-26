/*
 * DSP-1 (NEC uPD77C25 running Nintendo's DSP-1 program) — high-level emulation.
 *
 * (c) 2026 sp00nznet — MIT license
 *
 * Clean-room: written from the fullsnes documentation (problemkaputt.de) and the
 * SNESdev wiki DSP-1 page, plus black-box observation of the chip's I/O as seen
 * by Super Mario Kart. No emulator source code was consulted.
 */
#ifndef DSP1_H
#define DSP1_H

#include <stdint.h>
#include "statehandler.h"

typedef struct Dsp1 {
  uint16_t boundary;   // cart address at/above which the SR is mapped (DR below)
  uint8_t state;       // DSP1_IDLE / DSP1_INPUT / DSP1_OUTPUT / DSP1_RASTER
  uint8_t cmd;
  uint16_t pos;        // byte index into the current input or output block
  uint16_t nIn, nOut;  // words
  int16_t in[8];
  int16_t out[4];
  int16_t proj[7];     // last Parameter (02h) inputs: Fx Fy Fz Lfe Les Aas Azs
  int16_t att[3][4];   // Attitude A/B/C inputs: scale, Az, Ay, Ax
  int16_t rasterLine;  // next line for the streaming Raster (0Ah) output
} Dsp1;

extern Dsp1 dsp1;

void dsp1_init(void);
void dsp1_reset(void);
uint8_t dsp1_read(uint16_t adr);
void dsp1_write(uint16_t adr, uint8_t val);
void dsp1_handleState(StateHandler* sh);

#endif
