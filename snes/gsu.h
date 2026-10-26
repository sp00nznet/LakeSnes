/*
 * Super FX / GSU-2 coprocessor for LakeSnes.
 *
 * Written from the fullsnes hardware documentation ("SNES Cart GSU-n"
 * chapters, problemkaputt.de/fullsnes.htm). No emulator source was consulted.
 *
 * The GSU is clocked alongside the CPU (gsu_tick from snes_runCycle), so the
 * 65816 really runs while the GSU draws, sees ROM/RAM handed over via SCMR,
 * and gets the STOP IRQ at the right moment.
 *
 * (c) 2026 sp00nznet - MIT license
 */

#ifndef GSU_H
#define GSU_H

#include <stdint.h>
#include <stdbool.h>

#include "statehandler.h"

typedef struct Gsu {
  void* snes;           // back-pointer (Snes*), for the CPU IRQ line

  uint16_t r[16];       // R0-R15, R15 = PC
  uint8_t latch;        // SNES-side write latch for R0-R15 low bytes

  // SFR flags
  bool z, cy, s, ov, go, alt1, alt2, b, irq;
  uint8_t sreg, dreg;   // TO/FROM/WITH selections

  uint8_t pbr, rombr, rambr;
  uint16_t cbr;
  uint8_t scbr, scmr, colr, por, bramr, cfgr, clsr;

  uint8_t pipe;         // prefetched opcode byte
  bool r15Written;      // set by an instruction that jumped

  uint8_t romBuf;       // ROM read buffer, [ROMBR:R14]
  uint16_t ramAddr;     // last RAM address (for SBK)

  uint8_t cache[512];
  bool cacheValid[32];

  int32_t budget;       // master cycles owed to the GSU

  uint8_t* rom;
  uint32_t romSize;
  uint8_t* ram;         // Game Pak RAM (banks 70-71)
  uint32_t ram_size;
} Gsu;

Gsu* gsu_init(void* snes);
void gsu_free(Gsu* gsu);
void gsu_reset(Gsu* gsu);
void gsu_power(Gsu* gsu);
void gsu_load(Gsu* gsu, uint8_t* rom, uint32_t romSize, uint8_t* ram, uint32_t ramSize);

// SNES side: $3000-$34FF registers (minus the cache window) and $3100-$32FF cache
uint8_t gsu_read(Gsu* gsu, uint16_t adr);
void gsu_write(Gsu* gsu, uint16_t adr, uint8_t val);
uint8_t gsu_cache_read(Gsu* gsu, uint16_t adr);
void gsu_cache_write(Gsu* gsu, uint16_t adr, uint8_t val);

// advance the GSU by this many master cycles (no-op while stopped)
void gsu_tick(Gsu* gsu, int masterCycles);
// run until STOP, for callers outside the timed loop (snesrecomp's shell mode)
void gsu_run(Gsu* gsu);

// while running, the GSU owns the ROM/RAM bus the SNES would otherwise see
bool gsu_ownsRom(Gsu* gsu);
bool gsu_ownsRam(Gsu* gsu);
bool gsu_irqPending(Gsu* gsu);

void gsu_handleState(Gsu* gsu, StateHandler* sh);

#endif
