
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "cart.h"
#include "snes.h"
#include "gsu.h"
#include "statehandler.h"
#include "dsp1.h"

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr);
static uint8_t cart_readExHirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readSuperFX(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeSuperFX(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readDsp1Lorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeDsp1Lorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readDsp1Hirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeDsp1Hirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

Cart* cart_init(Snes* snes) {
  Cart* cart = malloc(sizeof(Cart));
  cart->snes = snes;
  cart->type = 0;
  cart->rom = NULL;
  cart->romSize = 0;
  cart->ram = NULL;
  cart->ramSize = 0;
  cart->gsu = NULL;
  return cart;
}

void cart_free(Cart* cart) {
  if(cart->gsu != NULL) gsu_free(cart->gsu);
  if(cart->rom != NULL) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  free(cart);
}

void cart_reset(Cart* cart) {
  // do not reset ram, assumed to be battery backed
  if(cart->gsu != NULL) gsu_reset(cart->gsu);
}

bool cart_handleTypeState(Cart* cart, StateHandler* sh) {
  // when loading, return if values match
  if(sh->saving) {
    sh_handleBytes(sh, &cart->type, NULL);
    sh_handleInts(sh, &cart->romSize, &cart->ramSize, NULL);
    return true;
  } else {
    uint8_t type = 0;
    uint32_t romSize = 0;
    uint32_t ramSize = 0;
    sh_handleBytes(sh, &type, NULL);
    sh_handleInts(sh, &romSize, &ramSize, NULL);
    return !(type != cart->type || romSize != cart->romSize || ramSize != cart->ramSize);
  }
}

void cart_handleState(Cart* cart, StateHandler* sh) {
  if(cart->ram != NULL) sh_handleByteArray(sh, cart->ram, cart->ramSize);
  if(cart->type == 4 || cart->type == 5) dsp1_handleState(sh);
  if(cart->gsu != NULL) gsu_handleState(cart->gsu, sh);
}

void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize) {
  cart->type = type;
  if(cart->rom != NULL) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  cart->rom = malloc(romSize);
  cart->romSize = romSize;
  if(ramSize > 0) {
    cart->ram = malloc(ramSize);
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
  memcpy(cart->rom, rom, romSize);
  // Initialize DSP-1 if applicable
  if(type == 4) {
    dsp1_init();
    // Small LoROM (<=1MB): boundary at $C000
    dsp1.boundary = 0xC000;
  } else if(type == 5) {
    dsp1_init();
    // HiROM DSP-1: DR at $6000-$6FFF, SR at $7000-$7FFF
    dsp1.boundary = 0x7000;
  }
}

bool cart_handleBattery(Cart* cart, bool save, uint8_t* data, int* size) {
  if(save) {
    *size = cart->ramSize;
    if(data == NULL) return true;
    // assumes data is correct size
    if(cart->ram != NULL) memcpy(data, cart->ram, cart->ramSize);
    return true;
  } else {
    if(*size != cart->ramSize) return false;
    if(cart->ram != NULL) memcpy(cart->ram, data, cart->ramSize);
    return true;
  }
}

uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr) {
  switch(cart->type) {
    case 0: return cart->snes->openBus;
    case 1: return cart_readLorom(cart, bank, adr);
    case 2: return cart_readHirom(cart, bank, adr);
    case 3: return cart_readExHirom(cart, bank, adr);
    case 4: return cart_readDsp1Lorom(cart, bank, adr);
    case 5: return cart_readDsp1Hirom(cart, bank, adr);
    case 6: return cart_readSuperFX(cart, bank, adr);
  }
  return cart->snes->openBus;
}

void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  switch(cart->type) {
    case 0: break;
    case 1: cart_writeLorom(cart, bank, adr, val); break;
    case 2: cart_writeHirom(cart, bank, adr, val); break;
    case 3: cart_writeHirom(cart, bank, adr, val); break;
    case 4: cart_writeDsp1Lorom(cart, bank, adr, val); break;
    case 5: cart_writeDsp1Hirom(cart, bank, adr, val); break;
    case 6: cart_writeSuperFX(cart, bank, adr, val); break;
  }
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr) {
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  bank &= 0x7f;
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[((bank << 15) | (adr & 0x7fff)) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  if(((bank >= 0x70 && bank < 0x7e) || bank > 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
  }
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[(((bank & 0x3f) << 16) | adr) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static uint8_t cart_readExHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  if((bank & 0x7f) < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  bool secondHalf = bank < 0x80;
  bank &= 0x7f;
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[(((bank & 0x3f) << 16) | (secondHalf ? 0x400000 : 0) | adr) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}

/*
 * Super FX (GSU-2) cartridge, SNES side ("GSU2 Memory Map (at SNES Side)",
 * fullsnes):
 *   00-3F:3000-34FF  GSU registers (with mirrors) and 3100-32FF code cache
 *   00-3F:6000-7FFF  first 8K of Game Pak RAM
 *   00-3F:8000-FFFF  ROM, LoROM; 40-5F:0000-FFFF the same ROM, HiROM
 *   60-71:0000-FFFF  Game Pak RAM (70-71 per fullsnes; 60-6F kept for older callers)
 *   80-FF            mirrors
 * While the GSU runs with SCMR.RON set, ROM reads return the fixed vector
 * bytes (so IRQ/NMI land in WRAM); with SCMR.RAN set, RAM is not visible.
 */

static uint8_t superfx_vector(uint16_t adr) {
  if(adr & 1) return 0x01;
  switch(adr & 0xe) {
    case 0x4: return 0x04;
    case 0xa: return 0x08;
    case 0xe: return 0x0c;
  }
  return 0x00;
}

static uint8_t* superfx_ram(Cart* cart, uint8_t bank, uint16_t adr) {
  Gsu* g = cart->gsu;
  if(gsu_ownsRam(g)) return NULL;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000) return &g->ram[(adr - 0x6000) & (g->ram_size - 1)];
  if(bank >= 0x60 && bank < 0x72) return &g->ram[(((uint32_t)(bank & 0x0f) << 16) | adr) & (g->ram_size - 1)];
  return NULL;
}

static uint8_t cart_readSuperFX(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x3000 && adr < 0x3500) return gsu_read(cart->gsu, adr);
  uint8_t* ram = superfx_ram(cart, bank, adr);
  if(ram) return *ram;
  bool rom = (bank < 0x40 && adr >= 0x8000) || (bank >= 0x40 && bank < 0x60);
  if(rom && gsu_ownsRom(cart->gsu)) return superfx_vector(adr);
  if(bank < 0x40 && adr >= 0x8000) return cart->rom[((bank << 15) | (adr & 0x7fff)) & (cart->romSize - 1)];
  if(bank >= 0x40 && bank < 0x60) return cart->rom[(((bank - 0x40) << 16) | adr) & (cart->romSize - 1)];
  return cart->snes->openBus;
}

static void cart_writeSuperFX(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x3000 && adr < 0x3500) { gsu_write(cart->gsu, adr, val); return; }
  uint8_t* ram = superfx_ram(cart, bank, adr);
  if(ram) *ram = val;
}

/*
 * DSP-1 LoROM (small, <=1MB ROMs like Super Mario Kart, Pilotwings).
 * Two address ranges hit the DSP-1:
 *   1) Banks $20-3F/$A0-BF, adr $8000-$FFFF (boundary at $C000: DR below, SR at/above)
 *   2) Banks $00-1F/$80-9F, adr $6000-$7FFF (DR at $6000-$6FFF, SR at $7000-$7FFF)
 * Everything else is standard LoROM.
 */
static uint8_t cart_readDsp1Lorom(Cart* cart, uint8_t bank, uint16_t adr) {
  uint8_t b = bank & 0x7f;
  // DSP-1 register space (variant A): banks 20-3F, adr 8000-FFFF
  if(b >= 0x20 && b < 0x40 && adr >= 0x8000) {
    return dsp1_read(adr);
  }
  // DSP-1 register space (variant B): banks 00-1F, adr 6000-7FFF
  if(b < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    // Remap to match boundary convention: 6000-6FFF→DR (below boundary), 7000-7FFF→SR (at/above)
    uint16_t mapped = (adr < 0x7000) ? 0x8000 : 0xC000;
    return dsp1_read(mapped);
  }
  // SRAM: banks 70-7E, adr 0000-7FFF
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  // Standard LoROM
  b = bank & 0x7f;
  if(adr >= 0x8000 || b >= 0x40) {
    return cart->rom[((b << 15) | (adr & 0x7fff)) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeDsp1Lorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  uint8_t b = bank & 0x7f;
  // DSP-1 register space (variant A): banks 20-3F, adr 8000-FFFF
  if(b >= 0x20 && b < 0x40 && adr >= 0x8000) {
    dsp1_write(adr, val);
    return;
  }
  // DSP-1 register space (variant B): banks 00-1F, adr 6000-7FFF
  if(b < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    uint16_t mapped = (adr < 0x7000) ? 0x8000 : 0xC000;
    dsp1_write(mapped, val);
    return;
  }
  // SRAM
  if(((bank >= 0x70 && bank < 0x7e) || bank > 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
  }
}

/*
 * DSP-1 HiROM (Super Mario Kart, etc.).
 * DSP-1 registers at banks $00-1F/$80-9F, adr $6000-$7FFF:
 *   DR (data register)   at $6000-$6FFF
 *   SR (status register) at $7000-$7FFF
 * Everything else is standard HiROM.
 */
static uint8_t cart_readDsp1Hirom(Cart* cart, uint8_t bank, uint16_t adr) {
  uint8_t b = bank & 0x7f;
  // DSP-1 register space: banks 00-1F, adr 6000-7FFF
  if(b < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    return dsp1_read(adr);
  }
  // SRAM: banks 20-3F, adr 6000-7FFF (shifted up since DSP-1 takes 00-1F)
  if(b >= 0x20 && b < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    return cart->ram[(((b & 0x1f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  // Standard HiROM ROM
  if(adr >= 0x8000 || b >= 0x40) {
    return cart->rom[(((b & 0x3f) << 16) | adr) & (cart->romSize - 1)];
  }
  return cart->snes->openBus;
}

static void cart_writeDsp1Hirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  uint8_t b = bank & 0x7f;
  // DSP-1 register space: banks 00-1F, adr 6000-7FFF
  if(b < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    dsp1_write(adr, val);
    return;
  }
  // SRAM: banks 20-3F, adr 6000-7FFF
  if(b >= 0x20 && b < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    cart->ram[(((b & 0x1f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}
