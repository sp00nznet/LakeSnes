/*
 * Super FX / GSU-2 coprocessor for LakeSnes.
 *
 * Written from the fullsnes hardware documentation ("SNES Cart GSU-n" I/O,
 * opcode, cache and bitmap chapters, problemkaputt.de/fullsnes.htm). No
 * emulator source was consulted. Section names below refer to fullsnes.
 *
 * Model:
 *  - One opcode per step. `pipe` holds the prefetched byte; a step fetches the
 *    byte at R15 into the pipe before executing, so any jump (Bxx, JMP, LOOP,
 *    or an ALU/MOV with R15 as destination) executes the following byte first,
 *    as "Jump Notes" describes. During execution R15 = address of next opcode.
 *  - Prefixes (ALT1/2/3, TO, WITH, FROM) persist into the next opcode; every
 *    other opcode resets them except Bxx ("GSU Prefix Opcodes").
 *  - Missing ALT variants fall back: ALT3 -> ALT1 -> plain ("Ignored Prefixes").
 *  - Timing is approximate: 1 clock per cached opcode byte, 3/5 clocks
 *    (10/21 MHz) per uncached ROM/RAM byte, plus the documented multiply cost.
 *    ponytail: no ROM/RAM wait states or RON/RAN stalls; add them if a game
 *    turns out to depend on them.
 *  - PLOT writes straight to RAM. The two-row pixel cache only changes timing,
 *    and RPIX/STOP would flush it anyway.
 *
 * (c) 2026 sp00nznet - MIT license
 */

#include <stdlib.h>
#include <string.h>

#include "gsu.h"
#include "snes.h"

#define R15 (g->r[15])

static int g_cost; // GSU clocks spent by the current step

// ---- GSU-side memory ("GSU2 Memory Map (at GSU Side)") ----

static uint8_t bus_read(Gsu* g, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40) return g->rom[(((uint32_t)bank << 15) | (adr & 0x7fff)) & (g->romSize - 1)];
  if(bank < 0x60) return g->rom[(((uint32_t)(bank - 0x40) << 16) | adr) & (g->romSize - 1)];
  if(bank >= 0x70) return g->ram[(((uint32_t)(bank & 1) << 16) | adr) & (g->ram_size - 1)];
  return 0;
}

static int mem_clocks(Gsu* g) { return (g->clsr & 1) ? 5 : 3; }

static uint8_t ram_read(Gsu* g, uint16_t adr) {
  g_cost += mem_clocks(g);
  return g->ram[(((uint32_t)g->rambr << 16) | adr) & (g->ram_size - 1)];
}

static void ram_write(Gsu* g, uint16_t adr, uint8_t val) {
  g_cost += mem_clocks(g);
  g->ram[(((uint32_t)g->rambr << 16) | adr) & (g->ram_size - 1)] = val;
}

// words at odd addresses access [adr^1] for the high byte ("CPU MOV Opcodes")
static uint16_t ram_read16(Gsu* g, uint16_t adr) {
  g->ramAddr = adr;
  return ram_read(g, adr) | (ram_read(g, adr ^ 1) << 8);
}

static void ram_write16(Gsu* g, uint16_t adr, uint16_t val) {
  g->ramAddr = adr;
  ram_write(g, adr, val & 0xff);
  ram_write(g, adr ^ 1, val >> 8);
}

// ---- code cache ("Code-Cache") ----

static void cache_flush(Gsu* g) {
  memset(g->cacheValid, 0, sizeof(g->cacheValid));
}

static uint8_t fetch(Gsu* g, uint16_t adr) {
  if((uint16_t)(adr - g->cbr) < 0x200) {
    int line = (adr & 0x1ff) >> 4;
    if(!g->cacheValid[line]) {
      uint16_t base = adr & 0xfff0;
      for(int i = 0; i < 16; i++) g->cache[(base + i) & 0x1ff] = bus_read(g, g->pbr, base + i);
      g->cacheValid[line] = true;
      g_cost += 16 * mem_clocks(g);
    }
    g_cost += 1;
    return g->cache[adr & 0x1ff];
  }
  g_cost += mem_clocks(g);
  return bus_read(g, g->pbr, adr);
}

// next immediate byte: consumes the pipe and refills it from the following address
static uint8_t operand(Gsu* g) {
  uint8_t v = g->pipe;
  R15++;
  g->pipe = fetch(g, R15);
  return v;
}

// ---- registers & flags ----

static void set_reg(Gsu* g, int n, uint16_t v) {
  g->r[n] = v;
  if(n == 14) g->romBuf = bus_read(g, g->rombr, v); // "ROM-Read-Data Cache"
  if(n == 15) g->r15Written = true;
}

static uint16_t sreg(Gsu* g) { return g->r[g->sreg]; }
static void dreg(Gsu* g, uint16_t v) { set_reg(g, g->dreg, v); }

static void sz(Gsu* g, uint16_t v) {
  g->s = (v & 0x8000) != 0;
  g->z = v == 0;
}

static void update_irq_line(Gsu* g) {
  Snes* snes = (Snes*)g->snes;
  if(snes) cpu_setIrq(snes->cpu, snes->inIrq || gsu_irqPending(g));
}

// ---- bitmap ("Bitmap I/O Ports") ----

static int bpp(Gsu* g) {
  switch(g->scmr & 3) {
    case 0: return 2;
    case 3: return 8;
    default: return 4;
  }
}

static uint32_t row_addr(Gsu* g, uint8_t x, uint8_t y) {
  int ht = (g->por & 0x10) ? 3 : (((g->scmr >> 2) & 1) | ((g->scmr >> 4) & 2));
  uint32_t tile;
  switch(ht) {
    case 0: tile = (x >> 3) * 0x10 + (y >> 3); break;
    case 1: tile = (x >> 3) * 0x14 + (y >> 3); break;
    case 2: tile = (x >> 3) * 0x18 + (y >> 3); break;
    default: tile = (y >> 7) * 0x200 + (x >> 7) * 0x100 + ((y >> 3) & 0xf) * 0x10 + ((x >> 3) & 0xf); break;
  }
  return tile * bpp(g) * 8 + g->scbr * 0x400 + (y & 7) * 2;
}

static void plot(Gsu* g) {
  uint8_t x = g->r[1], y = g->r[2];
  uint8_t c = g->colr;
  int depth = bpp(g);
  if((g->por & 2) && depth != 8 && ((x ^ y) & 1)) c >>= 4;
  if(!(g->por & 1)) {
    uint8_t mask = depth == 2 ? 3 : (depth == 4 || (g->por & 8)) ? 0xf : 0xff;
    if(!(c & mask)) return;
  }
  uint32_t row = row_addr(g, x, y);
  uint8_t bit = 0x80 >> (x & 7);
  for(int p = 0; p < depth; p++) {
    uint8_t* b = &g->ram[(row + (p >> 1) * 16 + (p & 1)) & (g->ram_size - 1)];
    *b = (c >> p) & 1 ? (*b | bit) : (*b & ~bit);
  }
  g_cost += depth / 2;
}

static uint8_t rpix(Gsu* g) {
  uint8_t x = g->r[1], y = g->r[2];
  uint32_t row = row_addr(g, x, y);
  uint8_t c = 0;
  for(int p = 0; p < bpp(g); p++) {
    uint8_t b = g->ram[(row + (p >> 1) * 16 + (p & 1)) & (g->ram_size - 1)];
    c |= ((b >> (7 - (x & 7))) & 1) << p;
  }
  g_cost += 20;
  return c;
}

static void set_color(Gsu* g, uint8_t v) {
  if(g->por & 4) v = (v & 0xf0) | (v >> 4);
  g->colr = (g->por & 8) ? (g->colr & 0xf0) | (v & 0x0f) : v;
}

// ---- ALU helpers ("CPU ALU Opcodes") ----

static uint16_t add(Gsu* g, uint16_t a, uint16_t b, int c) {
  uint32_t r = a + b + c;
  g->ov = (~(a ^ b) & (a ^ r) & 0x8000) != 0;
  g->cy = r > 0xffff;
  sz(g, r);
  return r;
}

static uint16_t sub(Gsu* g, uint16_t a, uint16_t b, int borrow) {
  int32_t r = (int32_t)a - b - borrow;
  g->ov = ((a ^ b) & (a ^ r) & 0x8000) != 0;
  g->cy = r >= 0;
  sz(g, r);
  return r;
}

// ---- execution ----

static void exec(Gsu* g, uint8_t op) {
  int n = op & 0xf;
  bool a1 = g->alt1, a2 = g->alt2;
  uint16_t s = sreg(g);

  switch(op >> 4) {
    case 0x0: {
      if(op >= 0x05) { // Bxx: prefixes survive
        int8_t off = operand(g);
        bool take;
        switch(op) {
          case 0x05: take = true; break;
          case 0x06: take = g->s == g->ov; break;
          case 0x07: take = g->s != g->ov; break;
          case 0x08: take = !g->z; break;
          case 0x09: take = g->z; break;
          case 0x0a: take = !g->s; break;
          case 0x0b: take = g->s; break;
          case 0x0c: take = !g->cy; break;
          case 0x0d: take = g->cy; break;
          case 0x0e: take = !g->ov; break;
          default: take = g->ov; break;
        }
        if(take) set_reg(g, 15, R15 + off);
        return;
      }
      if(op == 0x00) { // STOP
        g->go = false;
        g->irq = true;
        update_irq_line(g);
      } else if(op == 0x02) { // CACHE
        if(g->cbr != (R15 & 0xfff0)) {
          g->cbr = R15 & 0xfff0;
          cache_flush(g);
        }
      } else if(op == 0x03) { // LSR
        g->cy = s & 1;
        uint16_t r = s >> 1;
        dreg(g, r); sz(g, r);
      } else if(op == 0x04) { // ROL
        uint16_t r = (s << 1) | g->cy;
        g->cy = s >> 15;
        dreg(g, r); sz(g, r);
      }
      break;
    }
    case 0x1: // TO Rn, or MOVE Rn,Rs after WITH
      if(g->b) { set_reg(g, n, s); break; }
      g->dreg = n;
      return;
    case 0x2: // WITH Rn
      g->sreg = g->dreg = n;
      g->b = true;
      return;
    case 0x3:
      if(n < 12) { // STW (Rn) / STB (Rn)
        if(a1) { g->ramAddr = g->r[n]; ram_write(g, g->r[n], s & 0xff); }
        else ram_write16(g, g->r[n], s);
      } else if(n == 12) { // LOOP
        uint16_t r = g->r[12] - 1;
        set_reg(g, 12, r); sz(g, r);
        if(!g->z) set_reg(g, 15, g->r[13]);
      } else { // ALT1/ALT2/ALT3 prefixes
        g->alt1 = n & 1;
        g->alt2 = n == 0xe || n == 0xf;
        return;
      }
      break;
    case 0x4:
      if(n < 12) { // LDW (Rn) / LDB (Rn)
        if(a1) { g->ramAddr = g->r[n]; dreg(g, ram_read(g, g->r[n])); }
        else dreg(g, ram_read16(g, g->r[n]));
      } else if(n == 12) {
        if(a1) { uint8_t c = rpix(g); dreg(g, c); sz(g, c); } // RPIX
        else { plot(g); set_reg(g, 1, g->r[1] + 1); }          // PLOT
      } else if(n == 13) { // SWAP
        uint16_t r = (s >> 8) | (s << 8);
        dreg(g, r); sz(g, r);
      } else if(n == 14) {
        if(a1) g->por = s & 0x1f; // CMODE
        else set_color(g, s);     // COLOR
      } else { // NOT
        uint16_t r = ~s;
        dreg(g, r); sz(g, r);
      }
      break;
    case 0x5: { // ADD / ADC, register or #n
      uint16_t b = a2 ? n : g->r[n];
      dreg(g, add(g, s, b, a1 ? g->cy : 0));
      break;
    }
    case 0x6: { // SUB Rn / SBC Rn / SUB #n / CMP Rn
      if(a1 && a2) sub(g, s, g->r[n], 0);
      else if(a2) dreg(g, sub(g, s, n, 0));
      else dreg(g, sub(g, s, g->r[n], a1 ? !g->cy : 0));
      break;
    }
    case 0x7: {
      uint16_t r;
      if(n == 0) r = (g->r[7] & 0xff00) | (g->r[8] >> 8); // MERGE
      else {
        uint16_t b = a2 ? n : g->r[n];
        r = a1 ? (s & ~b) : (s & b); // BIC / AND
      }
      dreg(g, r);
      if(n == 0) { // MERGE flags ("CPU ALU Opcodes")
        g->s = (r & 0x8080) != 0;
        g->ov = (r & 0xc0c0) != 0;
        g->cy = (r & 0xe0e0) != 0;
        g->z = (r & 0xf0f0) != 0;
      } else sz(g, r);
      break;
    }
    case 0x8: { // MULT / UMULT, register or #n
      uint8_t b = a2 ? n : (g->r[n] & 0xff);
      uint16_t r = a1 ? (uint16_t)((s & 0xff) * b) : (uint16_t)((int8_t)s * (int8_t)b);
      dreg(g, r); sz(g, r);
      if(!(g->cfgr & 0x20) && !(g->clsr & 1)) g_cost += 1;
      break;
    }
    case 0x9:
      if(n == 0) { // SBK
        ram_write16(g, g->ramAddr, s);
      } else if(n <= 4) { // LINK #n
        set_reg(g, 11, R15 + n);
      } else if(n == 5) { // SEX
        uint16_t r = (int8_t)s;
        dreg(g, r); sz(g, r);
      } else if(n == 6) { // ASR / DIV2
        g->cy = s & 1;
        uint16_t r = (a1 && s == 0xffff) ? 0 : (uint16_t)((int16_t)s >> 1);
        dreg(g, r); sz(g, r);
      } else if(n == 7) { // ROR
        uint16_t r = (s >> 1) | (g->cy << 15);
        g->cy = s & 1;
        dreg(g, r); sz(g, r);
      } else if(n <= 13) {
        if(a1) { // LJMP: PBR=Rn, R15=Rs, reload cache
          g->pbr = g->r[n] & 0x7f;
          set_reg(g, 15, s);
          g->cbr = s & 0xfff0;
          cache_flush(g);
        } else set_reg(g, 15, g->r[n]); // JMP
      } else if(n == 14) { // LOB
        uint16_t r = s & 0xff;
        dreg(g, r);
        g->s = (r & 0x80) != 0; g->z = r == 0;
      } else { // FMULT / LMULT
        int32_t r = (int16_t)s * (int16_t)g->r[6];
        if(a1) set_reg(g, 4, r & 0xffff);
        dreg(g, r >> 16);
        g->cy = (r >> 15) & 1;
        sz(g, r >> 16);
        g_cost += (g->cfgr & 0x20) ? 3 : 7;
      }
      break;
    case 0xa: // IBT Rn,#pp / LMS Rn,(kk) / SMS (kk),Rn
      if(a2 && !a1) ram_write16(g, operand(g) << 1, g->r[n]);
      else if(a1) set_reg(g, n, ram_read16(g, operand(g) << 1));
      else set_reg(g, n, (int8_t)operand(g));
      break;
    case 0xb: // FROM Rn, or MOVES Rd,Rn after WITH
      if(g->b) {
        uint16_t v = g->r[n];
        dreg(g, v);
        g->ov = (v & 0x80) != 0;
        sz(g, v);
        break;
      }
      g->sreg = n;
      return;
    case 0xc:
      if(n == 0) { // HIB
        uint16_t r = s >> 8;
        dreg(g, r);
        g->s = (r & 0x80) != 0; g->z = r == 0;
      } else { // OR / XOR, register or #n
        uint16_t b = a2 ? n : g->r[n];
        uint16_t r = a1 ? (s ^ b) : (s | b);
        dreg(g, r); sz(g, r);
      }
      break;
    case 0xd:
      if(n < 15) { // INC Rn
        uint16_t r = g->r[n] + 1;
        set_reg(g, n, r); sz(g, r);
      } else if(a2 && a1) g->rombr = s & 0x7f; // ROMB (buffer keeps the old bank's byte)
      else if(a2) g->rambr = s & 1;          // RAMB
      else set_color(g, g->romBuf);          // GETC
      break;
    case 0xe:
      if(n < 15) { // DEC Rn
        uint16_t r = g->r[n] - 1;
        set_reg(g, n, r); sz(g, r);
      } else if(a1 && a2) dreg(g, (int8_t)g->romBuf);           // GETBS
      else if(a1) dreg(g, (g->romBuf << 8) | (s & 0xff));       // GETBH
      else if(a2) dreg(g, (s & 0xff00) | g->romBuf);            // GETBL
      else dreg(g, g->romBuf);                                  // GETB
      break;
    case 0xf: { // IWT Rn,#xxxx / LM Rn,(xxxx) / SM (xxxx),Rn
      uint16_t v = operand(g);
      v |= operand(g) << 8;
      if(a2 && !a1) ram_write16(g, v, g->r[n]);
      else if(a1) set_reg(g, n, ram_read16(g, v));
      else set_reg(g, n, v);
      break;
    }
  }
  // every non-prefix opcode resets the prefixes
  g->b = g->alt1 = g->alt2 = false;
  g->sreg = g->dreg = 0;
}

static void step(Gsu* g) {
  g_cost = 0;
  uint8_t op = g->pipe;
  g->pipe = fetch(g, R15);
  g->r15Written = false;
  exec(g, op);
  if(!g->r15Written) R15++;
  g->budget -= g_cost * ((g->clsr & 1) ? 1 : 2);
}

static void start(Gsu* g) {
  g->go = true;
  g->pipe = fetch(g, R15);
  R15++;
}

// ---- public ----

Gsu* gsu_init(void* snes) {
  Gsu* g = calloc(1, sizeof(Gsu));
  g->snes = snes;
  return g;
}

void gsu_free(Gsu* g) {
  if(!g) return;
  free(g->ram);
  free(g);
}

void gsu_power(Gsu* g) {
  uint8_t *rom = g->rom, *ram = g->ram;
  uint32_t romSize = g->romSize, ramSize = g->ram_size;
  void* snes = g->snes;
  memset(g, 0, sizeof(Gsu));
  g->snes = snes;
  g->rom = rom; g->romSize = romSize;
  g->ram = ram; g->ram_size = ramSize;
}

void gsu_reset(Gsu* g) { gsu_power(g); }

void gsu_load(Gsu* g, uint8_t* rom, uint32_t romSize, uint8_t* ram, uint32_t ramSize) {
  g->rom = rom; g->romSize = romSize;
  g->ram = ram; g->ram_size = ramSize;
}

bool gsu_ownsRom(Gsu* g) { return g->go && (g->scmr & 0x10); }
bool gsu_ownsRam(Gsu* g) { return g->go && (g->scmr & 0x08); }
bool gsu_irqPending(Gsu* g) { return g->irq && !(g->cfgr & 0x80); }

// "Full I/O Map with Mirrors for GSU2": 3000-303F repeats through 3000-34FF,
// with 3020-302F mirroring 3030-303F
static int io_reg(uint16_t adr) {
  int reg = adr & 0x3f;
  return (reg >= 0x20 && reg < 0x30) ? reg + 0x10 : reg;
}

uint8_t gsu_read(Gsu* g, uint16_t adr) {
  if(adr >= 0x3100 && adr < 0x3300) return gsu_cache_read(g, adr);
  int reg = io_reg(adr);
  if(reg < 0x20) return (reg & 1) ? g->r[reg >> 1] >> 8 : g->r[reg >> 1] & 0xff;
  switch(reg) {
    case 0x30:
      return (g->z << 1) | (g->cy << 2) | (g->s << 3) | (g->ov << 4) | (g->go << 5);
    case 0x31: {
      uint8_t v = g->alt1 | (g->alt2 << 1) | (g->b << 4) | (g->irq << 7);
      g->irq = false; // reading clears IRQ
      update_irq_line(g);
      return v;
    }
    case 0x34: return g->pbr;
    case 0x36: return g->rombr;
    case 0x3b: return 0x04; // VCR: GSU2
    case 0x3c: return g->rambr;
    case 0x3e: return g->cbr & 0xff;
    case 0x3f: return g->cbr >> 8;
  }
  return 0;
}

void gsu_write(Gsu* g, uint16_t adr, uint8_t val) {
  if(adr >= 0x3100 && adr < 0x3300) { gsu_cache_write(g, adr, val); return; }
  int reg = io_reg(adr);
  if(reg < 0x20) { // even: latch, odd: commit LSB=latch, MSB=val
    if(!(reg & 1)) { g->latch = val; return; }
    set_reg(g, reg >> 1, g->latch | (val << 8));
    if(reg == 0x1f) start(g);
    return;
  }
  switch(reg) {
    case 0x30: {
      bool wasGo = g->go;
      g->z = (val >> 1) & 1; g->cy = (val >> 2) & 1; g->s = (val >> 3) & 1;
      g->ov = (val >> 4) & 1; g->go = (val >> 5) & 1;
      if(wasGo && !g->go) { g->cbr = 0; cache_flush(g); } // abort
      if(!wasGo && g->go) { g->go = false; start(g); }
      break;
    }
    case 0x33: g->bramr = val & 1; break;
    case 0x34: g->pbr = val & 0x7f; break;
    case 0x37: g->cfgr = val; update_irq_line(g); break;
    case 0x38: g->scbr = val; break;
    case 0x39: g->clsr = val & 1; break;
    case 0x3a: g->scmr = val; break;
  }
}

// the cache window is addressed by GSU address & 1FF ("Cache Area")
uint8_t gsu_cache_read(Gsu* g, uint16_t adr) {
  return g->cache[(adr - 0x3100) & 0x1ff];
}

void gsu_cache_write(Gsu* g, uint16_t adr, uint8_t val) {
  int i = (adr - 0x3100) & 0x1ff;
  g->cache[i] = val;
  if((i & 0xf) == 0xf) g->cacheValid[i >> 4] = true; // last byte marks the line loaded
}

void gsu_tick(Gsu* g, int masterCycles) {
  if(!g->go) return;
  g->budget += masterCycles;
  while(g->go && g->budget > 0) step(g);
  if(!g->go) g->budget = 0;
}

void gsu_run(Gsu* g) {
  // ponytail: 100M-step cap so a runaway program can't hang the caller
  for(long i = 0; g->go && i < 100000000; i++) step(g);
  g->budget = 0;
}

void gsu_handleState(Gsu* g, StateHandler* sh) {
  sh_handleWordArray(sh, g->r, 16);
  sh_handleBools(sh, &g->z, &g->cy, &g->s, &g->ov, &g->go, &g->alt1, &g->alt2, &g->b, &g->irq, NULL);
  sh_handleBytes(sh, &g->latch, &g->sreg, &g->dreg, &g->pbr, &g->rombr, &g->rambr, &g->scbr, &g->scmr,
    &g->colr, &g->por, &g->bramr, &g->cfgr, &g->clsr, &g->pipe, &g->romBuf, NULL);
  sh_handleWords(sh, &g->cbr, &g->ramAddr, NULL);
  sh_handleByteArray(sh, g->cache, 512);
  for(int i = 0; i < 32; i++) sh_handleBools(sh, &g->cacheValid[i], NULL);
  sh_handleIntsS(sh, &g->budget, NULL);
  sh_handleByteArray(sh, g->ram, g->ram_size);
}
