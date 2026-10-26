/*
 * DSP-1 (NEC uPD77C25 running Nintendo's DSP-1 program) — high-level emulation.
 *
 * (c) 2026 sp00nznet — MIT license
 *
 * Clean-room. Sources: the fullsnes "SNES Cart DSP-n/ST010/ST011" chapters
 * (command list, SR/DR handshake, 80h end marker), the SNESdev wiki DSP-1 page
 * (number formats; Multiply/Inverse/Triangle/Radius/Range/Distance/Rotate/Polar
 * formulas), our own geometry, and black-box observation only: the DR traffic of
 * Super Mario Kart, and input/output comparison against the previous LakeSnes
 * DSP-1 build run as an opaque library (parameter counts, axis order and sign
 * conventions, the zenith limit). No emulator source code was consulted. The
 * chip's data ROM is not included: every value is computed with libm.
 *
 * ponytail: an HLE in double precision with floor rounding, not bit-exact —
 * results land within a unit or two of the previous build (projection ~1%).
 * Only Super Mario Kart has been run against it; the attitude family
 * (01h/03h/0Bh/0Dh/14h), Polar and Target are unchecked against a game that
 * uses them (Pilotwings). Target's Y is the exact inverse of Project and differs
 * from the previous build. Fix those against a game if one misbehaves.
 */
#include <math.h>
#include <string.h>
#include "dsp1.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum { DSP1_IDLE, DSP1_INPUT, DSP1_OUTPUT, DSP1_RASTER };

Dsp1 dsp1;

// Words in / words out per command. Unknown commands are ignored (they return
// to idle, which is what lets games resync by writing 80h bytes).
static int cmdInfo(uint8_t cmd, uint16_t* nIn, uint16_t* nOut) {
  switch(cmd) {
    case 0x00: case 0x20: *nIn = 2; *nOut = 1; return 1; // multiply
    case 0x10:            *nIn = 2; *nOut = 2; return 1; // inverse
    case 0x04:            *nIn = 2; *nOut = 2; return 1; // triangle
    case 0x08:            *nIn = 3; *nOut = 2; return 1; // radius
    case 0x18: case 0x38: *nIn = 4; *nOut = 1; return 1; // range
    case 0x28:            *nIn = 3; *nOut = 1; return 1; // distance
    case 0x0c:            *nIn = 3; *nOut = 2; return 1; // rotate
    case 0x1c:            *nIn = 6; *nOut = 3; return 1; // polar
    case 0x02:            *nIn = 7; *nOut = 4; return 1; // parameter
    case 0x0a:            *nIn = 1; *nOut = 4; return 1; // raster (streams)
    case 0x06:            *nIn = 3; *nOut = 3; return 1; // project
    case 0x0e:            *nIn = 2; *nOut = 2; return 1; // target
    case 0x01: case 0x11: case 0x21: *nIn = 4; *nOut = 0; return 1; // attitude
    case 0x0d: case 0x1d: case 0x2d: *nIn = 3; *nOut = 3; return 1; // objective
    case 0x03: case 0x13: case 0x23: *nIn = 3; *nOut = 3; return 1; // subjective
    case 0x0b: case 0x1b: case 0x2b: *nIn = 3; *nOut = 1; return 1; // scalar
    case 0x14:            *nIn = 6; *nOut = 3; return 1; // gyrate
    case 0x0f: case 0x2f: *nIn = 1; *nOut = 1; return 1; // memory test, ROM version
  }
  return 0; // ponytail: 1Fh (dump data ROM) skipped — we don't ship Nintendo's ROM
}

static double ang(int16_t a) { return a * (M_PI / 32768.0); }

static int16_t sat(double v) {
  v = floor(v);
  if(v != v) return 0;
  if(v > 32767) return 32767;
  if(v < -32768) return -32768;
  return (int16_t) v;
}

static double nz(double v) { return fabs(v) < 1e-9 ? (v < 0 ? -1e-9 : 1e-9) : v; }

// Camera from the last Parameter command. Map space: X right, Y down, Z up.
// Azimuth 0 looks toward -Y; zenith is measured from straight down.
// The eye sits Lfe back from the focus along the requested zenith, but the
// view itself never tilts above ZENITH_MAX (about 80 degrees, found by probing);
// the screen shift that causes is reported to the game as Vof.
#define ZENITH_MAX 14540
typedef struct { double f[3], r[3], u[3], e[3], h[2], les, cz, sz, vof; } Cam;

static void camera(Cam* c) {
  const int16_t* p = dsp1.proj;
  double th = ang(p[5]), ph = ang(p[6]), pv = ang(p[6] > ZENITH_MAX ? ZENITH_MAX : p[6]);
  c->h[0] = sin(th); c->h[1] = -cos(th);
  c->r[0] = cos(th); c->r[1] = sin(th); c->r[2] = 0;
  for(int i = 0; i < 2; i++) c->e[i] = p[i] - p[3] * sin(ph) * c->h[i]; // eye = focus - Lfe*forward
  c->e[2] = p[2] + p[3] * cos(ph);
  c->cz = cos(pv); c->sz = sin(pv);
  c->f[0] = c->sz * c->h[0]; c->f[1] = c->sz * c->h[1]; c->f[2] = -c->cz;
  c->u[0] = c->cz * c->h[0]; c->u[1] = c->cz * c->h[1]; c->u[2] = c->sz;
  c->les = p[4];
  c->vof = p[4] * (cos(pv) / sin(pv) - cos(ph) / nz(sin(ph))); // ponytail: matches the chip to ~1px up to 88 deg, drifts past 90
}

static void rasterLine(void) {
  Cam c; camera(&c);
  double y = dsp1.rasterLine;
  double s = 256.0 * c.e[2] / nz(c.les * c.cz + y * c.sz); // ground units per pixel, 8.8
  double sv = s / nz(c.cz);
  dsp1.out[0] = sat(s * c.r[0]);   // A
  dsp1.out[1] = sat(-sv * c.r[1]); // B
  dsp1.out[2] = sat(s * c.r[1]);   // C
  dsp1.out[3] = sat(sv * c.r[0]);  // D
}

// Rotate v about axis (0=X 1=Y 2=Z) by angle a. The sense of each axis was
// picked to match Polar/Objective/Subjective results seen from outside.
static void rot(double* v, int axis, int16_t a) {
  double c = cos(ang(a)), s = sin(ang(a));
  int i = axis == 0 ? 1 : 0, j = axis == 2 ? 1 : 2; // the two axes that move
  double p = v[i], q = v[j];
  if(axis == 1) { v[i] = p * c - q * s; v[j] = p * s + q * c; }
  else          { v[i] = p * c + q * s; v[j] = -p * s + q * c; }
}

static void execute(void) {
  int16_t* in = dsp1.in;
  int16_t* out = dsp1.out;
  switch(dsp1.cmd) {
    case 0x00: case 0x20: out[0] = sat((in[0] * in[1]) / 32768.0); break;
    case 0x10: { // 1 / (m * 2^e) as a normalised mantissa/exponent pair
      double v = in[0] / 32768.0 * ldexp(1.0, in[1]);
      if(in[0] == 0) { out[0] = 0x7fff; out[1] = 0x2f; break; }
      int e; double m = frexp(1.0 / v, &e); // m in [0.5,1)
      out[0] = sat(m * 32768.0); out[1] = (int16_t) e;
      break;
    }
    case 0x04: {
      double a = ang(in[0]);
      out[0] = sat(in[1] * sin(a)); out[1] = sat(in[1] * cos(a));
      break;
    }
    case 0x08: { // D2 format: LSB = 1/2, so the 32-bit result is twice the sum
      uint32_t s = 2u * ((uint32_t) (in[0] * in[0]) + (uint32_t) (in[1] * in[1]) + (uint32_t) (in[2] * in[2])); // wraps
      out[0] = (int16_t) (s & 0xffff); out[1] = (int16_t) (s >> 16);
      break;
    }
    case 0x18: case 0x38: {
      int64_t s = (int64_t) in[0] * in[0] + (int64_t) in[1] * in[1] + (int64_t) in[2] * in[2] - (int64_t) in[3] * in[3];
      out[0] = (int16_t) (s >> 15); // wraps on overflow
      break;
    }
    case 0x28:
      out[0] = sat(sqrt((double) in[0] * in[0] + (double) in[1] * in[1] + (double) in[2] * in[2]));
      break;
    case 0x0c: {
      double c = cos(ang(in[0])), s = sin(ang(in[0]));
      out[0] = sat(in[1] * c + in[2] * s); out[1] = sat(-in[1] * s + in[2] * c);
      break;
    }
    case 0x1c: { // rotate about Z by I1, then Y by I2, then X by I3
      double v[3] = { in[3], in[4], in[5] };
      rot(v, 2, in[0]); rot(v, 1, in[1]); rot(v, 0, in[2]);
      out[0] = sat(v[0]); out[1] = sat(v[1]); out[2] = sat(v[2]);
      break;
    }
    case 0x02: {
      memcpy(dsp1.proj, in, sizeof(dsp1.proj));
      Cam c; camera(&c);
      double reach = c.e[2] * c.sz / nz(c.cz); // where the centre ray meets the ground
      out[0] = sat(c.vof);
      out[1] = sat(-c.les * c.cz / nz(c.sz)); // Vva: horizon line relative to screen centre
      out[2] = sat(c.e[0] + reach * c.h[0]);
      out[3] = sat(c.e[1] + reach * c.h[1]);
      break;
    }
    case 0x0a: dsp1.rasterLine = in[0]; rasterLine(); break;
    case 0x06: {
      Cam c; camera(&c);
      double q[3], d = 0, x = 0, y = 0;
      for(int i = 0; i < 3; i++) q[i] = in[i] - c.e[i];
      for(int i = 0; i < 3; i++) { d += q[i] * c.f[i]; x += q[i] * c.r[i]; y += q[i] * c.u[i]; }
      d = nz(d);
      out[0] = sat(c.les * x / d); out[1] = sat(-c.les * y / d + c.vof); out[2] = sat(256.0 * c.les / d);
      break;
    }
    case 0x0e: { // screen point -> ground point (inverse of Project, on Z = 0)
      Cam c; camera(&c);
      double dir[3];
      for(int i = 0; i < 3; i++) dir[i] = c.les * c.f[i] + in[0] * c.r[i] - in[1] * c.u[i];
      double t = -c.e[2] / nz(dir[2]);
      out[0] = sat(c.e[0] + t * dir[0]); out[1] = sat(c.e[1] + t * dir[1]);
      break;
    }
    case 0x01: case 0x11: case 0x21: memcpy(dsp1.att[dsp1.cmd >> 4], in, sizeof(dsp1.att[0])); break;
    case 0x0d: case 0x1d: case 0x2d:   // global -> object
    case 0x0b: case 0x1b: case 0x2b:   // forward component of that (one output word)
    case 0x03: case 0x13: case 0x23: { // object -> global
      const int16_t* a = dsp1.att[dsp1.cmd >> 4]; // scale, Az, Ay, Ax
      double v[3] = { in[0], in[1], in[2] };
      if((dsp1.cmd & 0x0f) == 0x03) { rot(v, 0, a[3]); rot(v, 1, a[2]); rot(v, 2, a[1]); }
      else { rot(v, 2, -a[1]); rot(v, 1, -a[2]); rot(v, 0, -a[3]); }
      for(int i = 0; i < 3; i++) out[i] = sat(v[i] * a[0] / 65536.0); // scale is applied halved
      break;
    }
    case 0x14: { // gyrate: angles (Az, Ax, Ay) advanced by body rates (U, F, L)
      double cx = cos(ang(in[1])), tx = tan(ang(in[1])), cy = cos(ang(in[2])), sy = sin(ang(in[2]));
      double u = in[3], f = in[4], l = in[5];
      out[0] = (int16_t) (in[0] + (int) floor((u * cy - f * sy) / (fabs(cx) < 1e-6 ? 1e-6 : cx)));
      out[1] = (int16_t) (in[1] + (int) floor(u * sy + f * cy));
      out[2] = (int16_t) (in[2] + (int) floor(l - tx * (u * cy + f * sy)));
      break;
    }
    case 0x0f: out[0] = 0; break; // memory test: pass
    case 0x2f: out[0] = 0x0101; break; // DSP1B
  }
}

static void command(uint8_t val) {
  dsp1.cmd = val;
  dsp1.pos = 0;
  dsp1.state = cmdInfo(val, &dsp1.nIn, &dsp1.nOut) ? DSP1_INPUT : DSP1_IDLE;
}

// One DR byte transferred out (by a read, or by a write during output — the
// chip doesn't care which, per fullsnes).
static uint8_t outByte(void) {
  uint16_t w = (uint16_t) dsp1.out[dsp1.pos >> 1];
  uint8_t b = (dsp1.pos & 1) ? (w >> 8) : (w & 0xff);
  if(++dsp1.pos >= dsp1.nOut * 2) {
    dsp1.pos = 0;
    if(dsp1.state == DSP1_RASTER) { dsp1.rasterLine++; rasterLine(); }
    else dsp1.state = DSP1_IDLE;
  }
  return b;
}

void dsp1_init(void) {
  memset(&dsp1, 0, sizeof(dsp1));
}

void dsp1_reset(void) {
  uint16_t boundary = dsp1.boundary;
  dsp1_init();
  dsp1.boundary = boundary;
}

uint8_t dsp1_read(uint16_t adr) {
  if(adr >= dsp1.boundary) return 0x80; // SR high byte: RQM=1, always ready (ponytail: no busy timing)
  if(dsp1.state == DSP1_OUTPUT || dsp1.state == DSP1_RASTER) return outByte();
  return 0x80;
}

void dsp1_write(uint16_t adr, uint8_t val) {
  if(adr >= dsp1.boundary) return; // SR is read-only
  switch(dsp1.state) {
    case DSP1_IDLE: case DSP1_RASTER: command(val); break; // a write ends the raster stream
    case DSP1_OUTPUT: outByte(); break;
    case DSP1_INPUT: {
      uint16_t* w = (uint16_t*) &dsp1.in[dsp1.pos >> 1];
      if(dsp1.pos & 1) *w = (*w & 0x00ff) | (val << 8); else *w = val;
      if(++dsp1.pos >= dsp1.nIn * 2) {
        dsp1.pos = 0;
        execute();
        dsp1.state = dsp1.cmd == 0x0a ? DSP1_RASTER : dsp1.nOut ? DSP1_OUTPUT : DSP1_IDLE;
      }
      break;
    }
  }
}

void dsp1_handleState(StateHandler* sh) {
  sh_handleByteArray(sh, (uint8_t*) &dsp1, sizeof(dsp1)); // plain integer struct
}
