#pragma once

#define UTILS_EPS 0.001

typedef struct {
  float r = 0.0;  // [0.0, 1.0]
  float g = 0.0;  // [0.0, 1.0]
  float b = 0.0;  // [0.0, 1.0]
  float a = 0.0;
} RGB;

typedef struct {
  float h = 0.0;  // angle in degrees
  float s = 1.0;  // a fraction between 0 and 1
  float v = 1.0;  // a fraction between 0 and 1
} HSV;

static HSV rgb2hsv(RGB in);
static RGB hsv2rgb(HSV in);

[[maybe_unused]] HSV rgb2hsv(RGB in) {
  HSV out;
  float min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min < in.b ? min : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max > in.b ? max : in.b;

  out.v = max;  // v
  delta = max - min;
  if (delta < UTILS_EPS) {
    out.s = 0;
    out.h = 0;  // undefined, maybe nan?
    return out;
  }
  if (max > 0.0) {          // NOTE: if Max is == 0, this divide would cause a crash
    out.s = (delta / max);  // s
  } else {
    // if max is 0, then r = g = b = 0
    // s = 0, h is undefined
    out.s = 0.0;
    out.h = NAN;  // its now undefined
    return out;
  }
  if (in.r >= max)                  // > is bogus, just keeps compilor happy
    out.h = (in.g - in.b) / delta;  // between yellow & magenta
  else if (in.g >= max)
    out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
  else
    out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

  out.h *= 60.0;  // degrees

  if (out.h < 0.0)
    out.h += 360.0;

  return out;
}

// https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
[[maybe_unused]] RGB hsv2rgb(HSV in) {
  float hh, p, q, t, ff;
  long i;
  RGB out;

  if (in.s <= 0.0) {  // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }
  hh = in.h;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i) {
    case 0:
      out.r = in.v;
      out.g = t;
      out.b = p;
      break;
    case 1:
      out.r = q;
      out.g = in.v;
      out.b = p;
      break;
    case 2:
      out.r = p;
      out.g = in.v;
      out.b = t;
      break;

    case 3:
      out.r = p;
      out.g = q;
      out.b = in.v;
      break;
    case 4:
      out.r = t;
      out.g = p;
      out.b = in.v;
      break;
    case 5:
    default:
      out.r = in.v;
      out.g = p;
      out.b = q;
      break;
  }
  return out;
}

// https://stackoverflow.com/questions/726549/algorithm-for-additive-color-mixing-for-rgb-values
[[maybe_unused]] RGB rgbAdd(RGB fg, RGB bg) {
  RGB r;                              // result;
  r.a = 1.0 - (1.0 - fg.a) * (1.0 - bg.a);  // 0.75
  if (r.a < UTILS_EPS) // avoid division by zero
    return RGB{};

  r.r = fg.r * fg.a / r.a + bg.r * bg.a * (1.0 - fg.a) / r.a;  // 0.67
  r.g = fg.g * fg.a / r.a + bg.g * bg.a * (1.0 - fg.a) / r.a;  // 0.33
  r.b = fg.b * fg.a / r.a + bg.b * bg.a * (1.0 - fg.a) / r.a;  // 0.00
  return r;
}