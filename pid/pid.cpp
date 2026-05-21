#include "pid.h"

// calcPID - see pid.h for full documentation.
float calcPID(float sp, float y, bool reset)
{
  // Tunables
  constexpr float Kp = 1.0f;
  constexpr float Ki = 0.5f;
  constexpr float Kd = 0.02f;

  // D low-pass cutoff in rad/s.
  // Larger Wd = sharper/noisier D.
  // Smaller Wd = smoother/slower D.
  constexpr float Wd = 80.0f;

  // Fixed sample time in seconds
  constexpr float Ts = 0.001f;   // 1 kHz

  // Output clamp
  constexpr float uMin = -1.0f;
  constexpr float uMax =  1.0f;

  // States
  static float e1 = 0.0f;
  static float y1 = 0.0f;
  static float i1 = 0.0f;
  static float d1 = 0.0f;

  // Bilinear derivative filter coefficients:
  //
  // Continuous D feedback path:
  //
  //   D(s) = Kd * Wd*s / (s + Wd)
  //
  // Bilinear:
  //
  //   s = (2/Ts) * (1 - z^-1) / (1 + z^-1)
  //
  // Difference equation:
  //
  //   d[n] = dA*d[n-1] + dB*(y[n] - y[n-1])
  //
  constexpr float dA = (2.0f - Wd * Ts) / (2.0f + Wd * Ts);
  constexpr float dB = (2.0f * Kd * Wd) / (2.0f + Wd * Ts);

  if (reset) {
    e1 = sp - y;
    y1 = y;
    i1 = 0.0f;
    d1 = 0.0f;
    return 0.0f;
  }

  float e = sp - y;

  // P term on error
  float p = Kp * e;

  // I term on error, bilinear integrator
  //
  //   I[n] = I[n-1] + Ki*Ts/2 * (e[n] + e[n-1])
  //
  float i = i1 + Ki * Ts * 0.5f * (e + e1);

  // D term in feedback path.
  //
  // Important:
  //   This differentiates measurement y, NOT error.
  //
  // Output is:
  //
  //   u = P(error) + I(error) - D(measurement)
  //
  // This avoids derivative kick when setpoint changes suddenly.
  float d = dA * d1 + dB * (y - y1);

  float u = p + i - d;

  // Basic anti-windup:
  // If output saturates and error would push farther into saturation,
  // reject the new integrator value.
  if (u > uMax) {
    u = uMax;
    if (e > 0.0f) i = i1;
  } else if (u < uMin) {
    u = uMin;
    if (e < 0.0f) i = i1;
  }

  e1 = e;
  y1 = y;
  i1 = i;
  d1 = d;

  return u;
}
