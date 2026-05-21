# PID Controller

A single-function discrete PID controller suitable for embedded motor-control applications running at a fixed sample rate.

## File

| File | Description |
|------|-------------|
| `pid.h` | Function declaration with default parameter and documentation |
| `pid.cpp` | `calcPID(sp, y, reset)` — self-contained PID implementation |

## Features

- **Derivative on measurement** — differentiates the process variable `y` rather than the error, so the output stays smooth when the setpoint steps suddenly (no *derivative kick*).
- **Bilinear (Tustin) integrator** — more accurate than a simple forward-Euler integrator at higher gains.
- **First-order low-pass filter on D** — configurable cutoff `Wd` (rad/s) to trade off D sharpness against noise sensitivity.
- **Clamping anti-windup** — when the output is saturated and the error would push it further into saturation, the integrator update is discarded for that sample.

## Tunable parameters

Edit the `static const` values at the top of `calcPID`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `Kp` | `1.0` | Proportional gain |
| `Ki` | `0.5` | Integral gain |
| `Kd` | `0.02` | Derivative gain |
| `Wd` | `80.0` rad/s | D low-pass filter cutoff (~12.7 Hz) |
| `Ts` | `0.001` s | Sample period (must match your control-loop rate) |
| `uMin` / `uMax` | `-1.0` / `1.0` | Output clamp limits |

## Usage

```cpp
// Call once at start-up (or after an emergency stop) to initialise states
// from the current measurement and prevent a transient on first update.
calcPID(setpoint, measurement, /*reset=*/true);

// Call every Ts seconds inside your control loop
float u = calcPID(setpoint, measurement);
applyOutput(u);
```

> **Note:** The function uses `static` local variables for its state, so there is one implicit controller instance per translation unit. If you need multiple independent PID controllers in the same project, wrap the function in a struct/class or use a template parameter to distinguish instances.

## Math background

### Control law

```
u[n] = Kp·e[n]  +  I[n]  −  D[n]
```

where `e[n] = sp − y[n]`.

### Bilinear integrator

```
I[n] = I[n−1] + Ki·(Ts/2)·(e[n] + e[n−1])
```

### Filtered derivative (on measurement)

Continuous transfer function from measurement to D term:

```
D(s) = Kd · Wd·s / (s + Wd)
```

Discretised with the bilinear transform `s = (2/Ts)·(1−z⁻¹)/(1+z⁻¹)`:

```
d[n] = dA·d[n−1] + dB·(y[n] − y[n−1])

dA = (2 − Wd·Ts) / (2 + Wd·Ts)
dB = (2·Kd·Wd)  / (2 + Wd·Ts)
```
