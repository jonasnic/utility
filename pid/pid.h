#pragma once

/**
 * calcPID - Discrete PID controller.
 *
 * Parameters
 * ----------
 *   sp    : setpoint
 *   y     : process measurement
 *   reset : when true, initialise states from current values and return 0.
 *           Call this once before entering the control loop, or after an
 *           emergency stop, to prevent a transient on the first update.
 *
 * Returns the control output u, clamped to [uMin, uMax].
 *
 * Thread safety
 * -------------
 * The function uses static local variables for its state.  It is NOT
 * thread-safe and should be called from a single execution context
 * (e.g. one interrupt / task).  For multiple independent controllers or
 * concurrent access, wrap the state in a struct and pass it as a parameter.
 */
float calcPID(float sp, float y, bool reset = false);
