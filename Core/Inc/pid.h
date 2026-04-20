/*
 * pid.h
 * Self-balancing PID controller for STM32L4R5ZI + BNO055 + L298 H-Bridge
 *
 * Design notes:
 *  - Uses dynamic gains that change based on tilt angle (from Arduino reference)
 *  - Derivative term fed directly from BNO055 gyro (cleaner than finite difference)
 *  - PWM range matches TIM2 Period = 999  (0 = off, 999 = full speed)
 *  - Safety cutoff stops motors if robot falls past recovery angle
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include <stdbool.h>

/* ── PWM limits (TIM2 Period = 999) ──────────────────────────────────────────
 * These are the ABSOLUTE hardware limits — dynamic gains adjust within them.
 * PWM_DEADBAND: minimum duty to overcome motor static friction.
 * PWM_MAX:      maximum duty (keep below 999 to leave headroom).
 * -------------------------------------------------------------------------- */
#define PWM_MAX              750   // 80% duty — leave headroom for bursts
#define PWM_MIN_DEADBAND     150   // below this motors don't move, skip it

/* ── Safety cutoff ───────────────────────────────────────────────────────────
 * If the robot tilts past this angle it cannot self-recover.
 * Motors stop and integral resets to prevent windup on restart.
 * -------------------------------------------------------------------------- */
#define BALANCE_CUTOFF_DEG   65.0f  // stop trying to balance past 45°

/* ── Dead zone ───────────────────────────────────────────────────────────────
 * If pitch is within this band, brake instead of running motors.
 * Prevents buzzing/jitter when nearly balanced.
 * -------------------------------------------------------------------------- */
#define BALANCE_DEADZONE_DEG  2.30f  // ± degrees around setpoint = brake

/* ── Setpoint ramp rate ──────────────────────────────────────────────────────
 * Maximum degrees-per-second at which pid.setpoint is allowed to move
 * toward its target.  This prevents sudden PID error spikes when tracking
 * starts/stops, which would cause integral windup and overshoot.
 *
 * Rule of thumb:
 *   DRIVE_LEAN_DEG / SETPOINT_RAMP_RATE_DEG_S = ramp time in seconds.
 *   At 2.5 deg lean and 5.0 deg/s ramp → 0.5 s to reach full chase lean.
 *   Increase if the robot feels sluggish to start moving toward a target.
 *   Decrease if it still overshoots when tracking begins.
 * -------------------------------------------------------------------------- */
#define SETPOINT_RAMP_RATE_DEG_S  5.0f

/* ── dt guard ────────────────────────────────────────────────────────────────
 * Clamp dt to this range before passing into the PID.  When the Pixy2 SPI
 * poll or a printf stalls the loop, dt can spike to 20–30 ms; clamping it
 * to DT_MAX_S prevents the integral and derivative from seeing that glitch.
 * -------------------------------------------------------------------------- */
#define DT_MIN_S   0.002f   //  2 ms — guard against timer wrap / first call
#define DT_MAX_S   0.020f   // 20 ms — clamp Pixy/printf stall spikes

/* ── Steering low-pass filter ────────────────────────────────────────────────
 * Exponential smoothing coefficient for steer_output.
 *   steer = steer * (1 - alpha) + new_target * alpha
 * Lower alpha = more smoothing (slower response).
 * Higher alpha = faster response (less smoothing).
 * 0.25 → ~4-cycle time constant at 50 Hz Pixy poll rate.
 * -------------------------------------------------------------------------- */
#define STEER_LP_ALPHA  0.25f

/* ── PID controller struct ───────────────────────────────────────────────── */
typedef struct {

    /* Tunable gains — adjusted dynamically based on angle zone */
    float Kp;
    float Ki;
    float Kd;

    /* Internal state */
    float integral;
    float prev_error;

    /* Integral windup limit — also set dynamically */
    float i_limit;

    /* Output of last update — signed, maps to motor effort */
    float output;

    /* Setpoint: desired pitch angle (degrees). The main loop ramps this
     * toward setpoint_target at SETPOINT_RAMP_RATE_DEG_S to avoid sudden
     * error spikes when the Pixy2 tracking starts or stops.             */
    float setpoint;

    /* Target the main loop is ramping setpoint toward.
     * Write to this via PID_SetTarget(); never write setpoint directly
     * from the Pixy2 tracking code.                                     */
    float setpoint_target;

} PIDController;

/* ── Function declarations ───────────────────────────────────────────────── */

/**
 * @brief  Initialise all controller state to zero / defaults.
 *         Call once before the loop.
 */
void PID_Init(PIDController *pid);

/**
 * @brief  Run one PID iteration.
 *
 * @param  pid        Pointer to controller struct
 * @param  pitch      Current pitch angle from BNO055 Euler.Z (degrees)
 * @param  pitch_rate Current pitch rate from BNO055 Gyro.X  (deg/s)
 *                    Used directly as derivative — much cleaner than
 *                    computing it from angle differences.
 * @param  dt         Time since last call in seconds.
 *                    Should be pre-clamped to [DT_MIN_S, DT_MAX_S].
 *
 * @retval true  if output is valid and motors should be driven
 *         false if robot has fallen past cutoff — caller should brake
 */
bool PID_Update(PIDController *pid, float pitch, float pitch_rate, float dt);

/**
 * @brief  Set the target setpoint that the main loop will ramp toward.
 *
 *         When the new target differs from the current setpoint by more than
 *         1°, the integral is halved to reduce windup carried over from the
 *         previous operating point.  Always use this instead of writing
 *         pid->setpoint directly from tracking code.
 *
 * @param  pid     Pointer to controller struct
 * @param  target  Desired pitch angle in degrees
 */
void PID_SetTarget(PIDController *pid, float target);

/**
 * @brief  Ramp pid->setpoint one step toward pid->setpoint_target.
 *
 *         Call once per PID cycle BEFORE PID_Update().  dt is the same
 *         delta-time used for the PID so the ramp speed stays consistent
 *         regardless of loop timing variations.
 *
 * @param  pid  Pointer to controller struct
 * @param  dt   Seconds since last call (pre-clamped)
 */
void PID_RampSetpoint(PIDController *pid, float dt);

/**
 * @brief  Hard reset — zero integral, prev_error, and output.
 *         Call after a fall or when motors are stopped.
 */
void PID_Reset(PIDController *pid);

/**
 * @brief  Soft reset — halve the integral only.
 *         Called automatically by PID_SetTarget when setpoint changes
 *         significantly; can also be called manually after a tracking
 *         transition to reduce (but not eliminate) accumulated wind-up.
 */
void PID_SoftReset(PIDController *pid);

#endif /* INC_PID_H_ */
