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
#define PWM_MAX              800   // 80% duty — leave headroom for bursts
#define PWM_MIN_DEADBAND     372    // below this motors don't move, skip it

/* ── Safety cutoff ───────────────────────────────────────────────────────────
 * If the robot tilts past this angle it cannot self-recover.
 * Motors stop and integral resets to prevent windup on restart.
 * -------------------------------------------------------------------------- */
#define BALANCE_CUTOFF_DEG   45.0f  // stop trying to balance past 45°

/* ── Dead zone ───────────────────────────────────────────────────────────────
 * If pitch is within this band, brake instead of running motors.
 * Prevents buzzing/jitter when nearly balanced.
 * -------------------------------------------------------------------------- */
#define BALANCE_DEADZONE_DEG  0.05f  // ± degrees around setpoint = brake

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

    /* Setpoint: desired pitch angle in degrees (0 = perfectly upright) */
    float setpoint;

} PIDController;

/* ── Function declarations ───────────────────────────────────────────────── */

/**
 * @brief  Reset all controller state to zero. Call once before the loop.
 */
void PID_Init(PIDController *pid);

/**
 * @brief  Run one PID iteration.
 *
 * @param  pid        Pointer to controller struct
 * @param  pitch      Current pitch angle from BNO055 Euler.Y (degrees)
 * @param  pitch_rate Current pitch rate from BNO055 Gyro.Y (deg/s)
 *                    Used directly as derivative — much cleaner than
 *                    computing it from angle differences.
 * @param  dt         Time since last call in seconds (use HAL_GetTick delta)
 *
 * @retval true  if output is valid and motors should be driven
 *         false if robot has fallen past cutoff — caller should brake
 */
bool PID_Update(PIDController *pid, float pitch, float pitch_rate, float dt);

/**
 * @brief  Reset integral and previous error. Call when motors are stopped
 *         or after a fall to prevent windup.
 */
void PID_Reset(PIDController *pid);

#endif /* INC_PID_H_ */