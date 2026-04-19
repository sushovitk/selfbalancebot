/*
 * pid.c
 * Self-balancing PID controller for STM32L4R5ZI + BNO055 + L298 H-Bridge
 *
 * Key changes vs original:
 *  - PID_SetTarget()   : sets setpoint_target + soft-resets integral on big
 *                        setpoint changes to stop windup carrying over.
 *  - PID_RampSetpoint(): advances setpoint toward setpoint_target at a
 *                        controlled rate (SETPOINT_RAMP_RATE_DEG_S) so the
 *                        Pixy2 lean command never creates a sudden error spike.
 *  - PID_SoftReset()   : halves integral instead of zeroing it, reducing
 *                        windup while keeping some history when transitioning.
 *  - PID_Update()      : dt is now expected to be pre-clamped by the caller
 *                        (DT_MIN_S … DT_MAX_S) so SPI/printf stalls don't
 *                        cause rogue I/D kicks.
 */

#include "pid.h"
#include <math.h>

/* ── Init ─────────────────────────────────────────────────────────────────── */
void PID_Init(PIDController *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->output          = 0.0f;
    pid->setpoint        = 0.0f;
    pid->setpoint_target = 0.0f;

    pid->Kp      = 20.5f;
    pid->Ki      = 0.02f;
    pid->Kd      = 1.50f;
    pid->i_limit = 100.0f;
}

/* ── Hard Reset (call after fall or brake) ─────────────────────────────────── */
void PID_Reset(PIDController *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/* ── Soft Reset (call after large setpoint change) ───────────────────────── */
/*
 * Halving the integral (rather than zeroing it) keeps some "memory" of
 * the motor effort needed to balance at the current angle while removing
 * the windup that accumulated at the old setpoint.  This gives a smoother
 * transition than a hard reset and avoids the sudden lurch in the opposite
 * direction that a zero-reset can cause.
 */
void PID_SoftReset(PIDController *pid)
{
    pid->integral *= 0.5f;
}

/* ── Set Target Setpoint ─────────────────────────────────────────────────── */
/*
 * Always use this instead of writing pid->setpoint directly from the Pixy2
 * tracking code.  It records the desired angle in setpoint_target so the
 * main loop can ramp setpoint smoothly, and calls PID_SoftReset() when the
 * change is large enough to cause significant windup.
 */
void PID_SetTarget(PIDController *pid, float target)
{
    /* Only soft-reset the integral when the target changes noticeably.     */
    /* A 1° threshold avoids triggering on floating-point noise.            */
    if (fabsf(target - pid->setpoint_target) > 1.0f)
    {
        PID_SoftReset(pid);
    }
    pid->setpoint_target = target;
}

/* ── Ramp Setpoint ───────────────────────────────────────────────────────── */
/*
 * Advance pid->setpoint one dt-step toward pid->setpoint_target at a rate
 * of SETPOINT_RAMP_RATE_DEG_S degrees per second.
 *
 * Call this BEFORE PID_Update() each cycle.
 *
 * Why this matters:
 *   Without ramping, switching from balance (3.31°) to chase (5.81°) creates
 *   a 2.5° step error.  With Kp=65, that is an instant 162-count PWM spike —
 *   enough to lurch the robot and start filling the integral the wrong way.
 *   At 5 deg/s, the same transition takes 0.5 s and the PID error never
 *   exceeds ~0.05° per cycle at 100 Hz, which is well within the proportional
 *   band the gains were tuned for.
 */
void PID_RampSetpoint(PIDController *pid, float dt)
{
    float step = SETPOINT_RAMP_RATE_DEG_S * dt;
    float diff = pid->setpoint_target - pid->setpoint;

    if (diff > step)
        pid->setpoint += step;
    else if (diff < -step)
        pid->setpoint -= step;
    else
        pid->setpoint = pid->setpoint_target;  /* snap when close */
}

/* ── Update ───────────────────────────────────────────────────────────────── */
bool PID_Update(PIDController *pid, float pitch, float pitch_rate, float dt)
{
    /*
     * dt is clamped by the caller (main.c) to [DT_MIN_S, DT_MAX_S].
     * The guard below is a last-resort safety net only.
     */
    if (dt < DT_MIN_S) dt = DT_MIN_S;
    if (dt > DT_MAX_S) dt = DT_MAX_S;

    float abs_pitch = fabsf(pitch - pid->setpoint);

    /* ── Safety cutoff ──────────────────────────────────────────────────── */
    if (abs_pitch > BALANCE_CUTOFF_DEG)
    {
        PID_Reset(pid);
        return false;
    }

    /* ── Dead zone brake ────────────────────────────────────────────────── */
    /*
     * Only stop inside the deadzone when the setpoint is NOT actively being
     * ramped toward a different target.  While chasing, a small deadzone
     * error should still be corrected; the lean provides the drive force.
     */
//    if (abs_pitch < BALANCE_DEADZONE_DEG &&
//        fabsf(pid->setpoint - pid->setpoint_target) < BALANCE_DEADZONE_DEG)
//    {
//        pid->output = 0.0f;
//        return true;
//    }

    /* ── PID computation ─────────────────────────────────────────────────── */

    float error = pid->setpoint - pitch;

    /* Proportional */
    float P = pid->Kp * error;

    /* Integral with anti-windup clamping */
    pid->integral += error * dt;

    if      (pid->integral >  pid->i_limit) pid->integral =  pid->i_limit;
    else if (pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;

    float I = pid->Ki * pid->integral;

    /*
     * Derivative — use gyro rate directly, NOT (error-lastError)/dt.
     * Gyro gives smooth angular velocity; finite difference on angle
     * amplifies sensor noise.  Negate because pitch_rate > 0 means
     * falling in the positive direction — we want to resist that.
     */
    float D = -pid->Kd * pitch_rate;

    pid->output     = P + I + D;
    pid->prev_error = error;

    return true;
}
