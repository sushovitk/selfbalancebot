/*
 * pid.c
 * Self-balancing PID controller for STM32L4R5ZI + BNO055 + L298 H-Bridge
 *
 */

#include "pid.h"
#include <math.h>

/* ── Init ─────────────────────────────────────────────────────────────────── */
void PID_Init(PIDController *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;

    /* Default gains — will be overwritten dynamically in PID_Update */
    pid->Kp      = 16.0f;
    pid->Ki      = 0.05f;
    pid->Kd      = 0.3f;
    pid->i_limit = 100.0f;
    pid->setpoint = 0.0f;    // upright = 0°. Trim this if robot leans at rest.
}

/* ── Reset (call after fall or brake) ─────────────────────────────────────── */
void PID_Reset(PIDController *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/* ── Update ───────────────────────────────────────────────────────────────── */
bool PID_Update(PIDController *pid, float pitch, float pitch_rate, float dt)
{
    /* Guard against bad dt (first call, timer wrap, etc.) */
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    float abs_pitch = fabsf(pitch - pid->setpoint);

    /* ── Safety cutoff ──────────────────────────────────────────────────── */
    /* Robot has fallen too far — cannot recover. Stop everything.          */
    if (abs_pitch > BALANCE_CUTOFF_DEG) {
        PID_Reset(pid);
        return false;   /* caller should brake motors */
    }

    /* ── Dead zone brake ────────────────────────────────────────────────── */
    /* Very close to upright — just brake, no buzzing from tiny corrections */
    if (abs_pitch < BALANCE_DEADZONE_DEG) {
        pid->output = 0.0f;
        return true;    /* output = 0 signals caller to brake */
    }

    /* ── Dynamic gain scheduling ────────────────────────────────────────── */
    /* Gains and limits change based on how far the robot is tilted.        */
    /* Larger tilt → more aggressive correction, tighter integral limit.    */
    /* (Adapted from Arduino BNO055 reference)                              */
    if (abs_pitch > 20.0f) {
        /* Far tilt — aggressive */
        pid->Kp      = 20.0f;
        pid->Ki      = 0.05f;
        pid->Kd      = 0.6f;
        pid->i_limit = 50.0f;
    }
    else if (abs_pitch > 8.0f) {
        /* Medium tilt — moderate */
        pid->Kp      = 16.0f;
        pid->Ki      = 0.05f;
        pid->Kd      = 0.3f;
        pid->i_limit = 100.0f;
    }
    else {
        /* Near upright — gentle, more integral authority */
        pid->Kp      = 12.0f;
        pid->Ki      = 0.05f;
        pid->Kd      = 0.2f;
        pid->i_limit = 175.0f;
    }

    /* ── Extra derivative damping when rotating fast ────────────────────── */
    /* If the robot is spinning quickly toward a fall, damp harder.         */
    if (fabsf(pitch_rate) > 50.0f) {
        pid->Kd += 0.2f;
    }

    /* ── PID computation ─────────────────────────────────────────────────── */

    /* Signed error — MUST keep sign so robot knows which way to correct    */
    float error = pid->setpoint - pitch;

    /* Proportional */
    float P = pid->Kp * error;

    /* Integral with anti-windup clamping */
    pid->integral += error * dt;
    
    if(pid->integral >  pid->i_limit) 
    {
        pid->integral =  pid->i_limit;
    } 
    else if(pid->integral < -pid->i_limit)
    { 
        pid->integral = -pid->i_limit;
    }

    float I = pid->Ki * pid->integral;

    /* Derivative — use gyro rate directly, NOT (error-lastError)/dt.
     * Gyro gives smooth angular velocity; finite difference on angle
     * amplifies sensor noise. Negate because pitch_rate > 0 means
     * falling in the positive direction — we want to resist that.         */
    float D = -pid->Kd * pitch_rate;

    /* Total output — SIGNED: positive = drive forward, negative = backward */
    pid->output     = P + I + D;
    pid->prev_error = error;

    return true;
}