#include "pid_controller.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

int main(void)
{
    printf("=== Discrete PID Controller & Anti-Windup Verification ===\n");

    pid_controller_t pid;
    const float dt = 0.001f; /* 1 kHz sample rate */
    pid_init(&pid, 20.0f, 2.0f, 0.5f, dt, -1.0f, 1.0f);

    /* 1. Verify anti-windup clamping under sustained saturation */
    printf("[1] Testing Anti-Windup clamping under sustained step error...\n");
    for (int i = 0; i < 5000; i++)
    {
        pid_update(&pid, 100.0f, 0.0f); /* Massive error injection */
    }
    assert(pid.integrator <= pid.integrator_max);
    assert(pid.integrator >= pid.integrator_min);
    printf("    Integrator strictly clamped at: %.2f (Limits: [%.2f, %.2f])\n",
           pid.integrator, pid.integrator_min, pid.integrator_max);

    /* 2. Closed-loop step response simulation on 2nd-order mechanical plant */
    pid_reset(&pid);
    /* Tuned gains: Kp = 20.0, Ki = 3.0, Kd = 1.2 for critical damping */
    pid_init(&pid, 20.0f, 3.0f, 1.2f, dt, -1.0f, 1.0f);

    float pos = 0.0f;
    float vel = 0.0f;
    const float target = 10.0f;

    printf("[2] Running closed-loop step response to target = %.1f rad (3000 cycles / 3.0 s)...\n", target);
    for (int ms = 0; ms < 3000; ms++)
    {
        float effort = pid_update(&pid, target, pos);
        /* Plant dynamics: acceleration = 25.0 * effort - 2.0 * vel */
        float accel = (25.0f * effort) - (2.0f * vel);
        vel += accel * dt;
        pos += vel * dt;
    }

    printf("    Final Position: %.3f rad | Settling Error: %.4f rad\n", pos, fabsf(target - pos));
    assert(fabsf(target - pos) < 0.03f);

    printf("\n>> SUCCESS: Module 4 Discrete PID with Anti-Windup verified.\n");
    return 0;
}